/**
 * @file filesystem_stream.c
 * @brief Streaming atomic writes for big files (downloads, streamed
 *        uploads): open a TEMP sibling, write chunk by chunk, commit =
 *        fsync + rename — an interrupted stream never leaves a torn
 *        file. ONE stream at a time; chunk writes serialize with other
 *        fs operations per chunk (long streams don't starve them).
 */
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "filesystem.h"
#include "filesystem_private.h"

static const char *TAG = "filesystem";

struct filesystem_wstream
{
    FILE *f;
    char  final_path[FS_PATH_MAX];
    char  tmp_path[FS_PATH_MAX];
    bool  used;
};

static struct filesystem_wstream s_wstream; /* ONE stream at a time */

esp_err_t filesystem_write_open(const char *path,
                                filesystem_wstream_t **out)
{
    if (out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = fs_check_path(path, NULL);

    if (err != ESP_OK)
    {
        return err;
    }

    fs_lock();

    if (s_wstream.used)
    {
        fs_unlock();
        return ESP_ERR_INVALID_STATE; /* one stream at a time */
    }

    err = fs_path_temp_name(path, s_wstream.tmp_path,
                            sizeof(s_wstream.tmp_path));

    char parent[FS_PATH_MAX];

    if (err == ESP_OK &&
        fs_path_parent(path, parent, sizeof(parent)) == ESP_OK)
    {
        err = fs_mkdirs_in_lock(parent);
    }

    if (err == ESP_OK)
    {
        s_wstream.f = fopen(s_wstream.tmp_path, "wb");

        if (s_wstream.f == NULL)
        {
            ESP_LOGE(TAG, "stream open '%s' failed: errno %d",
                     s_wstream.tmp_path, errno);
            err = fs_errno_to_esp(errno);
        }
    }

    if (err == ESP_OK)
    {
        snprintf(s_wstream.final_path, sizeof(s_wstream.final_path), "%s",
                 path);
        s_wstream.used = true;
        *out = &s_wstream;
    }

    fs_unlock();
    return err;
}

esp_err_t filesystem_write_chunk(filesystem_wstream_t *ws,
                                 const void *data, size_t len)
{
    if (ws == NULL || !ws->used || (data == NULL && len > 0))
    {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t *src = data;
    esp_err_t err = ESP_OK;

    fs_lock(); /* per chunk: long streams don't starve other fs users */

    size_t scratch_size = 0;
    uint8_t *scratch = fs_scratch(&scratch_size);

    while (len > 0 && err == ESP_OK)
    {
        size_t chunk = (len > scratch_size) ? scratch_size : len;

        memcpy(scratch, src, chunk); /* PSRAM -> internal, cache on */

        if (fwrite(scratch, 1, chunk, ws->f) != chunk)
        {
            ESP_LOGE(TAG, "stream write '%s' failed: errno %d",
                     ws->final_path, errno);
            err = fs_errno_to_esp(errno);
        }

        src += chunk;
        len -= chunk;
    }

    fs_unlock();
    return err;
}

esp_err_t filesystem_write_commit(filesystem_wstream_t *ws)
{
    if (ws == NULL || !ws->used)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;

    fs_lock();
    fflush(ws->f);
    fsync(fileno(ws->f)); /* force to media before the rename */

    if (fclose(ws->f) != 0 ||
        rename(ws->tmp_path, ws->final_path) != 0)
    {
        ESP_LOGE(TAG, "stream commit '%s' failed: errno %d",
                 ws->final_path, errno);
        unlink(ws->tmp_path);
        err = ESP_FAIL;
    }

    ws->f = NULL;
    ws->used = false;
    fs_unlock();
    return err;
}

void filesystem_write_abort(filesystem_wstream_t *ws)
{
    if (ws == NULL || !ws->used)
    {
        return;
    }

    fs_lock();
    fclose(ws->f);
    unlink(ws->tmp_path);
    ws->f = NULL;
    ws->used = false;
    fs_unlock();
}

/* ---- the async pipe ---------------------------------------------------------- */

#define FS_ASYNC_SLOT_SIZE (32 * 1024)

typedef struct
{
    uint8_t data[FS_ASYNC_SLOT_SIZE];
    size_t  len;
} fs_async_slot_t;

static fs_async_slot_t s_slot[2] EXT_RAM_BSS_ATTR;
static SemaphoreHandle_t s_slot_filled;      /* producer -> writer      */
static StaticSemaphore_t s_slot_filled_buf;  /* internal: FreeRTOS obj  */
static SemaphoreHandle_t s_slot_free;        /* writer -> producer      */
static StaticSemaphore_t s_slot_free_buf;    /* internal: FreeRTOS obj  */
static SemaphoreHandle_t s_async_done;       /* writer: drained         */
static StaticSemaphore_t s_async_done_buf;   /* internal: FreeRTOS obj  */
static TaskHandle_t s_writer_task;
static StaticTask_t s_writer_tcb;            /* internal: FreeRTOS obj  */
static StackType_t s_writer_stack[4096];     /* internal: flash writes  */
static filesystem_wstream_t *volatile s_async_ws;
static volatile esp_err_t s_async_err;
static int s_prod_idx; /* producer/writer stay in lockstep forever      */

static void writer_task(void *arg)
{
    int idx = 0;

    (void)arg;

    while (true)
    {
        xSemaphoreTake(s_slot_filled, portMAX_DELAY);

        fs_async_slot_t *slot = &s_slot[idx];

        idx ^= 1;

        if (slot->len > 0)
        {
            if (s_async_err == ESP_OK)
            {
                s_async_err = filesystem_write_chunk(s_async_ws,
                                                     slot->data,
                                                     slot->len);
            }

            xSemaphoreGive(s_slot_free);
        }
        else
        {
            xSemaphoreGive(s_slot_free);
            xSemaphoreGive(s_async_done); /* len 0 sentinel = drained */
        }
    }
}

esp_err_t fs_stream_service_init(void)
{
    s_slot_filled = xSemaphoreCreateCountingStatic(2, 0,
                                                   &s_slot_filled_buf);
    s_slot_free = xSemaphoreCreateCountingStatic(2, 2, &s_slot_free_buf);
    s_async_done = xSemaphoreCreateBinaryStatic(&s_async_done_buf);
    s_writer_task = xTaskCreateStatic(writer_task, "fs_writer",
                                      sizeof(s_writer_stack) /
                                          sizeof(s_writer_stack[0]),
                                      NULL, 5, s_writer_stack,
                                      &s_writer_tcb);
    return (s_writer_task != NULL) ? ESP_OK : ESP_FAIL;
}

esp_err_t filesystem_write_async_begin(filesystem_wstream_t *ws)
{
    if (ws == NULL || !ws->used || s_writer_task == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_async_ws != NULL)
    {
        return ESP_ERR_INVALID_STATE; /* one pipe at a time */
    }

    s_async_err = ESP_OK;
    s_async_ws = ws;
    return ESP_OK;
}

esp_err_t filesystem_write_async_chunk(filesystem_wstream_t *ws,
                                       const void *data, size_t len)
{
    if (ws == NULL || ws != s_async_ws ||
        (data == NULL && len > 0))
    {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t *src = data;

    while (len > 0)
    {
        size_t piece = (len > FS_ASYNC_SLOT_SIZE) ? FS_ASYNC_SLOT_SIZE
                                                  : len;

        xSemaphoreTake(s_slot_free, portMAX_DELAY);
        memcpy(s_slot[s_prod_idx].data, src, piece);
        s_slot[s_prod_idx].len = piece;
        s_prod_idx ^= 1;
        xSemaphoreGive(s_slot_filled);
        src += piece;
        len -= piece;
    }

    return s_async_err; /* surface write errors early */
}

esp_err_t filesystem_write_async_end(filesystem_wstream_t *ws,
                                     bool commit)
{
    if (ws == NULL || ws != s_async_ws)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* len-0 sentinel, then wait until the writer has drained */
    xSemaphoreTake(s_slot_free, portMAX_DELAY);
    s_slot[s_prod_idx].len = 0;
    s_prod_idx ^= 1;
    xSemaphoreGive(s_slot_filled);
    xSemaphoreTake(s_async_done, portMAX_DELAY);

    esp_err_t err = s_async_err;

    s_async_ws = NULL;

    if (commit && err == ESP_OK)
    {
        return filesystem_write_commit(ws);
    }

    filesystem_write_abort(ws);
    return err; /* caller-requested abort with no write error = ESP_OK */
}
