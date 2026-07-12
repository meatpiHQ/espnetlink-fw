/**
 * @file api_http_fs.c
 * @brief The UI file manager (filesystem/HTTP_API.md): browse (list/info),
 *        DOWNLOAD any file (logs, configs — streamed), UPLOAD any file
 *        (multipart from the HTML form or raw body; atomic via
 *        filesystem_write), delete, mkdir. Write surface enabled
 *        2026-07-04 (meatpi's call — v1 AP-mode trust model, auth later).
 *
 * Path safety is the filesystem component's own validation (no "..",
 * known prefixes); the settings partition is not reachable through it.
 */
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "filesystem.h"
#include "http_server_manager.h"
#include "multipart_upload.h"

#include "api_http_private.h"

static const char *TAG = "api_http";

#define FS_CHUNK        1024              /* download chunk (internal §2)  */

/* internal: LittleFS reads run with flash cache constraints (§2) */
static char s_chunk[FS_CHUNK];
static SemaphoreHandle_t s_chunk_lock;
static StaticSemaphore_t s_chunk_lock_buf; /* internal: FreeRTOS object */

/* ---- shared helpers ------------------------------------------------------------ */

static esp_err_t fs_query_path(httpd_req_t *req, char *path, size_t path_len)
{
    char query[192];

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "path", path, path_len) != ESP_OK ||
        path[0] == '\0')
    {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static esp_err_t fs_error(httpd_req_t *req, esp_err_t err)
{
    if (err == ESP_ERR_NOT_FOUND)
    {
        return api_send_error(req, "404 Not Found", "not found");
    }

    if (err == ESP_ERR_INVALID_STATE)
    {
        return api_send_error(req, "503 Service Unavailable",
                              "backend unavailable");
    }

    return api_send_error(req, "400 Bad Request", "invalid path");
}

static esp_err_t fs_send_ok(httpd_req_t *req)
{
    cJSON *resp = cJSON_CreateObject();

    cJSON_AddBoolToObject(resp, "ok", true);
    return api_send_json(req, resp);
}

/* ---- browse (list / info) -------------------------------------------------------- */

static esp_err_t fs_list_cb(const char *name, bool is_dir, size_t size,
                            void *ctx)
{
    cJSON *entries = ctx;
    cJSON *entry = cJSON_CreateObject();

    cJSON_AddStringToObject(entry, "name", name);
    cJSON_AddBoolToObject(entry, "dir", is_dir);
    cJSON_AddNumberToObject(entry, "size", (double)size);
    cJSON_AddItemToArray(entries, entry);
    return ESP_OK;
}

static esp_err_t fs_list_handler(httpd_req_t *req)
{
    char path[128];

    if (fs_query_path(req, path, sizeof(path)) != ESP_OK)
    {
        return api_send_error(req, "400 Bad Request", "invalid path");
    }

    cJSON *resp = cJSON_CreateObject();

    cJSON_AddStringToObject(resp, "path", path);

    cJSON *entries = cJSON_AddArrayToObject(resp, "entries");
    esp_err_t err = filesystem_list(path, fs_list_cb, entries);

    if (err != ESP_OK)
    {
        cJSON_Delete(resp);
        return fs_error(req, err);
    }

    return api_send_json(req, resp);
}

static esp_err_t fs_info_handler(httpd_req_t *req)
{
    char path[128];

    if (fs_query_path(req, path, sizeof(path)) != ESP_OK)
    {
        return api_send_error(req, "400 Bad Request", "invalid path");
    }

    size_t total = 0;
    size_t used = 0;
    esp_err_t err = filesystem_info(path, &total, &used);

    if (err != ESP_OK)
    {
        return fs_error(req, err);
    }

    cJSON *resp = cJSON_CreateObject();

    cJSON_AddNumberToObject(resp, "total", (double)total);
    cJSON_AddNumberToObject(resp, "used", (double)used);
    return api_send_json(req, resp);
}

/* ---- download: GET /api/fs/download?path=/data/x --------------------------------- */

static esp_err_t fs_download_handler(httpd_req_t *req)
{
    char path[128];

    if (fs_query_path(req, path, sizeof(path)) != ESP_OK)
    {
        return api_send_error(req, "400 Bad Request", "invalid path");
    }

    FILE *f = filesystem_open(path, "rb");

    if (f == NULL)
    {
        return fs_error(req, filesystem_exists(path) ? ESP_ERR_INVALID_ARG
                                                     : ESP_ERR_NOT_FOUND);
    }

    /* save-as filename = the basename */
    const char *base = strrchr(path, '/');
    char disposition[160];

    snprintf(disposition, sizeof(disposition),
             "attachment; filename=\"%s\"",
             (base != NULL) ? base + 1 : path);
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", disposition);

    esp_err_t err = ESP_OK;

    xSemaphoreTake(s_chunk_lock, portMAX_DELAY);

    size_t n;

    while (err == ESP_OK &&
           (n = fread(s_chunk, 1, sizeof(s_chunk), f)) > 0)
    {
        err = httpd_resp_send_chunk(req, s_chunk, (ssize_t)n);
    }

    xSemaphoreGive(s_chunk_lock);
    fclose(f);

    if (err == ESP_OK)
    {
        err = httpd_resp_send_chunk(req, NULL, 0); /* end of stream */
    }

    return err;
}

/* ---- upload: POST /api/fs/upload?path=/data/x ------------------------------------- */

/* STREAMED through filesystem's async pipe (writer task): bytes are
 * handed off as they arrive and media writes overlap the next receive —
 * no size cap beyond free space, and the same ATOMIC temp+rename
 * guarantee (a power cut mid-upload never leaves a torn file). One
 * stream at a time (a concurrent download/upload gets 503). */
typedef struct
{
    filesystem_wstream_t *ws;
    size_t len;
    bool got_file;
    esp_err_t err;
} fs_upload_ctx_t;

static bool fs_part_begin(const multipart_part_info_t *info, void *user_ctx)
{
    fs_upload_ctx_t *ctx = user_ctx;

    if (ctx->got_file)
    {
        return false; /* one file per request */
    }

    (void)info;
    ctx->got_file = true;
    return true;
}

static esp_err_t fs_part_data(const char *data, size_t len, void *user_ctx)
{
    fs_upload_ctx_t *ctx = user_ctx;

    if (len == 0 || ctx->err != ESP_OK)
    {
        return ctx->err;
    }

    /* memcpy-into-slot only in parser context; the fs writer task does
     * the media writes concurrently */
    ctx->err = filesystem_write_async_chunk(ctx->ws, data, len);
    ctx->len += len;
    return ctx->err;
}

static esp_err_t fs_upload_handler(httpd_req_t *req)
{
    char path[128];

    if (fs_query_path(req, path, sizeof(path)) != ESP_OK)
    {
        return api_send_error(req, "400 Bad Request",
                              "invalid path (POST /api/fs/upload?path=...)");
    }

    fs_upload_ctx_t ctx = { .err = ESP_OK };
    esp_err_t err = filesystem_write_open(path, &ctx.ws);

    if (err == ESP_OK)
    {
        err = filesystem_write_async_begin(ctx.ws);

        if (err != ESP_OK)
        {
            filesystem_write_abort(ctx.ws);
        }
    }

    if (err != ESP_OK)
    {
        return api_send_error(req, (err == ESP_ERR_INVALID_STATE)
                                       ? "503 Service Unavailable"
                                       : "500 Internal Server Error",
                              (err == ESP_ERR_INVALID_STATE)
                                  ? "another transfer is in progress"
                                  : "stream open failed");
    }

    char ct[96] = "";

    httpd_req_get_hdr_value_str(req, "Content-Type", ct, sizeof(ct));

    if (strncasecmp(ct, "multipart/", 10) == 0)
    {
        static const multipart_upload_handlers_t HANDLERS =
        {
            .on_part_begin = fs_part_begin,
            .on_part_data = fs_part_data,
        };
        multipart_upload_config_t cfg = { .rx_buf_size = 4096 };

        err = multipart_upload_handle(req, &HANDLERS, &ctx, &cfg);

        if (err == ESP_OK && !ctx.got_file)
        {
            err = ESP_FAIL;
        }
    }
    else
    {
        /* raw body: recv straight into the pipe, chunk by chunk */
        size_t remaining = req->content_len;

        err = (remaining == 0) ? ESP_FAIL : ESP_OK;
        ctx.got_file = (remaining > 0);

        while (err == ESP_OK && ctx.len < remaining)
        {
            size_t want = remaining - ctx.len;

            if (want > sizeof(s_chunk))
            {
                want = sizeof(s_chunk);
            }

            xSemaphoreTake(s_chunk_lock, portMAX_DELAY);

            int r = httpd_req_recv(req, s_chunk, (int)want);

            if (r > 0)
            {
                err = filesystem_write_async_chunk(ctx.ws, s_chunk,
                                                   (size_t)r);
                ctx.len += (size_t)r;
            }
            else if (r != HTTPD_SOCK_ERR_TIMEOUT)
            {
                err = ESP_FAIL;
            }

            xSemaphoreGive(s_chunk_lock);
        }
    }

    if (err == ESP_OK && ctx.err != ESP_OK)
    {
        err = ctx.err;
    }

    esp_err_t end_err = filesystem_write_async_end(ctx.ws,
                                                   err == ESP_OK);

    if (err == ESP_OK)
    {
        err = end_err;
    }

    if (err != ESP_OK)
    {
        return api_send_error(req, "400 Bad Request",
                              "empty/interrupted body or write failure");
    }

    ESP_LOGI(TAG, "uploaded %u bytes -> %s", (unsigned)ctx.len, path);

    cJSON *resp = cJSON_CreateObject();

    cJSON_AddBoolToObject(resp, "ok", true);
    cJSON_AddNumberToObject(resp, "size", (double)ctx.len);
    cJSON_AddStringToObject(resp, "path", path);
    return api_send_json(req, resp);
}

/* ---- delete + mkdir ----------------------------------------------------------------- */

static esp_err_t fs_delete_handler(httpd_req_t *req)
{
    char path[128];

    if (fs_query_path(req, path, sizeof(path)) != ESP_OK)
    {
        return api_send_error(req, "400 Bad Request", "invalid path");
    }

    esp_err_t err = filesystem_delete(path);

    if (err != ESP_OK)
    {
        return fs_error(req, err);
    }

    ESP_LOGI(TAG, "deleted %s", path);
    return fs_send_ok(req);
}

static esp_err_t fs_mkdir_handler(httpd_req_t *req)
{
    char path[128];

    if (fs_query_path(req, path, sizeof(path)) != ESP_OK)
    {
        return api_send_error(req, "400 Bad Request", "invalid path");
    }

    esp_err_t err = filesystem_mkdirs(path);

    if (err != ESP_OK)
    {
        return fs_error(req, err);
    }

    return fs_send_ok(req);
}

/* ---- registration --------------------------------------------------------------------- */

esp_err_t api_http_register_fs(void)
{
    static const httpd_uri_t URIS[] =
    {
        { .uri = "/api/fs/list", .method = HTTP_GET,
          .handler = fs_list_handler },
        { .uri = "/api/fs/info", .method = HTTP_GET,
          .handler = fs_info_handler },
        { .uri = "/api/fs/download", .method = HTTP_GET,
          .handler = fs_download_handler },
        { .uri = "/api/fs/upload", .method = HTTP_POST,
          .handler = fs_upload_handler },
        { .uri = "/api/fs/file", .method = HTTP_DELETE,
          .handler = fs_delete_handler },
        { .uri = "/api/fs/mkdir", .method = HTTP_POST,
          .handler = fs_mkdir_handler },
    };

    if (s_chunk_lock == NULL)
    {
        s_chunk_lock = xSemaphoreCreateMutexStatic(&s_chunk_lock_buf);
    }

    return http_server_manager_register_handlers(
        URIS, sizeof(URIS) / sizeof(URIS[0]));
}
