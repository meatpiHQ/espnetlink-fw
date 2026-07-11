/**
 * @file test_storage_stub.c
 * @brief In-memory sm_storage_* for host tests. Uses the real codec so version
 *        and CRC behavior match the target exactly.
 */
#include <stdlib.h>
#include <string.h>

#include "settings_manager_private.h"
#include "test_storage_stub.h"

#define STUB_MAX_FILES 8
#define STUB_NAME_MAX  33

typedef struct
{
    char  name[STUB_NAME_MAX];
    char *envelope;
    bool  used;
} stub_file_t;

static stub_file_t s_files[STUB_MAX_FILES];
static int         s_save_count;

static stub_file_t *stub_find(const char *name)
{
    for (int i = 0; i < STUB_MAX_FILES; i++)
    {
        if (s_files[i].used && strcmp(s_files[i].name, name) == 0)
        {
            return &s_files[i];
        }
    }

    return NULL;
}

static stub_file_t *stub_find_or_alloc(const char *name)
{
    stub_file_t *f = stub_find(name);

    if (f != NULL)
    {
        return f;
    }

    for (int i = 0; i < STUB_MAX_FILES; i++)
    {
        if (!s_files[i].used)
        {
            s_files[i].used = true;
            strncpy(s_files[i].name, name, STUB_NAME_MAX - 1);
            s_files[i].name[STUB_NAME_MAX - 1] = '\0';
            s_files[i].envelope = NULL;
            return &s_files[i];
        }
    }

    return NULL;
}

void sm_stub_reset(void)
{
    for (int i = 0; i < STUB_MAX_FILES; i++)
    {
        free(s_files[i].envelope);
        s_files[i].envelope = NULL;
        s_files[i].used     = false;
    }

    s_save_count = 0;
}

void sm_stub_put(const char *name, uint32_t version, const char *data_json)
{
    cJSON *data = cJSON_Parse(data_json);
    char  *env  = NULL;

    if (data != NULL && sm_codec_encode(version, data, &env) == ESP_OK)
    {
        stub_file_t *f = stub_find_or_alloc(name);

        if (f != NULL)
        {
            free(f->envelope);
            f->envelope = env;
            env = NULL;
        }
    }

    free(env);
    cJSON_Delete(data);
}

void sm_stub_corrupt(const char *name)
{
    stub_file_t *f = stub_find(name);

    if (f != NULL && f->envelope != NULL)
    {
        size_t len = strlen(f->envelope);
        f->envelope[len / 2] ^= 0x5A;
    }
}

int sm_stub_save_count(void)
{
    return s_save_count;
}

cJSON *sm_stub_read(const char *name, uint32_t *version_out)
{
    stub_file_t *f = stub_find(name);

    if (f == NULL || f->envelope == NULL)
    {
        return NULL;
    }

    cJSON *data = NULL;

    if (sm_codec_decode(f->envelope, version_out, &data) != ESP_OK)
    {
        return NULL;
    }

    return data;
}

/* ---- sm_storage_* implementation ------------------------------------------ */

esp_err_t sm_storage_mount(void)
{
    return ESP_OK;
}

esp_err_t sm_storage_unmount(void)
{
    return ESP_OK;
}

esp_err_t sm_storage_load(const char *name, uint32_t *version_out, cJSON **data_out)
{
    stub_file_t *f = stub_find(name);

    if (f == NULL || f->envelope == NULL)
    {
        return ESP_ERR_NOT_FOUND;
    }

    return sm_codec_decode(f->envelope, version_out, data_out);
}

esp_err_t sm_storage_save(const char *name, uint32_t version, const cJSON *data)
{
    char *env = NULL;
    esp_err_t err = sm_codec_encode(version, data, &env);

    if (err != ESP_OK)
    {
        return err;
    }

    stub_file_t *f = stub_find_or_alloc(name);

    if (f == NULL)
    {
        free(env);
        return ESP_ERR_NO_MEM;
    }

    free(f->envelope);
    f->envelope = env;
    s_save_count++;
    return ESP_OK;
}

bool sm_storage_file_exists(const char *name)
{
    (void)name;
    return true;   /* format:"file" checks always pass on the host */
}
