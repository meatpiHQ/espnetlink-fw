/**
 * @file settings_manager_storage.c
 * @brief LittleFS ownership + atomic settings file IO. Target-only.
 *
 * The Settings Manager is the only code that mounts or touches the settings FS.
 * Writes use temp-file-plus-rename so a power loss mid-write leaves the previous
 * good file intact (rollback-on-corruption, Coding Standard §7).
 */
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_heap_caps.h"
#include "esp_littlefs.h"
#include "esp_log.h"

#include "settings_manager_private.h"

#define SM_PARTITION_LABEL "settings"
#define SM_BASE_PATH       "/settings"
#define SM_DIR             SM_BASE_PATH "/cfg"
#define SM_PATH_MAX        128

static const char *TAG = "settings_manager";
static bool s_mounted;

static void build_path(char *buf, size_t buf_len, const char *name, const char *ext)
{
    snprintf(buf, buf_len, "%s/%s%s", SM_DIR, name, ext);
}

esp_err_t sm_storage_mount(void)
{
    if (s_mounted)
    {
        return ESP_OK;
    }

    esp_vfs_littlefs_conf_t conf =
    {
        .base_path              = SM_BASE_PATH,
        .partition_label        = SM_PARTITION_LABEL,
        .format_if_mount_failed = true,
        .dont_mount             = false,
    };

    esp_err_t err = esp_vfs_littlefs_register(&conf);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "littlefs mount failed: %s", esp_err_to_name(err));
        return err;
    }

    mkdir(SM_DIR, 0775);   /* harmless if it already exists */
    s_mounted = true;
    return ESP_OK;
}

esp_err_t sm_storage_unmount(void)
{
    if (!s_mounted)
    {
        return ESP_OK;
    }

    esp_err_t err = esp_vfs_littlefs_unregister(SM_PARTITION_LABEL);
    s_mounted = false;
    return err;
}

/* Read an entire file into a newly malloc'd, NUL-terminated buffer. */
static esp_err_t read_file(const char *path, char **out)
{
    FILE *f = fopen(path, "rb");

    if (f == NULL)
    {
        return ESP_ERR_NOT_FOUND;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size < 0)
    {
        fclose(f);
        return ESP_FAIL;
    }

    char *buf = malloc((size_t)size + 1);

    if (buf == NULL)
    {
        fclose(f);
        return ESP_ERR_NO_MEM;
    }

    size_t read = fread(buf, 1, (size_t)size, f);
    fclose(f);
    buf[read] = '\0';

    *out = buf;
    return ESP_OK;
}

esp_err_t sm_storage_load(const char *name, uint32_t *version_out, cJSON **data_out)
{
    if (name == NULL || data_out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    char path[SM_PATH_MAX];
    build_path(path, sizeof(path), name, ".json");

    char *raw = NULL;
    esp_err_t err = read_file(path, &raw);

    if (err != ESP_OK)
    {
        return err;   /* NOT_FOUND on first boot is expected */
    }

    err = sm_codec_decode(raw, version_out, data_out);
    free(raw);
    return err;
}

esp_err_t sm_storage_save(const char *name, uint32_t version, const cJSON *data)
{
    if (name == NULL || data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    char *env = NULL;
    esp_err_t err = sm_codec_encode(version, data, &env);

    if (err != ESP_OK)
    {
        return err;
    }

    size_t env_len = strlen(env);

    /* internal: the source buffer is read during the flash write (cache off). */
    char *scratch = heap_caps_malloc(env_len, MALLOC_CAP_INTERNAL);

    if (scratch == NULL)
    {
        free(env);
        return ESP_ERR_NO_MEM;
    }

    memcpy(scratch, env, env_len);
    free(env);

    char tmp_path[SM_PATH_MAX];
    char path[SM_PATH_MAX];
    build_path(tmp_path, sizeof(tmp_path), name, ".tmp");
    build_path(path, sizeof(path), name, ".json");

    FILE *f = fopen(tmp_path, "wb");

    if (f == NULL)
    {
        free(scratch);
        ESP_LOGE(TAG, "open %s failed", tmp_path);
        return ESP_FAIL;
    }

    size_t written = fwrite(scratch, 1, env_len, f);
    fflush(f);
    fsync(fileno(f));   /* force to media before rename: fflush alone only empties
                           stdio buffers, leaving a power-cut corruption window */
    fclose(f);
    free(scratch);

    if (written != env_len)
    {
        remove(tmp_path);
        return ESP_FAIL;
    }

    /* Atomic swap: the .json is either the old good file or the new one. */
    if (rename(tmp_path, path) != 0)
    {
        remove(tmp_path);
        ESP_LOGE(TAG, "rename %s -> %s failed", tmp_path, path);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t sm_storage_wipe(void)
{
    if (!s_mounted)
    {
        return ESP_ERR_INVALID_STATE;
    }

    DIR *dir = opendir(SM_DIR);

    if (dir == NULL)
    {
        return ESP_FAIL;
    }

    /* unlink everything in the cfg dir (json + any stray .tmp) —
       the next boot's load pass finds nothing and applies pure
       factory defaults */
    int removed = 0;
    struct dirent *entry;

    while ((entry = readdir(dir)) != NULL)
    {
        if (entry->d_type == DT_DIR)
        {
            continue;
        }

        char path[SM_PATH_MAX + 256]; /* dir + d_name can't truncate */

        snprintf(path, sizeof(path), "%s/%s", SM_DIR, entry->d_name);

        if (remove(path) == 0)
        {
            removed++;
        }
        else
        {
            ESP_LOGE(TAG, "factory reset: remove %s failed", path);
        }
    }

    closedir(dir);
    ESP_LOGW(TAG, "factory reset: %d settings file(s) deleted", removed);
    return ESP_OK;
}

bool sm_storage_file_exists(const char *name)
{
    if (name == NULL)
    {
        return false;
    }

    char path[SM_PATH_MAX];
    snprintf(path, sizeof(path), "%s/%s", SM_BASE_PATH, name);

    struct stat st;
    return stat(path, &st) == 0;
}
