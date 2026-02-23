#include "config_manager.h"
#include "config_schema.h"
#include "filesystem.h"

#include "cJSON.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <limits.h>
#include <unistd.h>

#define TAG            "config_mgr"
#define CONFIG_FILE    FS_BASE_PATH "/config.json"
#define CONFIG_FILE_TMP FS_BASE_PATH "/config.json.tmp"
#define CONFIG_VERSION 1
#define CONFIG_JSON_KEY_VERSION "version"
#define CONFIG_JSON_KEY_CONFIG  "config"
#define CONFIG_JSON_KEY_CRC32   "crc32"
#define MAX_JSON_BYTES 4096

static cJSON             *s_config = NULL;
static SemaphoreHandle_t  s_mutex  = NULL;
static uint32_t           s_last_saved_crc = 0;
static bool               s_has_last_saved_crc = false;

/* ── internal helpers ────────────────────────────────────────────────────── */

static const config_schema_entry_t *find_entry(const char *key)
{
    for (size_t i = 0; i < g_config_schema_count; i++)
    {
        if (strcmp(g_config_schema[i].key, key) == 0)
        {
            return &g_config_schema[i];
        }
    }
    return NULL;
}

static void add_default_for_entry_locked(const config_schema_entry_t *entry)
{
    switch (entry->type)
    {
        case CONFIG_TYPE_STRING:
            cJSON_AddStringToObject(s_config, entry->key,
                                    entry->default_str ? entry->default_str : "");
            break;

        case CONFIG_TYPE_INT:
            cJSON_AddNumberToObject(s_config, entry->key, entry->default_int);
            break;

        case CONFIG_TYPE_BOOL:
            if (entry->default_bool)
            {
                cJSON_AddTrueToObject(s_config, entry->key);
            }
            else
            {
                cJSON_AddFalseToObject(s_config, entry->key);
            }
            break;
    }
}

static bool schema_item_valid(const config_schema_entry_t *entry, const cJSON *item)
{
    if (item == NULL)
    {
        return false;
    }

    switch (entry->type)
    {
        case CONFIG_TYPE_STRING:
            if (!cJSON_IsString(item) || item->valuestring == NULL)
            {
                return false;
            }
            return strlen(item->valuestring) <= entry->max_len;

        case CONFIG_TYPE_INT:
            return cJSON_IsNumber(item);

        case CONFIG_TYPE_BOOL:
            return cJSON_IsBool(item);
    }

    return false;
}

static esp_err_t compute_crc32_for_object(const cJSON *object, uint32_t *out_crc)
{
    if (object == NULL || out_crc == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    char *json_str = cJSON_PrintUnformatted((cJSON *)object);
    if (json_str == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    uint32_t crc = 0xFFFFFFFFU;
    crc = esp_rom_crc32_le(crc, (const uint8_t *)json_str, (uint32_t)strlen(json_str));
    crc ^= 0xFFFFFFFFU;

    free(json_str);
    *out_crc = crc;
    return ESP_OK;
}

/**
 * Walk the schema and ensure each key exists and matches schema constraints.
 * Missing or invalid values are replaced with defaults.
 * Returns true if at least one key was changed.
 *
 * Caller must hold s_mutex (or call during single-threaded init).
 */
static bool apply_defaults_locked(void)
{
    bool changed = false;

    for (size_t i = 0; i < g_config_schema_count; i++)
    {
        const config_schema_entry_t *e = &g_config_schema[i];
        cJSON *item = cJSON_GetObjectItemCaseSensitive(s_config, e->key);

        if (schema_item_valid(e, item))
        {
            continue;
        }

        if (item != NULL)
        {
            ESP_LOGW(TAG, "Invalid value for '%s' in config.json; restoring default", e->key);
        }

        cJSON_DeleteItemFromObjectCaseSensitive(s_config, e->key);
        add_default_for_entry_locked(e);
        changed = true;
    }

    return changed;
}

/**
 * Read and parse config.json into s_config.
 * On success s_config points to the parsed object.
 * On failure s_config is left unchanged.
 *
 * Caller must hold s_mutex (or call during single-threaded init).
 */
static esp_err_t load_file_locked(void)
{
    FILE *f = fopen(CONFIG_FILE, "r");
    if (f == NULL)
    {
        ESP_LOGW(TAG, "config.json not found — will use defaults");
        return ESP_ERR_NOT_FOUND;
    }

    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    rewind(f);

    if (file_size <= 0 || file_size > MAX_JSON_BYTES)
    {
        ESP_LOGE(TAG, "config.json size invalid (%ld bytes)", file_size);
        fclose(f);
        return ESP_FAIL;
    }

    char *buf = malloc((size_t)file_size + 1);
    if (buf == NULL)
    {
        fclose(f);
        return ESP_ERR_NO_MEM;
    }

    size_t bytes_read = fread(buf, 1, (size_t)file_size, f);
    buf[file_size] = '\0';

    if (bytes_read != (size_t)file_size)
    {
        ESP_LOGE(TAG, "Failed to read config.json (%u/%u bytes)",
                 (unsigned)bytes_read, (unsigned)file_size);
        free(buf);
        fclose(f);
        return ESP_FAIL;
    }

    if (fclose(f) != 0)
    {
        ESP_LOGE(TAG, "Failed to close config.json after read");
        free(buf);
        return ESP_FAIL;
    }

    cJSON *parsed = cJSON_Parse(buf);
    free(buf);

    if (parsed == NULL || !cJSON_IsObject(parsed))
    {
        ESP_LOGE(TAG, "Failed to parse config.json");
        cJSON_Delete(parsed);
        return ESP_FAIL;
    }

    cJSON *version_item = cJSON_GetObjectItemCaseSensitive(parsed, CONFIG_JSON_KEY_VERSION);
    cJSON *config_item  = cJSON_GetObjectItemCaseSensitive(parsed, CONFIG_JSON_KEY_CONFIG);
    cJSON *crc_item     = cJSON_GetObjectItemCaseSensitive(parsed, CONFIG_JSON_KEY_CRC32);

    if (!cJSON_IsNumber(version_item) ||
        (int)version_item->valuedouble != CONFIG_VERSION ||
        !cJSON_IsObject(config_item) ||
        !cJSON_IsNumber(crc_item))
    {
        ESP_LOGE(TAG, "Invalid config.json wrapper format");
        cJSON_Delete(parsed);
        return ESP_FAIL;
    }

    double crc_field = crc_item->valuedouble;
    if (crc_field < 0.0 || crc_field > 4294967295.0)
    {
        ESP_LOGE(TAG, "Invalid crc32 value in config.json");
        cJSON_Delete(parsed);
        return ESP_FAIL;
    }

    uint32_t stored_crc = (uint32_t)crc_field;
    uint32_t computed_crc = 0;
    esp_err_t crc_ret = compute_crc32_for_object(config_item, &computed_crc);
    if (crc_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to compute config CRC32");
        cJSON_Delete(parsed);
        return crc_ret;
    }

    if (computed_crc != stored_crc)
    {
        ESP_LOGE(TAG, "config.json CRC mismatch (stored=0x%08X computed=0x%08X)",
                 stored_crc, computed_crc);
        cJSON_Delete(parsed);
        return ESP_FAIL;
    }

    cJSON *config_obj = cJSON_DetachItemFromObjectCaseSensitive(parsed, CONFIG_JSON_KEY_CONFIG);
    if (config_obj == NULL || !cJSON_IsObject(config_obj))
    {
        ESP_LOGE(TAG, "Failed to extract config object from config.json");
        cJSON_Delete(config_obj);
        cJSON_Delete(parsed);
        return ESP_FAIL;
    }

    cJSON_Delete(parsed);

    cJSON_Delete(s_config);
    s_config = config_obj;
    s_last_saved_crc = stored_crc;
    s_has_last_saved_crc = true;
    return ESP_OK;
}

/**
 * Serialise s_config and write it to CONFIG_FILE.
 *
 * Caller must hold s_mutex (or call during single-threaded init).
 */
static esp_err_t save_locked(void)
{
    uint32_t config_crc = 0;
    esp_err_t crc_ret = compute_crc32_for_object(s_config, &config_crc);
    if (crc_ret != ESP_OK)
    {
        return crc_ret;
    }

    if (s_has_last_saved_crc && config_crc == s_last_saved_crc)
    {
        ESP_LOGI(TAG, "Config unchanged; skip flash write");
        return ESP_OK;
    }

    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    cJSON *config_copy = cJSON_Duplicate(s_config, true);
    if (config_copy == NULL)
    {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(root, CONFIG_JSON_KEY_VERSION, CONFIG_VERSION);
    cJSON_AddItemToObject(root, CONFIG_JSON_KEY_CONFIG, config_copy);
    cJSON_AddNumberToObject(root, CONFIG_JSON_KEY_CRC32, (double)config_crc);

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json_str == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    FILE *f = fopen(CONFIG_FILE_TMP, "w");
    if (f == NULL)
    {
        free(json_str);
        ESP_LOGE(TAG, "Cannot open temp config file for writing");
        return ESP_FAIL;
    }

    size_t json_len = strlen(json_str);
    size_t bytes_written = fwrite(json_str, 1, json_len, f);
    if (bytes_written != json_len)
    {
        ESP_LOGE(TAG, "Short write to temp config file (%u/%u bytes)",
                 (unsigned)bytes_written, (unsigned)json_len);
        fclose(f);
        remove(CONFIG_FILE_TMP);
        free(json_str);
        return ESP_FAIL;
    }

    if (fflush(f) != 0)
    {
        ESP_LOGE(TAG, "fflush failed for temp config file");
        fclose(f);
        remove(CONFIG_FILE_TMP);
        free(json_str);
        return ESP_FAIL;
    }

    if (fsync(fileno(f)) != 0)
    {
        ESP_LOGE(TAG, "fsync failed for temp config file");
        fclose(f);
        remove(CONFIG_FILE_TMP);
        free(json_str);
        return ESP_FAIL;
    }

    if (fclose(f) != 0)
    {
        ESP_LOGE(TAG, "Failed to close temp config file");
        remove(CONFIG_FILE_TMP);
        free(json_str);
        return ESP_FAIL;
    }

    if (rename(CONFIG_FILE_TMP, CONFIG_FILE) != 0)
    {
        ESP_LOGE(TAG, "Failed to atomically replace config.json");
        remove(CONFIG_FILE_TMP);
        free(json_str);
        return ESP_FAIL;
    }

    free(json_str);
    s_last_saved_crc = config_crc;
    s_has_last_saved_crc = true;

    ESP_LOGI(TAG, "Config saved to " CONFIG_FILE " (crc32=0x%08X)", config_crc);
    return ESP_OK;
}

/* ── public API ──────────────────────────────────────────────────────────── */

esp_err_t config_manager_init(void)
{
    s_has_last_saved_crc = false;
    s_last_saved_crc = 0;

    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    s_config = cJSON_CreateObject();
    if (s_config == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    bool need_save = false;
    esp_err_t ret  = load_file_locked();

    if (ret == ESP_ERR_NOT_FOUND || ret == ESP_FAIL)
    {
        /* Start fresh — apply all defaults and persist. */
        cJSON_Delete(s_config);
        s_config  = cJSON_CreateObject();
        need_save = true;
    }

    /* Fill any keys missing from the loaded file (e.g. new firmware keys). */
    bool defaults_added = apply_defaults_locked();

    if (need_save || defaults_added)
    {
        ret = save_locked();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to save initial config: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    ESP_LOGI(TAG, "Initialised — %u schema keys", (unsigned)g_config_schema_count);
    return ESP_OK;
}

esp_err_t config_save(void)
{
    if (s_config == NULL || s_mutex == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    esp_err_t ret = save_locked();
    xSemaphoreGive(s_mutex);
    return ret;
}

esp_err_t config_reset_defaults(void)
{
    if (s_config == NULL || s_mutex == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    cJSON_Delete(s_config);
    s_config = cJSON_CreateObject();
    apply_defaults_locked();
    esp_err_t ret = save_locked();
    xSemaphoreGive(s_mutex);
    return ret;
}

/* ── string ──────────────────────────────────────────────────────────────── */

esp_err_t config_get_str(const char *key, char *out, size_t out_len)
{
    if (key == NULL || out == NULL || out_len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_config == NULL || s_mutex == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const config_schema_entry_t *entry = find_entry(key);
    if (entry == NULL)
    {
        ESP_LOGW(TAG, "config_get_str: unknown key '%s'", key);
        return ESP_ERR_NOT_FOUND;
    }
    if (entry->type != CONFIG_TYPE_STRING)
    {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    cJSON *item = cJSON_GetObjectItem(s_config, key);
    esp_err_t ret;

    if (item == NULL || !cJSON_IsString(item))
    {
        ret = ESP_ERR_NOT_FOUND;
    }
    else
    {
        strlcpy(out, item->valuestring, out_len);
        ret = ESP_OK;
    }

    xSemaphoreGive(s_mutex);
    return ret;
}

esp_err_t config_set_str(const char *key, const char *value)
{
    if (key == NULL || value == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_config == NULL || s_mutex == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const config_schema_entry_t *entry = find_entry(key);
    if (entry == NULL)
    {
        ESP_LOGW(TAG, "config_set_str: unknown key '%s'", key);
        return ESP_ERR_NOT_FOUND;
    }
    if (entry->type != CONFIG_TYPE_STRING)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (strlen(value) > entry->max_len)
    {
        ESP_LOGE(TAG, "'%s' value too long (%u > %u)",
                 key, (unsigned)strlen(value), (unsigned)entry->max_len);
        return ESP_ERR_INVALID_SIZE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    cJSON_DeleteItemFromObject(s_config, key);
    cJSON_AddStringToObject(s_config, key, value);
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

/* ── int ─────────────────────────────────────────────────────────────────── */

esp_err_t config_get_int(const char *key, int *out)
{
    if (key == NULL || out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_config == NULL || s_mutex == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const config_schema_entry_t *entry = find_entry(key);
    if (entry == NULL)
    {
        ESP_LOGW(TAG, "config_get_int: unknown key '%s'", key);
        return ESP_ERR_NOT_FOUND;
    }
    if (entry->type != CONFIG_TYPE_INT)
    {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    cJSON *item = cJSON_GetObjectItem(s_config, key);
    esp_err_t ret;

    if (item == NULL || !cJSON_IsNumber(item))
    {
        ret = ESP_ERR_NOT_FOUND;
    }
    else
    {
        *out = (int)item->valuedouble;
        ret  = ESP_OK;
    }

    xSemaphoreGive(s_mutex);
    return ret;
}

esp_err_t config_set_int(const char *key, int value)
{
    if (key == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_config == NULL || s_mutex == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const config_schema_entry_t *entry = find_entry(key);
    if (entry == NULL)
    {
        ESP_LOGW(TAG, "config_set_int: unknown key '%s'", key);
        return ESP_ERR_NOT_FOUND;
    }
    if (entry->type != CONFIG_TYPE_INT)
    {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    cJSON_DeleteItemFromObject(s_config, key);
    cJSON_AddNumberToObject(s_config, key, value);
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

/* ── bool ────────────────────────────────────────────────────────────────── */

esp_err_t config_get_bool(const char *key, bool *out)
{
    if (key == NULL || out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_config == NULL || s_mutex == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const config_schema_entry_t *entry = find_entry(key);
    if (entry == NULL)
    {
        ESP_LOGW(TAG, "config_get_bool: unknown key '%s'", key);
        return ESP_ERR_NOT_FOUND;
    }
    if (entry->type != CONFIG_TYPE_BOOL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    cJSON *item = cJSON_GetObjectItem(s_config, key);
    esp_err_t ret;

    if (item == NULL || !cJSON_IsBool(item))
    {
        ret = ESP_ERR_NOT_FOUND;
    }
    else
    {
        *out = cJSON_IsTrue(item);
        ret  = ESP_OK;
    }

    xSemaphoreGive(s_mutex);
    return ret;
}

/* ── generic helpers ─────────────────────────────────────────────────────── */

esp_err_t config_get_as_str(const char *key, char *out, size_t out_len)
{
    if (key == NULL || out == NULL || out_len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const config_schema_entry_t *entry = find_entry(key);
    if (entry == NULL)
    {
        return ESP_ERR_NOT_FOUND;
    }

    switch (entry->type)
    {
        case CONFIG_TYPE_STRING:
            return config_get_str(key, out, out_len);

        case CONFIG_TYPE_INT:
        {
            int val;
            esp_err_t ret = config_get_int(key, &val);
            if (ret == ESP_OK)
            {
                snprintf(out, out_len, "%d", val);
            }
            return ret;
        }

        case CONFIG_TYPE_BOOL:
        {
            bool val;
            esp_err_t ret = config_get_bool(key, &val);
            if (ret == ESP_OK)
            {
                strlcpy(out, val ? "true" : "false", out_len);
            }
            return ret;
        }
    }

    return ESP_FAIL;
}

esp_err_t config_set_from_str(const char *key, const char *value_str)
{
    if (key == NULL || value_str == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const config_schema_entry_t *entry = find_entry(key);
    if (entry == NULL)
    {
        ESP_LOGW(TAG, "config_set_from_str: unknown key '%s'", key);
        return ESP_ERR_NOT_FOUND;
    }

    switch (entry->type)
    {
        case CONFIG_TYPE_STRING:
            return config_set_str(key, value_str);

        case CONFIG_TYPE_INT:
        {
            char *end_ptr = NULL;
            errno = 0;
            long parsed = strtol(value_str, &end_ptr, 10);

            if (value_str[0] == '\0' || end_ptr == NULL || *end_ptr != '\0')
            {
                ESP_LOGE(TAG, "'%s': invalid int value '%s'", key, value_str);
                return ESP_ERR_INVALID_ARG;
            }

            if (errno == ERANGE || parsed < INT_MIN || parsed > INT_MAX)
            {
                ESP_LOGE(TAG, "'%s': int out of range '%s'", key, value_str);
                return ESP_ERR_INVALID_ARG;
            }

            return config_set_int(key, (int)parsed);
        }

        case CONFIG_TYPE_BOOL:
        {
            bool val;
            if (strcmp(value_str, "1")    == 0 ||
                strcmp(value_str, "true") == 0 ||
                strcmp(value_str, "yes")  == 0)
            {
                val = true;
            }
            else if (strcmp(value_str, "0")     == 0 ||
                     strcmp(value_str, "false")  == 0 ||
                     strcmp(value_str, "no")     == 0)
            {
                val = false;
            }
            else
            {
                ESP_LOGE(TAG, "'%s': invalid bool value '%s'", key, value_str);
                return ESP_ERR_INVALID_ARG;
            }
            return config_set_bool(key, val);
        }
    }

    return ESP_FAIL;
}

/* ── bool ────────────────────────────────────────────────────────────────── */

esp_err_t config_set_bool(const char *key, bool value)
{
    if (key == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_config == NULL || s_mutex == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const config_schema_entry_t *entry = find_entry(key);
    if (entry == NULL)
    {
        ESP_LOGW(TAG, "config_set_bool: unknown key '%s'", key);
        return ESP_ERR_NOT_FOUND;
    }
    if (entry->type != CONFIG_TYPE_BOOL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    cJSON_DeleteItemFromObject(s_config, key);

    if (value)
    {
        cJSON_AddTrueToObject(s_config, key);
    }
    else
    {
        cJSON_AddFalseToObject(s_config, key);
    }

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}
