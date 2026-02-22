#include "config_manager.h"
#include "config_schema.h"
#include "filesystem.h"

#include "cJSON.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define TAG            "config_mgr"
#define CONFIG_FILE    FS_BASE_PATH "/config.json"
#define MAX_JSON_BYTES 4096

static cJSON             *s_config = NULL;
static SemaphoreHandle_t  s_mutex  = NULL;

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

/**
 * Walk the schema and add any key that is absent from s_config using its
 * default value.  Returns true if at least one key was added.
 *
 * Caller must hold s_mutex (or call during single-threaded init).
 */
static bool apply_defaults_locked(void)
{
    bool added = false;

    for (size_t i = 0; i < g_config_schema_count; i++)
    {
        const config_schema_entry_t *e = &g_config_schema[i];

        if (cJSON_GetObjectItem(s_config, e->key) != NULL)
        {
            continue; /* already present */
        }

        switch (e->type)
        {
            case CONFIG_TYPE_STRING:
                cJSON_AddStringToObject(s_config, e->key,
                                        e->default_str ? e->default_str : "");
                break;

            case CONFIG_TYPE_INT:
                cJSON_AddNumberToObject(s_config, e->key, e->default_int);
                break;

            case CONFIG_TYPE_BOOL:
                if (e->default_bool)
                {
                    cJSON_AddTrueToObject(s_config, e->key);
                }
                else
                {
                    cJSON_AddFalseToObject(s_config, e->key);
                }
                break;
        }

        added = true;
    }

    return added;
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

    fread(buf, 1, (size_t)file_size, f);
    buf[file_size] = '\0';
    fclose(f);

    cJSON *parsed = cJSON_Parse(buf);
    free(buf);

    if (parsed == NULL || !cJSON_IsObject(parsed))
    {
        ESP_LOGE(TAG, "Failed to parse config.json");
        cJSON_Delete(parsed);
        return ESP_FAIL;
    }

    cJSON_Delete(s_config);
    s_config = parsed;
    return ESP_OK;
}

/**
 * Serialise s_config and write it to CONFIG_FILE.
 *
 * Caller must hold s_mutex (or call during single-threaded init).
 */
static esp_err_t save_locked(void)
{
    char *json_str = cJSON_PrintUnformatted(s_config);
    if (json_str == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    FILE *f = fopen(CONFIG_FILE, "w");
    if (f == NULL)
    {
        free(json_str);
        ESP_LOGE(TAG, "Cannot open config.json for writing");
        return ESP_FAIL;
    }

    fputs(json_str, f);
    fclose(f);
    free(json_str);

    ESP_LOGI(TAG, "Config saved to " CONFIG_FILE);
    return ESP_OK;
}

/* ── public API ──────────────────────────────────────────────────────────── */

esp_err_t config_manager_init(void)
{
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
        save_locked();
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
            return config_set_int(key, (int)strtol(value_str, NULL, 10));

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
