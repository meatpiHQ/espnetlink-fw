/**
 * @file settings_manager_codec.c
 * @brief Self-validating envelope: {"crc32":N,"version":N,"data":{...}} + CRC-32.
 *        Pure logic, no filesystem — host-testable on the linux target.
 */
#include <stdlib.h>
#include <string.h>

#include "settings_manager_private.h"

uint32_t sm_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (int b = 0; b < 8; b++)
        {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }

    return ~crc;
}

esp_err_t sm_codec_encode(uint32_t version, const cJSON *data, char **out_str)
{
    if (data == NULL || out_str == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* CRC is taken over the serialized data object exactly as it will be
       re-serialized on decode, so the two always agree. */
    char *data_str = cJSON_PrintUnformatted(data);

    if (data_str == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    uint32_t crc = sm_crc32((const uint8_t *)data_str, strlen(data_str));
    free(data_str);

    cJSON *env = cJSON_CreateObject();

    if (env == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    cJSON *data_copy = cJSON_Duplicate(data, true);

    if (data_copy == NULL ||
        cJSON_AddNumberToObject(env, "crc32", (double)crc) == NULL ||
        cJSON_AddNumberToObject(env, "version", (double)version) == NULL)
    {
        cJSON_Delete(data_copy);
        cJSON_Delete(env);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddItemToObject(env, "data", data_copy);

    char *env_str = cJSON_PrintUnformatted(env);
    cJSON_Delete(env);

    if (env_str == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    *out_str = env_str;
    return ESP_OK;
}

esp_err_t sm_codec_decode(const char *envelope_str, uint32_t *version_out,
                          cJSON **data_out)
{
    if (envelope_str == NULL || data_out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *env = cJSON_Parse(envelope_str);

    if (env == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const cJSON *crc_item  = cJSON_GetObjectItemCaseSensitive(env, "crc32");
    const cJSON *ver_item  = cJSON_GetObjectItemCaseSensitive(env, "version");
    const cJSON *data_item = cJSON_GetObjectItemCaseSensitive(env, "data");

    if (!cJSON_IsNumber(crc_item) || !cJSON_IsObject(data_item))
    {
        cJSON_Delete(env);
        return ESP_ERR_INVALID_ARG;
    }

    char *data_str = cJSON_PrintUnformatted(data_item);

    if (data_str == NULL)
    {
        cJSON_Delete(env);
        return ESP_ERR_NO_MEM;
    }

    uint32_t calc   = sm_crc32((const uint8_t *)data_str, strlen(data_str));
    uint32_t stored = (uint32_t)crc_item->valuedouble;
    free(data_str);

    if (calc != stored)
    {
        cJSON_Delete(env);
        return ESP_ERR_INVALID_CRC;
    }

    /* Read the version BEFORE deleting env: ver_item is owned by env, and
       reading it afterwards is a use-after-free (found by ASan on the host). */
    uint32_t version = cJSON_IsNumber(ver_item) ? (uint32_t)ver_item->valuedouble : 0;

    cJSON *detached = cJSON_Duplicate(data_item, true);
    cJSON_Delete(env);

    if (detached == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    if (version_out != NULL)
    {
        *version_out = version;
    }

    *data_out = detached;
    return ESP_OK;
}
