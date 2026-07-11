/**
 * @file main_settings.c
 * @brief The "system" settings descriptor (device-wide keys that have no
 *        component home; today just device_name — the legacy DEVICE_NAME).
 */
#include <string.h>

#include "cJSON.h"
#include "settings_manager.h"

#include "main_compose.h"

static const settings_field_t FIELDS[] =
{
    SETTINGS_STR("device_name", 32, "ESPNetLink"), /* legacy DEVICE_NAME */
};

static char s_device_name[33] = "ESPNetLink";

const char *main_system_device_name(void)
{
    return s_device_name;
}

static esp_err_t on_apply(const cJSON *settings)
{
    const cJSON *name = cJSON_GetObjectItemCaseSensitive(settings,
                                                         "device_name");

    if (cJSON_IsString(name) && name->valuestring != NULL)
    {
        strlcpy(s_device_name, name->valuestring, sizeof(s_device_name));
    }

    return ESP_OK;
}

esp_err_t main_register_settings(void)
{
    static const settings_descriptor_t DESC =
    {
        .name        = "system",
        .version     = 1,
        .fields      = FIELDS,
        .field_count = sizeof(FIELDS) / sizeof(FIELDS[0]),
        .on_apply    = on_apply,
    };

    return settings_manager_register(&DESC);
}
