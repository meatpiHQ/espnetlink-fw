/**
 * @file usb_cli_console_settings.c
 * @brief Minimal {cli} settings descriptor for usb_cli_console (§6b: a
 *        component with no other settings still gets the knob). The boot
 *        apply registers this transport's own commands (`echo`, `ota`).
 */
#include "cJSON.h"
#include "settings_manager.h"

#include "usb_cli_console_internal.h"

static const settings_field_t FIELDS[] =
{
    SETTINGS_BOOL("cli", true),
};

static esp_err_t on_apply(const cJSON *settings)
{
    const cJSON *cli = cJSON_GetObjectItemCaseSensitive(settings, "cli");

    if (!cJSON_IsFalse(cli))
    {
        usb_cli_console_register_cli();
    }

    return ESP_OK;
}

esp_err_t usb_cli_console_register_settings(void)
{
    static const settings_descriptor_t DESC =
    {
        .name        = "usb_cli_console",
        .version     = 1,
        .fields      = FIELDS,
        .field_count = sizeof(FIELDS) / sizeof(FIELDS[0]),
        .on_apply    = on_apply,
    };

    return settings_manager_register(&DESC);
}
