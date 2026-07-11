/**
 * @file cmdline_manager_settings.c
 * @brief settings_manager descriptor for cmdline_manager: field-table
 *        schema (source of truth for shape/defaults) and on_apply (stores
 *        the boot-applied enabled/uart flags read by _start()).
 */
#include "settings_manager.h"

#include "cmdline_manager_private.h"

static const settings_field_t FIELDS[] =
{
    SETTINGS_BOOL("enabled", true),
    /* ESPNetLink port: default OFF (WiCAN ships true). UART0 here carries
     * boot logs and the optional GPS_UART0_OUT NMEA stream; the product
     * console is the USB CDC one. */
    SETTINGS_BOOL("uart", false),
};

/* boot-applied settings */
static bool s_enabled;
static bool s_uart;
static bool s_configured;

bool cm_settings_enabled(void)
{
    return s_enabled;
}

bool cm_settings_uart(void)
{
    return s_uart;
}

bool cm_settings_is_configured(void)
{
    return s_configured;
}

static esp_err_t on_apply(const cJSON *settings)
{
    const cJSON *item =
        cJSON_GetObjectItemCaseSensitive(settings, "enabled");

    s_enabled = cJSON_IsTrue(item);
    item = cJSON_GetObjectItemCaseSensitive(settings, "uart");
    s_uart = cJSON_IsTrue(item);
    s_configured = true;
    return ESP_OK;
}

esp_err_t cm_settings_register(void)
{
    static const settings_descriptor_t DESC =
    {
        .name = "cmdline_manager",
        .version = 1,
        .fields = FIELDS,
        .field_count = sizeof(FIELDS) / sizeof(FIELDS[0]),
        .on_apply = on_apply,
    };

    return settings_manager_register(&DESC);
}
