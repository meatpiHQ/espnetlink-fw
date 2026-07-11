/**
 * @file imu_settings.c
 * @brief Minimal {cli} settings descriptor for the imu component (§6b: a
 *        component with no other settings still gets the knob). The boot
 *        apply registers the `imu` console command.
 */
#include "cJSON.h"
#include "settings_manager.h"

#include "imu.h"

static const settings_field_t FIELDS[] =
{
    SETTINGS_BOOL("cli", true),
};

static esp_err_t on_apply(const cJSON *settings)
{
    const cJSON *cli = cJSON_GetObjectItemCaseSensitive(settings, "cli");

    if (!cJSON_IsFalse(cli))
    {
        imu_console_register();
    }

    return ESP_OK;
}

esp_err_t imu_register_settings(void)
{
    static const settings_descriptor_t DESC =
    {
        .name        = "imu",
        .version     = 1,
        .fields      = FIELDS,
        .field_count = sizeof(FIELDS) / sizeof(FIELDS[0]),
        .on_apply    = on_apply,
    };

    return settings_manager_register(&DESC);
}
