/**
 * @file restart_tracker_settings.c
 * @brief settings_manager descriptor for restart_tracker (CLI ownership).
 *
 * Minimal descriptor: the one knob is whether the component registers its
 * console command (`restart_tracker`) at boot. Init runs before
 * settings_manager_init, so main wires this separately (the
 * log_manager_register_settings pattern). Reboot-to-apply.
 */
#include "settings_manager.h"

#include "restart_tracker.h"

static const settings_field_t RT_SETTINGS_FIELDS[] =
{
    SETTINGS_BOOL("cli", true),
};

static esp_err_t rt_settings_apply(const cJSON *settings)
{
    if (!cJSON_IsFalse(cJSON_GetObjectItemCaseSensitive(settings, "cli")))
    {
        static bool s_cli_registered;

        if (!s_cli_registered && restart_tracker_register_cli() == ESP_OK)
        {
            s_cli_registered = true;
        }
    }

    return ESP_OK;
}

esp_err_t restart_tracker_register_settings(void)
{
    static const settings_descriptor_t DESC =
    {
        .name        = "restart_tracker",
        .version     = 1,
        .fields      = RT_SETTINGS_FIELDS,
        .field_count = sizeof(RT_SETTINGS_FIELDS) /
                       sizeof(RT_SETTINGS_FIELDS[0]),
        .on_apply    = rt_settings_apply,
    };

    return settings_manager_register(&DESC);
}
