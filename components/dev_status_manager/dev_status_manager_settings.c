/**
 * @file dev_status_manager_settings.c
 * @brief Settings descriptor ("dev_status_manager", CLI ownership).
 *        Minimal descriptor: the one knob is whether the component
 *        registers its console commands (`version`, `status`) at boot.
 *        Init runs before settings_manager_init, so main wires
 *        dev_status_manager_register_settings() separately (the
 *        log_manager_register_settings pattern). Reboot-to-apply.
 */
#include "settings_manager.h"

#include "dev_status_manager.h"

static const settings_field_t DSM_SETTINGS_FIELDS[] =
{
    SETTINGS_BOOL("cli", true),
};

static esp_err_t dsm_settings_apply(const cJSON *settings)
{
    if (!cJSON_IsFalse(cJSON_GetObjectItemCaseSensitive(settings, "cli")))
    {
        static bool s_cli_registered;

        if (!s_cli_registered &&
            dev_status_manager_register_cli() == ESP_OK)
        {
            s_cli_registered = true;
        }
    }

    return ESP_OK;
}

esp_err_t dev_status_manager_register_settings(void)
{
    static const settings_descriptor_t DESC =
    {
        .name        = "dev_status_manager",
        .version     = 1,
        .fields      = DSM_SETTINGS_FIELDS,
        .field_count = sizeof(DSM_SETTINGS_FIELDS) /
                       sizeof(DSM_SETTINGS_FIELDS[0]),
        .on_apply    = dsm_settings_apply,
    };

    return settings_manager_register(&DESC);
}
