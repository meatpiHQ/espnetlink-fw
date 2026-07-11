/**
 * @file imu_manager_settings.c
 * @brief settings_manager descriptor for imu_manager: field-table schema
 *        (source of truth for shape/ranges/defaults), on_apply (parses into
 *        the boot-applied config read through the imu_settings_* getters),
 *        on_migrate (delegates to the pure imu_settings_migrate —
 *        imu_manager_migrate.c, host-tested).
 */
#include "settings_manager.h"

#include "imu_manager.h" /* imu_manager_register_cli (settings-gated) */

#include "imu_manager_private.h"

static const settings_field_t FIELDS[] =
{
    SETTINGS_BOOL("enabled", true),
    SETTINGS_BOOL("smd", true),               /* sustained -> activity    */
    SETTINGS_BOOL("wom", true),               /* bumps -> events          */
    SETTINGS_INT("smd_sensitivity", 0, 4, 0), /* 0 = most sensitive       */
    SETTINGS_INT("wom_threshold", 1, 255, 8), /* 1 LSB ~= 3.9 mg          */
    SETTINGS_INT("stationary_s", 1, 3600, 3),
    SETTINGS_BOOL("cli", true),
};

static imu_settings_t s_config; /* boot-applied settings */
static bool s_configured;

const imu_settings_t *imu_settings_config(void)
{
    return &s_config;
}

bool imu_settings_is_configured(void)
{
    return s_configured;
}

static esp_err_t on_apply(const cJSON *settings)
{
    imu_settings_t *c = &s_config;
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(settings,
                                                         "enabled");

    c->enabled = cJSON_IsTrue(item);
    item = cJSON_GetObjectItemCaseSensitive(settings, "smd");
    c->smd = cJSON_IsTrue(item);
    item = cJSON_GetObjectItemCaseSensitive(settings, "wom");
    c->wom = cJSON_IsTrue(item);
    item = cJSON_GetObjectItemCaseSensitive(settings, "smd_sensitivity");
    c->smd_sensitivity =
        (uint8_t)(cJSON_IsNumber(item) ? item->valueint : 0);
    item = cJSON_GetObjectItemCaseSensitive(settings, "wom_threshold");
    c->wom_threshold = (uint8_t)(cJSON_IsNumber(item) ? item->valueint : 8);
    item = cJSON_GetObjectItemCaseSensitive(settings, "stationary_s");
    c->stationary_ms =
        1000u * (uint32_t)(cJSON_IsNumber(item) ? item->valueint : 3);

    /* CLI ownership: settings-gated self-registration (reboot-to-apply) */
    if (!cJSON_IsFalse(cJSON_GetObjectItemCaseSensitive(settings, "cli")))
    {
        static bool s_cli_registered;

        if (!s_cli_registered && imu_manager_register_cli() == ESP_OK)
        {
            s_cli_registered = true;
        }
    }

    s_configured = true;
    return ESP_OK;
}

static esp_err_t on_migrate(uint32_t from_version, cJSON *settings)
{
    return imu_settings_migrate(from_version, settings);
}

esp_err_t imu_settings_register(void)
{
    static const settings_descriptor_t DESC =
    {
        .name = "imu_manager",
        .version = 3, /* v1 WoM-only; v2 SMD-only; v3 both + enables */
        .fields = FIELDS,
        .field_count = sizeof(FIELDS) / sizeof(FIELDS[0]),
        .on_apply = on_apply,
        .on_migrate = on_migrate,
    };

    return settings_manager_register(&DESC);
}
