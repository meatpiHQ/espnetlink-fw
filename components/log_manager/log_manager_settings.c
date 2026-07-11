/**
 * @file log_manager_settings.c
 * @brief Settings descriptor ("log_manager", §9.4): persisted boot defaults
 *        for the global level and the built-in sink enables. Runtime changes
 *        (log_manager_set_level, sink toggles) stay ephemeral and reset to
 *        these on reboot.
 */
#include <string.h>

#include "esp_log.h"

#include "settings_manager.h"

#include "log_manager.h"

/* component-private bridge into log_manager.c */
void lm_apply_sink_config(bool console_enabled, bool ring_enabled);

/* clang-format off */
static const settings_field_t LM_FIELDS[] =
{
    SETTINGS_STR_ENUM("level", "none,error,warn,info,debug,verbose", "info"),
    SETTINGS_BOOL("console_enabled", true),
    SETTINGS_BOOL("ring_enabled",    true),
};
/* clang-format on */

static esp_log_level_t parse_level(const char *s)
{
    if (strcmp(s, "none") == 0)
    {
        return ESP_LOG_NONE;
    }

    if (strcmp(s, "error") == 0)
    {
        return ESP_LOG_ERROR;
    }

    if (strcmp(s, "warn") == 0)
    {
        return ESP_LOG_WARN;
    }

    if (strcmp(s, "debug") == 0)
    {
        return ESP_LOG_DEBUG;
    }

    if (strcmp(s, "verbose") == 0)
    {
        return ESP_LOG_VERBOSE;
    }

    return ESP_LOG_INFO;
}

static esp_err_t lm_on_apply(const cJSON *settings)
{
    const cJSON *level = cJSON_GetObjectItemCaseSensitive(settings, "level");
    const cJSON *console = cJSON_GetObjectItemCaseSensitive(settings,
                                                            "console_enabled");
    const cJSON *ring = cJSON_GetObjectItemCaseSensitive(settings,
                                                         "ring_enabled");

    if (cJSON_IsString(level) && level->valuestring != NULL)
    {
        esp_log_level_set("*", parse_level(level->valuestring));
    }

    lm_apply_sink_config(!cJSON_IsFalse(console), !cJSON_IsFalse(ring));
    return ESP_OK;
}

esp_err_t log_manager_register_settings(void)
{
    static const settings_descriptor_t desc =
    {
        .name        = "log_manager",
        .version     = 1,
        .fields      = LM_FIELDS,
        .field_count = sizeof(LM_FIELDS) / sizeof(LM_FIELDS[0]),
        .on_apply    = lm_on_apply,
    };

    return settings_manager_register(&desc);
}
