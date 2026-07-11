/**
 * @file ha_webhooks_settings.c
 * @brief settings_manager descriptor for ha_webhooks: field-table schema
 *        (source of truth for shape/ranges/defaults) and on_apply. The
 *        applied config is handed to ha_webhooks.c via hw_config_store —
 *        the cache lives THERE because the /api/webhook URL push mutates
 *        it at runtime (hw_config_apply_live), not only at boot.
 */
#include <string.h>

#include "settings_manager.h"

#include "ha_webhooks.h" /* ha_webhooks_register_cli (settings-gated) */
#include "ha_webhooks_private.h"

/* Field table -> the manager generates the JSON Schema (Standard §5). */
/* clang-format off */
static const settings_field_t HW_FIELDS[] =
{
    SETTINGS_BOOL("enabled",           false),
    SETTINGS_STR ("url",               HW_URL_LEN - 1, ""),
    SETTINGS_STR ("url2",              HW_URL_LEN - 1, ""),
    SETTINGS_INT ("interval_s",        1, 3600, 60), /* contract: 1..3600 */
    SETTINGS_STR_ENUM("data_mode",     "changed,full", "changed"),
    SETTINGS_BOOL("gzip",              false), /* v2: gzip the push body —
        works in BOTH data modes; off by default (HA integrations older
        than 2026-07-10 can't inflate)                                    */
    SETTINGS_BOOL("manual_override",   false),
    SETTINGS_STR ("cert_set",          HW_CERTSET_LEN - 1, ""),
    SETTINGS_BOOL("cli",               true),
};
/* clang-format on */

static void copy_str(char *dst, size_t cap, const cJSON *obj, const char *key)
{
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(obj, key);

    dst[0] = '\0';

    if (cJSON_IsString(v) && v->valuestring != NULL)
    {
        strlcpy(dst, v->valuestring, cap);
    }
}

static void config_from_json(const cJSON *s, hw_config_t *c)
{
    memset(c, 0, sizeof(*c));

    c->enabled = cJSON_IsTrue(
        cJSON_GetObjectItemCaseSensitive(s, "enabled"));
    copy_str(c->url, sizeof(c->url), s, "url");
    copy_str(c->url2, sizeof(c->url2), s, "url2");

    const cJSON *iv = cJSON_GetObjectItemCaseSensitive(s, "interval_s");

    c->interval_s = cJSON_IsNumber(iv) ? (uint32_t)iv->valueint : 60;

    char mode[8];

    copy_str(mode, sizeof(mode), s, "data_mode");
    c->data_mode_full = (strcmp(mode, "full") == 0);

    c->gzip = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(s, "gzip"));
    c->manual_override = cJSON_IsTrue(
        cJSON_GetObjectItemCaseSensitive(s, "manual_override"));
    copy_str(c->cert_set, sizeof(c->cert_set), s, "cert_set");
}

/** v1 -> v2: the `gzip` field was added (default off). */
static esp_err_t on_migrate(uint32_t from_version, cJSON *settings)
{
    (void)from_version;

    if (cJSON_GetObjectItemCaseSensitive(settings, "gzip") == NULL)
    {
        cJSON_AddBoolToObject(settings, "gzip", false);
    }

    return ESP_OK;
}

static esp_err_t on_apply(const cJSON *settings)
{
    hw_config_t c;

    config_from_json(settings, &c);
    hw_config_store(&c);

    if (cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(settings, "cli")))
    {
        static bool s_cli_done;

        if (!s_cli_done && ha_webhooks_register_cli() == ESP_OK)
        {
            s_cli_done = true;
        }
    }

    return ESP_OK;
}

esp_err_t hw_settings_register(void)
{
    static const settings_descriptor_t DESC =
    {
        .name        = "ha_webhooks",
        .version     = 2, /* v2: + gzip */
        .fields      = HW_FIELDS,
        .field_count = sizeof(HW_FIELDS) / sizeof(HW_FIELDS[0]),
        .on_apply    = on_apply,
        .on_migrate  = on_migrate,
    };

    return settings_manager_register(&DESC);
}
