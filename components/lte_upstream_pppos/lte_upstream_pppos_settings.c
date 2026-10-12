/**
 * @file lte_upstream_pppos_settings.c
 * @brief settings_manager descriptor for lte_upstream_pppos (standard §4.1):
 *        field-table schema and on_apply storing the boot-applied config.
 *
 * Field names mirror the legacy config_manager keys (APN, LTE_ENABLED, ...);
 * main's one-time legacy import maps the old /storage/config.json values
 * onto this descriptor. `ncm_share` defaults to TRUE again since 2026-10-12:
 * the dongle starts as a USB LTE modem in every mode (the WiCAN pairs in
 * usb_rndis mode by default and needs the uplink shared over the wire), so
 * a matching default spares the dongle restart that the WiCAN's ensure
 * pass would otherwise cost at the first contact. A PC on the cable gets
 * its default route over LTE with it; turn it off per device (the dongle's
 * web UI) to keep the USB link a control channel. (2026-08-23 to
 * 2026-10-12 the default was false, for that PC case.)
 */
#include <stdio.h>

#include "esp_err.h"

#include "cJSON.h"
#include "settings_manager.h"

#include "lte_upstream_pppos.h"

static const settings_field_t FIELDS[] =
{
    SETTINGS_BOOL("enabled",   true),            /* legacy LTE_ENABLED */
    SETTINGS_STR ("apn",          63, "internet"), /* legacy APN       */
    SETTINGS_STR ("apn_user",     32, ""),       /* legacy APN_USER    */
    SETTINGS_STR ("apn_password", 32, ""),       /* legacy APN_PASS    */
    SETTINGS_STR ("apn_pin",       8, ""),       /* legacy APN_PIN     */
    SETTINGS_BOOL("ncm_share", true),            /* legacy NCM_SHARE   */
    SETTINGS_BOOL("cli",       true),
};

static lte_upstream_pppos_settings_t s_cfg;
static bool s_configured;

const lte_upstream_pppos_settings_t *lte_upstream_pppos_settings(void)
{
    return &s_cfg;
}

bool lte_upstream_pppos_settings_is_configured(void)
{
    return s_configured;
}

static void copy_str(char *dst, size_t dst_len, const cJSON *obj,
                     const char *key)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);

    dst[0] = '\0';
    if (cJSON_IsString(item) && item->valuestring != NULL)
    {
        /* snprintf, not strlcpy: this file also compiles in the host test
         * and glibc < 2.38 (the IDF 5.5 CI container) lacks strlcpy. */
        snprintf(dst, dst_len, "%s", item->valuestring);
    }
}

static bool get_bool(const cJSON *obj, const char *key, bool fallback)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);

    return cJSON_IsBool(item) ? cJSON_IsTrue(item) : fallback;
}

static esp_err_t on_apply(const cJSON *settings)
{
    s_cfg.enabled   = get_bool(settings, "enabled", true);
    s_cfg.ncm_share = get_bool(settings, "ncm_share", true);
    s_cfg.cli       = get_bool(settings, "cli", true);
    copy_str(s_cfg.apn, sizeof(s_cfg.apn), settings, "apn");
    copy_str(s_cfg.apn_user, sizeof(s_cfg.apn_user), settings, "apn_user");
    copy_str(s_cfg.apn_password, sizeof(s_cfg.apn_password), settings,
             "apn_password");
    copy_str(s_cfg.apn_pin, sizeof(s_cfg.apn_pin), settings, "apn_pin");
    s_configured = true;

    /* §6b: register the `lte` console command on the boot apply, gated
     * by `cli`. */
    if (s_cfg.cli)
    {
        lte_upstream_pppos_console_register();
    }

    return ESP_OK;
}

esp_err_t lte_upstream_pppos_register_settings(void)
{
    static const settings_descriptor_t DESC =
    {
        .name        = "lte_upstream_pppos",
        .version     = 1,
        .fields      = FIELDS,
        .field_count = sizeof(FIELDS) / sizeof(FIELDS[0]),
        .on_apply    = on_apply,
    };

    return settings_manager_register(&DESC);
}
