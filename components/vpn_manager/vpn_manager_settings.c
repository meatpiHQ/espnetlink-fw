/**
 * @file vpn_manager_settings.c
 * @brief settings_manager descriptor for vpn_manager: field-table schema
 *        (source of truth for shape/ranges/defaults), on_validate (the
 *        pure vpn_check_config rules), on_apply (parses into the
 *        boot-applied config + the derived display endpoint).
 */
#include <stdio.h>
#include <string.h>

#include "settings_manager.h"

#include "vpn_manager.h" /* vpn_manager_register_cli (settings-gated) */
#include "vpn_manager_private.h"

static const settings_field_t FIELDS[] =
{
    SETTINGS_BOOL("enabled", false),
    SETTINGS_STR_ENUM("type", "wireguard,tailscale", "wireguard"),
    SETTINGS_STR("ts_auth_key", 95, ""),
    SETTINGS_STR("ts_device_name", 47, ""),
    SETTINGS_STR("ts_control_url", 63, ""),
    SETTINGS_STR("private_key", 63, ""),
    SETTINGS_STR("peer_public_key", 63, ""),
    SETTINGS_STR("preshared_key", 63, ""),
    SETTINGS_STR("address", 31, ""),
    SETTINGS_STR("allowed_ip", 31, "0.0.0.0"),
    SETTINGS_STR("allowed_ip_mask", 31, "0.0.0.0"),
    SETTINGS_STR("endpoint", 63, ""),
    SETTINGS_INT("port", 1, 65535, 51820),
    SETTINGS_INT("keepalive_s", 0, 3600, 25),
    SETTINGS_BOOL("default_route", false),
    SETTINGS_STR("dns", 15, ""),
    SETTINGS_BOOL("cli", true),
};

static vpn_config_t s_cfg;
static bool s_configured;
static char s_endpoint[64]; /* display endpoint (status/log strings) */

const vpn_config_t *vpn_settings_config(void)
{
    return &s_cfg;
}

bool vpn_settings_is_configured(void)
{
    return s_configured;
}

const char *vpn_settings_endpoint(void)
{
    return s_endpoint;
}

static void field_str(const cJSON *settings, const char *key, char *out,
                      size_t len)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(settings, key);

    strlcpy(out, cJSON_IsString(item) ? item->valuestring : "", len);
}

static void config_from_json(const cJSON *settings, vpn_config_t *cfg)
{
    const cJSON *item;

    memset(cfg, 0, sizeof(*cfg));
    cfg->enabled = cJSON_IsTrue(
        cJSON_GetObjectItemCaseSensitive(settings, "enabled"));
    item = cJSON_GetObjectItemCaseSensitive(settings, "type");
    cfg->tailscale = cJSON_IsString(item) &&
                     strcmp(item->valuestring, "tailscale") == 0;
    field_str(settings, "ts_auth_key", cfg->ts_auth_key,
              sizeof(cfg->ts_auth_key));
    field_str(settings, "ts_device_name", cfg->ts_device_name,
              sizeof(cfg->ts_device_name));
    field_str(settings, "ts_control_url", cfg->ts_control_url,
              sizeof(cfg->ts_control_url));
    field_str(settings, "private_key", cfg->private_key,
              sizeof(cfg->private_key));
    field_str(settings, "peer_public_key", cfg->peer_public_key,
              sizeof(cfg->peer_public_key));
    field_str(settings, "preshared_key", cfg->preshared_key,
              sizeof(cfg->preshared_key));
    field_str(settings, "address", cfg->address, sizeof(cfg->address));
    field_str(settings, "allowed_ip", cfg->allowed_ip,
              sizeof(cfg->allowed_ip));
    field_str(settings, "allowed_ip_mask", cfg->allowed_ip_mask,
              sizeof(cfg->allowed_ip_mask));
    field_str(settings, "endpoint", cfg->endpoint,
              sizeof(cfg->endpoint));
    item = cJSON_GetObjectItemCaseSensitive(settings, "port");
    cfg->port = cJSON_IsNumber(item) ? item->valueint : 51820;
    item = cJSON_GetObjectItemCaseSensitive(settings, "keepalive_s");
    cfg->keepalive_s = cJSON_IsNumber(item) ? item->valueint : 25;
    cfg->default_route = cJSON_IsTrue(
        cJSON_GetObjectItemCaseSensitive(settings, "default_route"));
    field_str(settings, "dns", cfg->dns, sizeof(cfg->dns));
}

static esp_err_t on_validate(const cJSON *settings, char *err,
                             size_t err_len)
{
    vpn_config_t cfg;

    config_from_json(settings, &cfg);

    const char *reason = vpn_check_config(&cfg);

    if (reason != NULL)
    {
        strlcpy(err, reason, err_len);
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static esp_err_t on_apply(const cJSON *settings)
{
    config_from_json(settings, &s_cfg);

    if (s_cfg.tailscale)
    {
        snprintf(s_endpoint, sizeof(s_endpoint), "%.62s",
                 s_cfg.ts_control_url[0] ? s_cfg.ts_control_url
                                         : "tailscale.com");
    }
    else
    {
        snprintf(s_endpoint, sizeof(s_endpoint),
                 "%.56s:%d", s_cfg.endpoint, s_cfg.port);
    }

    if (!cJSON_IsFalse(cJSON_GetObjectItemCaseSensitive(settings, "cli")))
    {
        static bool s_cli_registered;

        if (!s_cli_registered && vpn_manager_register_cli() == ESP_OK)
        {
            s_cli_registered = true;
        }
    }

    s_configured = true;
    return ESP_OK;
}

esp_err_t vpn_settings_register(void)
{
    static const settings_descriptor_t DESC =
    {
        .name = "vpn_manager",
        .version = 1,
        .fields = FIELDS,
        .field_count = sizeof(FIELDS) / sizeof(FIELDS[0]),
        .on_apply = on_apply,
        .on_validate = on_validate,
    };

    return settings_manager_register(&DESC);
}
