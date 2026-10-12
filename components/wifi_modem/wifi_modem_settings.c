/**
 * @file wifi_modem_settings.c
 * @brief settings_manager descriptor for wifi_modem (standard §4.1), v2: the
 *        field table (single source of truth for shape/defaults), on_apply
 *        (stores the boot-applied config, registers the CLI behind the
 *        `cli` gate, §6b), on_migrate (delegates to the pure
 *        wifi_modem_migrate.c), the const getters, and the persist helpers
 *        the CLI/provisioning use (reboot-to-apply, never a live
 *        reconfigure).
 */
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#include "cJSON.h"
#include "settings_manager.h"

#include "wifi_modem.h"
#include "wifi_modem_private.h"

static const char *TAG = "wifi_modem";

static const settings_field_t FIELDS[] =
{
    /* Every mode boots with the USB data lines on the DIP route, so a
       host can always reach the device after a power cycle (2026-10-12:
       a forgotten AP password must never lock the owner out).
       auto (shipping default) = USB Ethernet (NCM or RNDIS per
       usb_dev_ethernet.class) + the CDC console; the host may cut the
       data lines at runtime (the WiCAN reads the AP credentials over USB,
       then asks for the cut so the cable carries 5 V only and GPS is not
       desensed) and the cut lasts until the next boot.
       ncm = the same composite, the runtime cut refused (locked on).
       console = the WiFi AP is the product (charger-powered); USB presents
       the CDC console ONLY, no Ethernet function: `wifi_modem -k` and
       `wifi_modem -u auto` are always one cable away. */
    SETTINGS_STR_ENUM("usb_mode", "auto,ncm,console", "auto"),
    /* NAPT the WiFi AP netif into the LTE uplink + DNS proxy for clients */
    SETTINGS_BOOL("ap_share", true),
    /* set once by wifi_modem_provision(): the stock AP password has been
       replaced (or a custom one was found). Never cleared by the firmware;
       a factory reset starts the provisioning over. */
    SETTINGS_BOOL("ap_password_provisioned", false),
    SETTINGS_BOOL("cli", true),
};

/* boot-applied configuration, written by on_apply only */
static wifi_modem_settings_t s_cfg;
static bool s_configured;

const wifi_modem_settings_t *wifi_modem_settings_config(void)
{
    return &s_cfg;
}

bool wifi_modem_settings_is_configured(void)
{
    return s_configured;
}

#ifdef WIFI_MODEM_HOST_TEST
/* Test-only (§3 exception): the Unity harness re-inits between cases; on
 * the device a reboot clears this state. Not in the public header. */
void wifi_modem_settings_test_reset(void)
{
    memset(&s_cfg, 0, sizeof(s_cfg));
    s_configured = false;
}
#endif

bool wifi_modem_usb_ethernet_enabled(void)
{
    /* unconfigured (settings degraded) => legacy behaviour: USB Ethernet */
    return !s_configured || wmc_usb_ethernet_present(s_cfg.usb_mode);
}

static bool get_bool(const cJSON *obj, const char *key, bool fallback)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);

    return cJSON_IsBool(item) ? cJSON_IsTrue(item) : fallback;
}

static esp_err_t on_apply(const cJSON *settings)
{
    const cJSON *mode = cJSON_GetObjectItemCaseSensitive(settings, "usb_mode");
    const char *m = cJSON_IsString(mode) ? mode->valuestring : "auto";

    if (!wmc_usb_mode_parse(m, &s_cfg.usb_mode))
    {
        s_cfg.usb_mode = WIFI_MODEM_USB_AUTO;
    }
    s_cfg.ap_share = get_bool(settings, "ap_share", true);
    s_cfg.ap_password_provisioned =
        get_bool(settings, "ap_password_provisioned", false);
    s_cfg.cli = get_bool(settings, "cli", true);
    s_configured = true;

    ESP_LOGI(TAG, "usb_mode=%s ap_share=%d", wmc_usb_mode_str(s_cfg.usb_mode),
             (int)s_cfg.ap_share);

    /* §6b: the component registers its own command, gated by `cli`;
       registration failure is log-and-continue inside register_cli */
    if (s_cfg.cli)
    {
        static bool s_cli_registered;

        if (!s_cli_registered && wifi_modem_register_cli() == ESP_OK)
        {
            s_cli_registered = true;
        }
    }

    return ESP_OK;
}

static esp_err_t on_migrate(uint32_t from_version, cJSON *settings)
{
    return wifi_modem_settings_migrate(from_version, settings);
}

esp_err_t wifi_modem_settings_register(void)
{
    static const settings_descriptor_t DESC =
    {
        .name        = "wifi_modem",
        .version     = 2, /* v1 power_only + boot_cut; v2 console, no hint */
        .fields      = FIELDS,
        .field_count = sizeof(FIELDS) / sizeof(FIELDS[0]),
        .on_apply    = on_apply,
        .on_migrate  = on_migrate,
    };

    return settings_manager_register(&DESC);
}

/* GET -> replace one key -> SET (full-object replace, §6). */
static esp_err_t persist_item(const char *field, cJSON *value)
{
    cJSON *obj = NULL;
    esp_err_t err = settings_manager_get("wifi_modem", &obj);

    if (err != ESP_OK)
    {
        cJSON_Delete(value);
        return err;
    }

    cJSON_DeleteItemFromObjectCaseSensitive(obj, field);
    cJSON_AddItemToObject(obj, field, value);

    char errbuf[96] = "";

    err = settings_manager_set("wifi_modem", obj, errbuf, sizeof(errbuf), NULL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "persist %s: %s %s", field, esp_err_to_name(err), errbuf);
    }
    cJSON_Delete(obj);
    return err;
}

esp_err_t wifi_modem_settings_persist_str(const char *field, const char *value)
{
    if (field == NULL || value == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *item = cJSON_CreateString(value);

    return (item != NULL) ? persist_item(field, item) : ESP_ERR_NO_MEM;
}

esp_err_t wifi_modem_settings_persist_bool(const char *field, bool value)
{
    if (field == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *item = cJSON_CreateBool(value);

    return (item != NULL) ? persist_item(field, item) : ESP_ERR_NO_MEM;
}
