/**
 * @file wifi_modem_settings.c
 * @brief settings_manager descriptor for wifi_modem (standard §4.1): the
 *        field table (single source of truth for shape/defaults), on_apply
 *        (stores the boot-applied config, registers the CLI behind the
 *        `cli` gate — §6b), the const getters, and the persist helpers the
 *        CLI/provisioning use (reboot-to-apply, never a live reconfigure).
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
    /* auto (shipping default) = boot with USB data on (today's NCM + CDC
       behaviour for a PC), and let the host cut it at runtime — the WiCAN
       reads the AP credentials over USB, then asks for the data lines to
       be cut so the cable carries 5 V only and GPS is not desensed.
       ncm = USB data locked on (runtime cut refused).
       power_only = cut at boot (powered from a charger, AP only). */
    SETTINGS_STR_ENUM("usb_mode", "auto,ncm,power_only", "auto"),
    /* NAPT the WiFi AP netif into the LTE uplink + DNS proxy for clients */
    SETTINGS_BOOL("ap_share", true),
    /* set once by wifi_modem_provision(): the stock AP password has been
       replaced (or a custom one was found). Never cleared by the firmware;
       a factory reset starts the provisioning over. */
    SETTINGS_BOOL("ap_password_provisioned", false),
    /* device state, not user config (the ap_password_provisioned pattern):
       the last runtime usb_data change was a CUT by a host. usb_mode=auto
       then cuts the lines AT BOOT (the paired-WiCAN steady state: GPS
       acquires from power-on with a quiet bus, and the host never needs
       the USB link again) and self-restores + clears this after 60 s
       when no station joins the AP — a PC on the cable gets its
       tethering back, one boot late. Cleared by any runtime restore and
       by factory reset. */
    SETTINGS_BOOL("boot_cut", false),
    SETTINGS_BOOL("cli", true),
};

/* boot-applied configuration — written by on_apply only */
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

bool wifi_modem_usb_data_enabled(void)
{
    /* unconfigured (settings degraded) => legacy behaviour: USB data on */
    return !s_configured || s_cfg.usb_mode != WIFI_MODEM_USB_POWER_ONLY;
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
    s_cfg.boot_cut = get_bool(settings, "boot_cut", false);
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

esp_err_t wifi_modem_settings_register(void)
{
    static const settings_descriptor_t DESC =
    {
        .name        = "wifi_modem",
        .version     = 1,
        .fields      = FIELDS,
        .field_count = sizeof(FIELDS) / sizeof(FIELDS[0]),
        .on_apply    = on_apply,
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
