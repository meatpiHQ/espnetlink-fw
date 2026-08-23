/**
 * @file wifi_modem_provision.c
 * @brief One-time per-device AP password provisioning. Runs once per device
 *        lifetime (marker: the `ap_password_provisioned` setting): a stock
 *        "12345678" in wifi_manager's settings is replaced by a random
 *        password; a custom one is kept. Persist-only — main reboots once
 *        when something changed (§4.2 reboot-to-apply). Host-tested through
 *        wifi_modem_provision_with() with a scripted RNG.
 *
 * §10: the generated password is never logged. Read it with `wifi_modem -k`.
 */
#include "wifi_modem_private.h"

#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#include "cJSON.h"
#include "settings_manager.h"

static const char *TAG = "wifi_modem";

static esp_err_t persist_generated(uint32_t (*rng)(void), bool *changed)
{
    cJSON *wm = NULL;
    esp_err_t err = settings_manager_get("wifi_manager", &wm);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "provision: read wifi_manager settings: %s",
                 esp_err_to_name(err));
        return err;
    }

    const cJSON *pw = cJSON_GetObjectItemCaseSensitive(wm, "ap_password");
    const char *current = cJSON_IsString(pw) ? pw->valuestring : "";

    if (wmc_provision_decide(false, current) != WMC_PROVISION_GENERATE)
    {
        cJSON_Delete(wm); /* custom password: keep it */
        return ESP_OK;
    }

    char generated[WIFI_MODEM_AP_PASSWORD_LEN + 1];

    if (!wmc_generate_password(generated, sizeof(generated),
                               WIFI_MODEM_AP_PASSWORD_LEN, rng))
    {
        cJSON_Delete(wm);
        return ESP_ERR_INVALID_ARG;
    }

    cJSON_DeleteItemFromObjectCaseSensitive(wm, "ap_password");
    cJSON_AddStringToObject(wm, "ap_password", generated);

    char errbuf[96] = "";

    err = settings_manager_set("wifi_manager", wm, errbuf, sizeof(errbuf),
                               changed);
    memset(generated, 0, sizeof(generated));
    cJSON_Delete(wm);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "provision: persist AP password: %s %s",
                 esp_err_to_name(err), errbuf);
        return err;
    }

    ESP_LOGW(TAG, "per-device AP password provisioned (%d chars) — read it "
                  "with `wifi_modem -k`", WIFI_MODEM_AP_PASSWORD_LEN);
    return ESP_OK;
}

esp_err_t wifi_modem_provision_with(uint32_t (*rng)(void),
                                    bool *reboot_required)
{
    if (reboot_required == NULL || rng == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    *reboot_required = false;

    if (!wifi_modem_settings_is_configured())
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (wifi_modem_settings_config()->ap_password_provisioned)
    {
        return ESP_OK;
    }

    bool changed = false;
    esp_err_t err = persist_generated(rng, &changed);

    if (err != ESP_OK)
    {
        return err; /* try again next boot; marker stays false */
    }

    /* marker first-class: a later deliberate "12345678" is never overridden */
    err = wifi_modem_settings_persist_bool("ap_password_provisioned", true);
    if (err != ESP_OK)
    {
        return err;
    }

    *reboot_required = changed;
    return ESP_OK;
}
