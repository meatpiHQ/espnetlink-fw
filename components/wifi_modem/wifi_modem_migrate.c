/**
 * @file wifi_modem_migrate.c
 * @brief The pure "wifi_modem" settings migration (§4/§5, host-tested):
 *        v1 to v2 (2026-10-12). Only cJSON, no IDF services.
 *
 * v1 (2026-08-23): usb_mode auto | ncm | power_only, ap_share,
 *     ap_password_provisioned, boot_cut (device state: a host's cut armed
 *     a boot-time cut), cli.
 * v2 (2026-10-12): usb_mode auto | ncm | console; boot_cut gone. The data
 *     lines are on at every boot in every mode, so a forgotten AP password
 *     is never a lock-out: power_only (cut at boot, no way in) maps to
 *     console (the CDC console on USB, WiFi AP as the product).
 */
#include "wifi_modem_private.h"

#include <string.h>

esp_err_t wifi_modem_settings_migrate(uint32_t from_version, cJSON *settings)
{
    if (!cJSON_IsObject(settings))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (from_version >= 2)
    {
        return ESP_OK; /* nothing older than v2 to lift */
    }

    const cJSON *mode = cJSON_GetObjectItemCaseSensitive(settings, "usb_mode");

    if (cJSON_IsString(mode) && mode->valuestring != NULL &&
        strcmp(mode->valuestring, "power_only") == 0)
    {
        cJSON_DeleteItemFromObjectCaseSensitive(settings, "usb_mode");
        cJSON_AddStringToObject(settings, "usb_mode", "console");
    }

    /* device state of the retired boot-time cut: nothing reads it now */
    cJSON_DeleteItemFromObjectCaseSensitive(settings, "boot_cut");

    return ESP_OK;
}
