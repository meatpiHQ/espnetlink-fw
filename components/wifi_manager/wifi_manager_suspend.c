/**
 * @file wifi_manager_suspend.c
 * @brief Runtime per-interface suspension — the actuation surface for
 *        interface_manager's wireless arbitration (meatpi 2026-07-05:
 *        BLE connect suspends STA / AP-vs-BLE exclusivity). EPHEMERAL:
 *        settings stay untouched, a reboot (or resume) restores the
 *        configured mode. The reconnect task and the AP-restore paths
 *        in wifi_manager.c consult the suspend flags.
 */
#include "wifi_manager.h"

#include "esp_log.h"
#include "esp_wifi.h"

#include "wifi_manager_private.h"

static const char *TAG = "wifi_manager";

static volatile bool s_sta_susp;
static volatile bool s_ap_susp;

bool wm_suspend_sta_active(void)
{
    return s_sta_susp;
}

bool wm_suspend_ap_active(void)
{
    return s_ap_susp;
}

bool wifi_manager_mode_has_sta(void)
{
    const wm_config_t *cfg = wm_settings_config();

    return cfg->mode == WM_MODE_STA || cfg->mode == WM_MODE_APSTA;
}

bool wifi_manager_mode_has_ap(void)
{
    const wm_config_t *cfg = wm_settings_config();

    return cfg->mode == WM_MODE_AP || cfg->mode == WM_MODE_APSTA;
}

/** Recompute the esp_wifi mode from (configured mode − suspensions). */
static esp_err_t apply_suspend_state(void)
{
    const wm_config_t *cfg = wm_settings_config();
    bool want_sta = wifi_manager_mode_has_sta() && !s_sta_susp;
    bool want_ap = wifi_manager_mode_has_ap() && !s_ap_susp;

    /* respect ap_auto_disable: don't resurrect an auto-disabled AP */
    if (want_ap && cfg->ap_auto_disable && wifi_manager_is_sta_connected())
    {
        want_ap = false;
    }

    wifi_mode_t target = want_sta && want_ap ? WIFI_MODE_APSTA
                         : want_sta          ? WIFI_MODE_STA
                         : want_ap           ? WIFI_MODE_AP
                                             : WIFI_MODE_NULL;
    wifi_mode_t current;

    if (esp_wifi_get_mode(&current) != ESP_OK)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (current == target)
    {
        return ESP_OK;
    }

    bool had_sta = (current == WIFI_MODE_STA || current == WIFI_MODE_APSTA);

    if (had_sta && !want_sta)
    {
        esp_wifi_disconnect(); /* clean leave before the iface drops */
    }

    /* adding STA fires WIFI_EVENT_STA_START -> select_and_connect */
    return esp_wifi_set_mode(target);
}

esp_err_t wifi_manager_suspend_sta(void)
{
    if (!wm_core_started() || !wifi_manager_mode_has_sta())
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_sta_susp)
    {
        ESP_LOGI(TAG, "STA suspended (interface policy)");
        s_sta_susp = true;
    }

    return apply_suspend_state();
}

esp_err_t wifi_manager_resume_sta(void)
{
    if (!wm_core_started())
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_sta_susp)
    {
        ESP_LOGI(TAG, "STA resumed (interface policy)");
        s_sta_susp = false;
    }

    return apply_suspend_state();
}

esp_err_t wifi_manager_suspend_ap(void)
{
    if (!wm_core_started() || !wifi_manager_mode_has_ap())
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_ap_susp)
    {
        ESP_LOGI(TAG, "AP suspended (interface policy)");
        s_ap_susp = true;
    }

    return apply_suspend_state();
}

esp_err_t wifi_manager_resume_ap(void)
{
    if (!wm_core_started())
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_ap_susp)
    {
        ESP_LOGI(TAG, "AP resumed (interface policy)");
        s_ap_susp = false;
    }

    return apply_suspend_state();
}
