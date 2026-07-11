/**
 * @file wifi_manager.h
 * @brief WiCAN WiFi manager (feature component).
 *
 * Owns the WiFi radio: STA with prioritized fallback networks and an
 * auth-failure ban list, AP with static IP + DHCP server, APSTA with
 * AP-auto-disable and AP-channel-follows-STA, and a supervised reconnect
 * task.
 *
 * Configuration is a settings_manager descriptor (name "wifi_manager") and
 * applies ONLY at boot (Coding Standard §4.2 reboot-to-apply): there is no
 * runtime set-mode / set-credentials API. Changing WiFi settings = PUT via
 * the settings transport, then reboot. The runtime surface below is status,
 * scanning, and event callbacks only.
 *
 * Lifecycle (composition in main):
 *   wifi_manager_init()      — allocate state, register the settings
 *                              descriptor. Before settings_manager_start().
 *   settings_manager_start() — runs on_apply (stores config; touches no HW)
 *   wifi_manager_start()     — netifs + esp_wifi up per config; refuses with
 *                              ESP_ERR_INVALID_STATE if unconfigured (§4.3).
 *   wifi_manager_stop()      — stops the radio and the reconnect task.
 *
 * Requires nvs_flash_init() before start (esp_wifi dependency); main owns
 * that, per §3 ("a component assumes its dependencies are already init'd").
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Event-group bits for external status monitoring (wifi_manager_get_event_group). */
#define WIFI_MANAGER_BIT_STA_CONNECTED   BIT0  /* associated + got IP        */
#define WIFI_MANAGER_BIT_STA_DISCONNECTED BIT1
#define WIFI_MANAGER_BIT_AP_STARTED      BIT2
#define WIFI_MANAGER_BIT_ENABLED         BIT3  /* radio started              */
#define WIFI_MANAGER_BIT_STA_AP_OVERLAP  BIT4  /* STA and AP subnets overlap */

/** Event callbacks. All run in the default event loop task — keep them short,
 *  never block, never call back into wifi_manager stop/start from them. */
typedef struct
{
    void (*sta_connected)(void);                              /* got IP      */
    void (*sta_disconnected)(void);
    void (*ap_station_connected)(const uint8_t *mac, uint8_t aid);
    void (*ap_station_disconnected)(const uint8_t *mac, uint8_t aid);
} wifi_manager_callbacks_t;

/** Allocate state and register the settings descriptor ("wifi_manager"). */
esp_err_t wifi_manager_init(void);

/** Bring the radio up per the boot-applied settings. ESP_ERR_INVALID_STATE
 *  if the component was left unconfigured by the settings boot pass, ESP_OK
 *  (radio off) when the configured mode is "off". */
esp_err_t wifi_manager_start(void);

/** Stop the reconnect task and the radio. */
esp_err_t wifi_manager_stop(void);

/* ---- status ------------------------------------------------------------- */

bool wifi_manager_is_enabled(void);
bool wifi_manager_is_sta_connected(void);
bool wifi_manager_is_ap_started(void);

/** Copy the current STA IPv4 address ("" when not connected). */
esp_err_t wifi_manager_get_sta_ip(char *buf, size_t buf_len);

/** Number of stations currently associated to our AP. */
uint16_t wifi_manager_get_ap_station_count(void);

/** Copy the AP's current gateway IP (dotted quad) into @p buf.
 *  ESP_ERR_INVALID_STATE when the AP interface has no address. */
esp_err_t wifi_manager_get_ap_ip(char *buf, size_t buf_len);

/** IPv4 DNS servers as strings; "N/A" written when unavailable. */
esp_err_t wifi_manager_get_sta_dns(char *dns_main, size_t dns_main_len,
                                   char *dns_backup, size_t dns_backup_len);

/** Event group carrying the WIFI_MANAGER_BIT_* bits, for waiters. */
EventGroupHandle_t wifi_manager_get_event_group(void);

/* ---- runtime operations --------------------------------------------------- */

/**
 * Blocking scan; returns a malloc'd JSON string
 * {"networks":[{"ssid","rssi","channel","auth_mode","bssid"},...]} the
 * caller frees, or NULL on error. Temporarily switches AP-only mode to
 * APSTA for the scan and restores it after.
 */
char *wifi_manager_scan_networks(void);

/** Register (or clear, with NULL) event callbacks. */
esp_err_t wifi_manager_set_callbacks(const wifi_manager_callbacks_t *cbs);

/* ---- runtime interface suspension (interface_manager's actuators) ----------
 * EPHEMERAL: settings untouched, a reboot restores the configured mode.
 * Suspending STA also parks the reconnect task; resuming reconnects.
 * ESP_ERR_INVALID_STATE when not started or the configured mode doesn't
 * include the interface. */
esp_err_t wifi_manager_suspend_sta(void);
esp_err_t wifi_manager_resume_sta(void);
esp_err_t wifi_manager_suspend_ap(void);
esp_err_t wifi_manager_resume_ap(void);

/** Whether the BOOT-CONFIGURED mode includes STA / AP (regardless of any
 *  runtime suspension) — policy inputs for interface_manager. */
bool wifi_manager_mode_has_sta(void);
bool wifi_manager_mode_has_ap(void);

/**
 * Register the component's /api/wifi/status + /api/wifi/scan routes with
 * http_server_manager (HTTP_API.md §7). Called by main ONLY in compositions
 * that include the HTTP server, after http_server_manager_init(); the rest
 * of the component never touches HTTP.
 */
/**
 * Network-trust admission gate for http_server_manager (wired by main):
 * false when the current STA network is marked untrusted AND the request
 * arrived via the STA address. Own-AP/USB requests always pass; outbound
 * clients are unaffected (meatpi 2026-07-08 — shared networks must not
 * expose configuration).
 */
bool wifi_manager_http_request_allowed(int sockfd);

esp_err_t wifi_manager_register_http(void);

/** Register the `wifi` CLI command with cmdline_manager. Called
 *  INTERNALLY on the settings boot apply when the `cli` setting is true
 *  (default) — main no longer wires it. */
esp_err_t wifi_manager_register_cli(void);

#ifdef __cplusplus
}
#endif
