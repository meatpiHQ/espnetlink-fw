/**
 * @file wifi_modem.h
 * @brief WiFi-modem mode (feature component): the ESPNetLink serves a WiCAN
 *        (or any WiFi client) internet through its LTE uplink over its own
 *        WiFi AP, with the USB-C port carrying POWER ONLY.
 *
 * Why: the WiCAN's USB host transmitter desenses the GNSS front-end while
 * the USB-NCM link is active (a cold fix never completes). With the USB
 * data lines cut at the mux and the WiCAN joined to this AP instead, the
 * bench measured aided cold TTFF medians of ~65 s with the whole stack live
 * (docs/espnetlink_wifi_mode_plan.md, CH343_ROUTE_RESULTS.md 2026-08-22).
 *
 * What the component owns (standard §3/§4/§6/§6b):
 *  - settings descriptor "wifi_modem": usb_mode = auto | ncm | power_only,
 *    ap_share, ap_password_provisioned, cli — reboot-to-apply.
 *  - boot-time USB route: power_only drives USB_SEL_1/2 = 1/1 (every USB
 *    data path disconnected, VBUS still powers the board) right after the
 *    settings apply pass, BEFORE any USB device stack would start; main
 *    skips usb_dev_ethernet_start() when !wifi_modem_usb_data_enabled().
 *  - runtime USB data cut (usb_mode = auto): the host on the cable — the
 *    WiCAN — reads the AP credentials over USB (GET /api/wifi_modem/
 *    credentials, never served over WiFi) and then asks for the data lines
 *    to be cut (POST /api/wifi_modem/usb_data {"enabled":false}). The cut
 *    is an electrical unplug at the mux: the device stack idles exactly as
 *    with no host attached; "enabled":true releases the pins to the DIP
 *    route and the host re-enumerates. No reboot in either direction.
 *  - AP sharing: NAPT on the AP netif, DHCP offers the AP address as the
 *    DNS server, and a UDP/53 proxy forwards to the PPP-learned resolver
 *    (clients that joined BEFORE the PPP link came up keep resolving).
 *  - per-device AP password provisioning: the first boot replaces
 *    wifi_manager's stock "12345678" with a random 12-character password
 *    (the AP is the product's main interface in this mode; a shared stock
 *    password would let anyone join). Read it with `wifi_modem -k`.
 *  - GET /api/wifi_modem: one cheap health document for the WiCAN poller.
 *  - `wifi_modem` console command (self-registered on settings apply).
 *
 * Lifecycle (composition in main, in this order):
 *   wifi_modem_init()            — log + settings descriptors; between
 *                                  settings_manager_init() and _start().
 *   settings_manager_start()     — runs on_apply (stores the config).
 *   wifi_modem_provision(&rb)    — one-time AP password provisioning; main
 *                                  reboots once when rb is true.
 *   wifi_modem_apply_usb_route() — drives the mux for the configured mode;
 *                                  BEFORE any USB device stack start.
 *   wifi_modem_register_http()   — before http_server_manager_start().
 *   wifi_modem_start()           — after wifi_manager_start(): arms sharing.
 *   wifi_modem_stop()            — NAPT off, proxy down, handlers gone.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** wifi_manager's schema-default AP password — never shipped on a device
 *  running this component (see wifi_modem_provision). */
#define WIFI_MODEM_STOCK_AP_PASSWORD "12345678"

/** Length of the generated per-device AP password (characters). */
#define WIFI_MODEM_AP_PASSWORD_LEN 12

typedef enum
{
    WIFI_MODEM_USB_AUTO = 0,     /**< boot with USB data on; the host may cut
                                      it at runtime (the WiCAN flow)         */
    WIFI_MODEM_USB_NCM,          /**< USB data always on (runtime cut locked) */
    WIFI_MODEM_USB_POWER_ONLY,   /**< USB data cut at boot (5 V only)        */
} wifi_modem_usb_mode_t;

/** Boot-applied configuration (settings descriptor "wifi_modem"). */
typedef struct
{
    wifi_modem_usb_mode_t usb_mode;
    bool ap_share;                 /**< NAPT the AP netif into the LTE uplink */
    bool ap_password_provisioned;  /**< per-device AP password already set    */
    bool cli;
} wifi_modem_settings_t;

/* ---- lifecycle (§3) ------------------------------------------------------ */

/** Register the log descriptor and the "wifi_modem" settings descriptor.
 *  Call between settings_manager_init() and settings_manager_start(). */
esp_err_t wifi_modem_init(void);

/** Arm AP sharing (NAPT + DHCP DNS offer + DNS proxy) and the PPP uplink
 *  hook. Call after wifi_manager_start(). ESP_ERR_INVALID_STATE when the
 *  settings boot pass left the component unconfigured; ESP_OK (nothing
 *  armed) when ap_share is off or the WiFi mode has no AP. */
esp_err_t wifi_modem_start(void);

/** NAPT off, DNS proxy down, IP event handler unregistered. */
esp_err_t wifi_modem_stop(void);

/* ---- boot-applied settings (§4.1 getters) -------------------------------- */
const wifi_modem_settings_t *wifi_modem_settings_config(void);
bool wifi_modem_settings_is_configured(void);

/* ---- boot hooks (main) --------------------------------------------------- */

/**
 * One-time per-device AP password provisioning. When the "wifi_modem"
 * settings say the password was never provisioned and wifi_manager still
 * holds the stock "12345678", a random WIFI_MODEM_AP_PASSWORD_LEN-character
 * password is persisted into wifi_manager's settings (reboot-to-apply:
 * @p reboot_required comes back true and main reboots once, the way the
 * legacy-config import does). A custom password is left alone. Either way
 * ap_password_provisioned is persisted true so this never runs again — a
 * user who later chooses the stock password deliberately keeps it.
 * Call after settings_manager_start() and before wifi_manager_start().
 */
esp_err_t wifi_modem_provision(bool *reboot_required);

/**
 * Apply the USB route for the configured mode. Call once, after
 * settings_manager_start() and before any USB device stack start. In
 * power_only it drives the mux to 1/1; in ncm it leaves the DIP route
 * untouched (today's behaviour).
 */
esp_err_t wifi_modem_apply_usb_route(void);

/** True when the USB data path should be brought up at boot (usb_mode is
 *  auto or ncm, or the component is unconfigured — legacy behaviour is the
 *  safe default). */
bool wifi_modem_usb_data_enabled(void);

/**
 * Runtime USB data switch (the WiCAN flow, usb_mode = auto). false cuts
 * the data lines at the mux (electrical unplug; the device stack idles as
 * with no host), true releases the pins back to the DIP route so the host
 * re-enumerates. ESP_ERR_NOT_SUPPORTED when usb_mode = ncm (locked on),
 * ESP_ERR_INVALID_STATE when unconfigured. Idempotent.
 */
esp_err_t wifi_modem_usb_data_set(bool enabled);

/** Current runtime state of the USB data lines (true = routed to the S3). */
bool wifi_modem_usb_data_routed(void);

/** Register GET /api/wifi_modem, GET /api/wifi_modem/credentials (non-WiFi
 *  interfaces only) and POST /api/wifi_modem/usb_data — before
 *  http_server_manager_start(). */
esp_err_t wifi_modem_register_http(void);

/** Register the `wifi_modem` console command with cmdline_manager. Called
 *  INTERNALLY on the settings boot apply when the `cli` setting is true
 *  (default) — main wires nothing (§6b). */
esp_err_t wifi_modem_register_cli(void);

/* ---- status (HTTP/CLI views) --------------------------------------------- */
bool wifi_modem_napt_active(void);
uint32_t wifi_modem_upstream_dns(void);   /**< network order, 0 = none yet */

/* ---- persist helpers (reboot to apply) ----------------------------------- */
esp_err_t wifi_modem_settings_persist_str(const char *field, const char *value);
esp_err_t wifi_modem_settings_persist_bool(const char *field, bool value);

#ifdef __cplusplus
}
#endif
