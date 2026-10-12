/**
 * @file wifi_modem.h
 * @brief WiFi-modem mode (feature component): the ESPNetLink serves a WiCAN
 *        (or any WiFi client) internet through its LTE uplink over its own
 *        WiFi AP, with the USB-C port carrying POWER ONLY once a host has
 *        asked for the data lines to be cut.
 *
 * Why: the WiCAN's USB host transmitter desenses the GNSS front-end while
 * the USB-NCM link is active (a cold fix never completes). With the USB
 * data lines cut at the mux and the WiCAN joined to this AP instead, the
 * bench measured aided cold TTFF medians of ~65 s with the whole stack live
 * (docs/espnetlink_wifi_mode_plan.md, CH343_ROUTE_RESULTS.md 2026-08-22).
 *
 * 2026-10-12: nothing about a cut persists any more. Every boot starts with
 * the USB data lines on the DIP route and the USB device stack up, so a
 * user who forgot the AP password always has the USB link (the web UI at
 * 192.168.7.1 and the CDC console) to get back in. The host cuts the lines
 * at runtime after every one of its own pairing passes. The former
 * `boot_cut` hint, the boot-time cut and the 60 s self-restore are gone,
 * and `power_only` (a cut at boot with no way in) became `console`.
 *
 * What the component owns (standard §3/§4/§6/§6b):
 *  - settings descriptor "wifi_modem" v2: usb_mode = auto | ncm | console,
 *    ap_share, ap_password_provisioned, cli (reboot-to-apply); the v1 to v2
 *    migration maps power_only to console and drops boot_cut.
 *  - the boot-time USB route: wifi_modem_apply_usb_route() releases any
 *    held mux pins and leaves the DIP route (USB data on) in every mode;
 *    main starts usb_dev_ethernet in every mode, with the Ethernet function
 *    off in `console` (wifi_modem_usb_ethernet_enabled()).
 *  - the runtime USB data cut (usb_mode auto or console): the host on the
 *    cable, the WiCAN, reads the AP credentials over USB
 *    (GET /api/wifi_modem/credentials, never served over WiFi) and then
 *    asks for the data lines to be cut (POST /api/wifi_modem/usb_data
 *    {"enabled":false}). The cut is an electrical unplug at the mux: the
 *    device stack idles exactly as with no host attached; "enabled":true
 *    releases the pins to the DIP route and the host re-enumerates. No
 *    reboot in either direction, and the next boot is USB on again.
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
 *   wifi_modem_init()            - log + settings descriptors; between
 *                                  settings_manager_init() and _start().
 *   settings_manager_start()     - runs on_apply (stores the config).
 *   wifi_modem_provision(&rb)    - one-time AP password provisioning; main
 *                                  reboots once when rb is true.
 *   wifi_modem_apply_usb_route() - releases the mux for the DIP route;
 *                                  BEFORE any USB device stack start.
 *   wifi_modem_register_http()   - before http_server_manager_start().
 *   wifi_modem_start()           - after wifi_manager_start(): arms sharing.
 *   wifi_modem_stop()            - NAPT off, proxy down, handlers gone.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** wifi_manager's schema-default AP password, never shipped on a device
 *  running this component (see wifi_modem_provision). */
#define WIFI_MODEM_STOCK_AP_PASSWORD "12345678"

/** Length of the generated per-device AP password (characters). */
#define WIFI_MODEM_AP_PASSWORD_LEN 12

typedef enum
{
    WIFI_MODEM_USB_AUTO = 0,     /**< USB Ethernet + console at every boot; the
                                      host may cut the data lines at runtime
                                      (the WiCAN flow); nothing persists    */
    WIFI_MODEM_USB_NCM,          /**< USB Ethernet + console, the runtime cut
                                      refused (locked on)                   */
    WIFI_MODEM_USB_CONSOLE,      /**< WiFi AP is the product; USB presents
                                      the CDC console ONLY (no Ethernet), so
                                      the owner can always get back in      */
} wifi_modem_usb_mode_t;

/** Boot-applied configuration (settings descriptor "wifi_modem" v2). */
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
 * ap_password_provisioned is persisted true so this never runs again: a
 * user who later chooses the stock password deliberately keeps it.
 * Call after settings_manager_start() and before wifi_manager_start().
 */
esp_err_t wifi_modem_provision(bool *reboot_required);

/**
 * Apply the boot-time USB route. Call once, after settings_manager_start()
 * and before any USB device stack start. Every mode boots with the data
 * lines on the DIP route (a host can always reach the device after a
 * power cycle); the call releases a mux hold left by deep sleep and logs
 * the mode.
 */
esp_err_t wifi_modem_apply_usb_route(void);

/** True when the USB device must present the Ethernet function (usb_mode
 *  auto or ncm, or the component unconfigured: the legacy behaviour is the
 *  safe default). False in `console`: the CDC console alone. */
bool wifi_modem_usb_ethernet_enabled(void);

/**
 * Runtime USB data switch (the WiCAN flow). false cuts the data lines at
 * the mux (electrical unplug; the device stack idles as with no host),
 * true releases the pins back to the DIP route so the host re-enumerates.
 * Nothing persists: the next boot is USB on. ESP_ERR_NOT_SUPPORTED when
 * usb_mode = ncm (locked on), ESP_ERR_INVALID_STATE when unconfigured.
 * Idempotent.
 */
esp_err_t wifi_modem_usb_data_set(bool enabled);

/** Current runtime state of the USB data lines (true = routed to the S3). */
bool wifi_modem_usb_data_routed(void);

/** Register GET /api/wifi_modem, GET /api/wifi_modem/credentials (non-WiFi
 *  interfaces only) and POST /api/wifi_modem/usb_data, before
 *  http_server_manager_start(). */
esp_err_t wifi_modem_register_http(void);

/** Register the `wifi_modem` console command with cmdline_manager. Called
 *  INTERNALLY on the settings boot apply when the `cli` setting is true
 *  (default); main wires nothing (§6b). */
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
