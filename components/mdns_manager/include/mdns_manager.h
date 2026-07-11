/**
 * @file mdns_manager.h
 * @brief WiCAN mDNS advertisement owner (feature component).
 *
 * ESPNetLink port of the WiCAN rewrite of the legacy `wc_mdns.c`. The
 * ON-AIR CONTRACT the Home Assistant integration discovers by:
 *
 *  - hostname `espnetlink_<device_id>` (→ `espnetlink_<id>.local`)
 *  - default instance "espnetlink web server"
 *  - service "ESPNetLink-WebServer" of type **`_meatpi._tcp` on
 *    port 80** — device-contract v2 (2026-07-11,
 *    ha_webhooks/device-contract): the brand-wide discovery surface
 *    every MeatPi product shares (integration 3.0 matches only
 *    `_meatpi`/`_wican` types). ESPNetLink advertises `_meatpi` ONLY;
 *    `_wican._tcp` is the WiCAN-family legacy service and is not
 *    registered here.
 *  - TXT keys (contract v2): `device_type` (Kconfig
 *    `WICAN_DEVICE_TYPE`, profile slug), `device_id`, `mac` (STA MAC,
 *    colon-separated uppercase — HA's stable unique ID), `fw` (app
 *    version), `api` ("6").
 *
 * esp-mdns handles interface up/down itself, so start() works before
 * the network is up; responses begin when an interface appears.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Register settings ("mdns_manager") + log descriptors. No network. */
esp_err_t mdns_manager_init(void);

/** Start the responder + advertise the _meatpi service (no-op when
 *  disabled in settings). ESP_ERR_INVALID_STATE when unconfigured. */
esp_err_t mdns_manager_start(void);

/** Withdraw the service and stop the responder. */
esp_err_t mdns_manager_stop(void);

/** "espnetlink_<id>.local" (the legacy wc_mdns_get_hostname role).
 *  Valid after start(). */
const char *mdns_manager_hostname(void);

#ifdef __cplusplus
}
#endif
