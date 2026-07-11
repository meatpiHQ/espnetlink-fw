/**
 * @file api_http.h
 * @brief ESPNetLink device HTTP API glue (MeatPi standard §6).
 *
 * Implements the core components' `/api/settings` routes on top of
 * http_server_manager: get/put/schema/list/submit, whole-device backup and
 * restore, and factory reset. Core components never touch HTTP — this glue
 * depends DOWN on them (never the reverse).
 *
 * Conventions (standard §6): JSON in/out, GET never mutates, password
 * redaction on reads ("" on PUT = keep stored), PUT is full-object replace,
 * submit reboots only when a PUT in the batch actually changed something.
 *
 * ESPNetLink port note: WiCAN routes reboots through restart_tracker; that
 * component is not composed here yet, so the delayed reboot uses
 * esp_restart() directly.
 *
 * Lifecycle (called by main, in dependency order):
 *   http_server_manager_init();
 *   api_http_init();               // registers the /api routes (buffered)
 *   ...
 *   http_server_manager_start();   // routes go live, catch-all last
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Register the /api/settings routes with http_server_manager (buffered
 *  until the server starts) and the component's log descriptor. Call after
 *  http_server_manager_init(). */
esp_err_t api_http_init(void);

/** Lifecycle uniformity (§3); passive after init — both trivial. */
esp_err_t api_http_start(void);
esp_err_t api_http_stop(void);

#ifdef __cplusplus
}
#endif
