/**
 * @file web_ui.h
 * @brief The built-in web UI (WiCAN Pro v2 design, single-page app) —
 *        registers its embedded gzipped assets with http_server_manager.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Register the UI asset table (call between http_server_manager_init and
 *  _start). API/WS routes always win — the catch-all is installed last. */
esp_err_t web_ui_register(void);

#ifdef __cplusplus
}
#endif
