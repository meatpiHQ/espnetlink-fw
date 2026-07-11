/**
 * @file web_ui.h
 * @brief v6 (legacy) built-in web UI — registers its embedded ES-module
 *        assets with http_server_manager. No-op unless CONFIG_WICAN_WEBUI_V6
 *        is selected, so main can always call it.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Register the v6 asset table (call between http_server_manager_init and
 *  _start). Returns ESP_OK and does nothing if v6 isn't the selected UI. */
esp_err_t web_ui_register(void);

#ifdef __cplusplus
}
#endif
