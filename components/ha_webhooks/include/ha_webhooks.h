/**
 * @file ha_webhooks.h
 * @brief Home Assistant integration (v6 port of the legacy ha_webhooks +
 *        autopid webhook poster). Owns the HA telemetry link:
 *
 *   - inbound  `/api/webhook` GET/POST/DELETE — the HA HACS integration
 *              auto-registers its webhook URL here (discovery push);
 *   - outbound a PSRAM-stack poster task that, every `interval_s`, builds
 *              `{status, autopid_data, config}` and POSTs it to the
 *              webhook URL(s) with failover, while autopid is enabled and
 *              the network is up.
 *
 * Feature component: consumes http_server_manager, http_client_manager,
 * cert_manager, autopid (snapshot + cached config), dev_status_manager.
 * Config via settings_manager (`"ha_webhooks"`); the URL push applies
 * LIVE (documented reboot-to-apply exception, see TASK_ha_webhooks.md §7).
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Register the settings + log descriptors. No hardware, no network. */
esp_err_t ha_webhooks_init(void);

/** Start the poster task if enabled (else log + ESP_OK). */
esp_err_t ha_webhooks_start(void);

/** Stop the poster task and park. */
esp_err_t ha_webhooks_stop(void);

/** Register the `/api/webhook` routes (HTTP compositions only, §9.1). */
esp_err_t ha_webhooks_register_http(void);

/** Register the `webhook` console command (gated by the `cli` setting;
 *  called internally from on_apply — main wires nothing, §6b). */
esp_err_t ha_webhooks_register_cli(void);

#ifdef __cplusplus
}
#endif
