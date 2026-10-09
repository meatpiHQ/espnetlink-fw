/**
 * @file lte_upstream_pppos_private.h
 * @brief Internals shared between lte_upstream_pppos.c and the files split
 *        out of it per §1 (lte_upstream_pppos_cli.c,
 *        lte_upstream_pppos_monitor.c). Not public API.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_err.h"

#include "lte_upstream_pppos.h"

#if CONFIG_PPP_SUPPORT
#include "esp_modem_api.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* PPP/link state event group + bits (defined in lte_upstream_pppos.c) */
extern EventGroupHandle_t s_evt;
extern const EventBits_t BIT_CONNECTED;
extern const EventBits_t BIT_DISCONNECTED;
extern const EventBits_t BIT_NETWORK_DETACHED;

#if CONFIG_PPP_SUPPORT
/* Engine state (defined in lte_upstream_pppos.c) */
extern esp_modem_dce_t *s_dce;          /* NULL when stopped */
extern volatile bool s_monitor_running;
extern TaskHandle_t s_monitor_task;
extern volatile bool s_supervisor_running;
extern volatile bool s_stop_requested;
extern TaskHandle_t s_supervisor_task;
extern volatile bool s_time_synced;
extern portMUX_TYPE s_status_mux;
extern lte_status_t s_status;           /* protected by s_status_mux */
extern lte_upstream_pppos_config_t s_saved_cfg;

/* lte_upstream_pppos.c: engine lifecycle used by the supervisor task */
esp_err_t lte_inner_start(const lte_upstream_pppos_config_t *cfg);
void lte_inner_stop(void);

/* lte_upstream_pppos.c: what the bring-up learned about the SIM (the
 * attach wait in _at.c reports it; GET /api/lte and the health show it) */
void lte_sim_note(lte_sim_state_t s);

/* lte_upstream_pppos_at.c: SIM readiness + attach/registration wait */
esp_err_t wait_for_cellular_attach(esp_modem_dce_t *dce, uint32_t timeout_ms,
                                   const char *pin,
                                   bool reboot_on_timeout,
                                   uint32_t reboot_wait_ms);

/* lte_upstream_pppos_monitor.c: task entry + creation helpers */
void lte_time_sync_task(void *arg);
TaskHandle_t lte_monitor_start(void);
TaskHandle_t lte_supervisor_start(void);
#endif /* CONFIG_PPP_SUPPORT */

#ifdef __cplusplus
}
#endif
