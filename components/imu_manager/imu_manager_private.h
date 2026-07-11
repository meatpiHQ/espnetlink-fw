/**
 * @file imu_manager_private.h
 * @brief Internal contracts: the PURE activity state machine (time
 *        injected, no RTOS), the PURE settings migration — both
 *        host-testable — and the boot-applied settings surface.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* forward-declared so the state-machine users don't drag cJSON in */
typedef struct cJSON cJSON;

/** v1 (wom_threshold) -> v2 (smd_sensitivity). Mutates in place;
 *  missing new keys are filled from schema defaults by validation.
 *  (imu_manager_migrate.c — pure, host-tested.) */
esp_err_t imu_settings_migrate(uint32_t from_version, cJSON *settings);

/* ---- settings (imu_manager_settings.c) -------------------------------------- */

/** Boot-applied config (written only by the settings on_apply). */
typedef struct
{
    bool     enabled;
    bool     smd;               /* sustained -> activity                 */
    bool     wom;               /* bumps -> events                       */
    uint8_t  smd_sensitivity;   /* 0 = most sensitive                    */
    uint8_t  wom_threshold;     /* 1 LSB ~= 3.9 mg                       */
    uint32_t stationary_ms;
} imu_settings_t;

/** Register the "imu_manager" descriptor with settings_manager. */
esp_err_t imu_settings_register(void);

const imu_settings_t *imu_settings_config(void);
bool imu_settings_is_configured(void); /* boot apply ran (standard §4.3) */

/** States mirror imu_manager_activity_t (0 = stationary, 1 = active). */
typedef struct
{
    int      state;
    uint32_t last_motion_ms;
    uint32_t stationary_after_ms;
    uint32_t wom_throttle_ms;   /* min gap between published WOM events  */
    uint32_t last_wom_pub_ms;
    bool     wom_pub_primed;    /* first WOM always publishes            */
} imu_policy_t;

void imu_policy_init(imu_policy_t *p, uint32_t stationary_after_ms,
                     uint32_t wom_throttle_ms, uint32_t now_ms);

/** A significant-motion (SMD) event fired. Returns the new state. */
int imu_policy_on_motion(imu_policy_t *p, uint32_t now_ms);

/** Periodic tick. Returns the (possibly new) state. Wrap-safe. */
int imu_policy_on_tick(imu_policy_t *p, uint32_t now_ms);

/** WOM publish gate: true = publish this one, false = throttled.
 *  (Driving vibration fires WoM continuously — subscribers get at most
 *  one event per throttle window.) Wrap-safe. */
bool imu_policy_wom_gate(imu_policy_t *p, uint32_t now_ms);

/* event_manager glue (imu_manager_events.c) — ESPNetLink does not
 * compose an event_manager: the file is excluded from the build and the
 * call sites in imu_manager.c are no-ops. WiCAN declarations, for
 * reference:
 *   void imu_events_register(void);
 *   void imu_events_start(void);
 */

/* ---- ESPNetLink extension (imu_manager_wom.c) ------------------------------- */

/* forward-declared: the vendored driver handle (icm42670.h) */
typedef void *imu_mgr_dev_handle_t;

/** The component's icm42670 device handle (NULL before init succeeded).
 *  For the WoM deep-sleep extension only — everything else goes through
 *  the public API. */
imu_mgr_dev_handle_t imu_mgr_dev(void);
