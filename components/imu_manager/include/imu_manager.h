/**
 * @file imu_manager.h
 * @brief WiCAN motion sensor owner (feature component).
 *
 * Owns the ICM-42670-P IMU (shared bus via i2c_bus; vendored espressif
 * driver) and runs BOTH of its motion detectors side by side. The
 * detector task POLLS INT_STATUS2 every 200 ms — the bits latch until
 * read, so nothing is missed and there is no ISR (meatpi's call: safer,
 * and nothing here is time-critical; the chip's INT1 pin stays wired
 * but unused):
 *
 *  - SMD (DMP/APEX, needs SUSTAINED motion) drives the ACTIVITY state —
 *    "the vehicle is moving": ACTIVE (+ DEV_STATUS_BIT_MOTION) until
 *    quiet for `stationary_s` → STATIONARY. Sleep/drive logic consumes
 *    the dev-status bit.
 *  - WoM (per-sample threshold, PREVIOUS-sample reference so a slope or
 *    orientation change can't latch it) catches single bumps — door
 *    open/close, a knock, vibration. It does NOT touch the activity
 *    state; it is published as an EVENT.
 *
 * EVENTS: any component subscribes a queue (`imu_manager_event_t` items)
 * and reacts — send a CAN frame on a door slam, an MQTT alert on motion,
 * whatever the product wants. Fan-out is drop-and-count per subscriber
 * (never blocks the detector task); WOM publishes are throttled so
 * driving vibration can't flood the queues. Activity transitions are
 * published too, so most consumers never need to poll.
 *
 * The accel runs permanently in low-power mode feeding both detectors
 * (reads work anytime); the gyro stays OFF in v1 — nothing consumes it,
 * and it dominates the chip's power budget.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_MANAGER_MAX_SUBSCRIBERS 4

typedef enum
{
    IMU_MANAGER_STATIONARY = 0,
    IMU_MANAGER_ACTIVE,
    IMU_MANAGER_UNKNOWN,      /* not started / chip absent               */
} imu_manager_activity_t;

typedef enum
{
    IMU_MANAGER_EVENT_WOM = 0,    /* single bump (door, knock, vibration);
                                     throttled to one publish/500 ms     */
    IMU_MANAGER_EVENT_SMD,        /* significant (sustained) motion      */
    IMU_MANAGER_EVENT_ACTIVE,     /* activity transition (SMD-driven)    */
    IMU_MANAGER_EVENT_STATIONARY, /* activity transition                 */
} imu_manager_event_type_t;

typedef struct
{
    imu_manager_event_type_t type;
    uint8_t  wom_axes;   /* WOM only: bit0=Z bit1=Y bit2=X (INT_STATUS2) */
    uint32_t uptime_ms;  /* when the detector task saw it                */
} imu_manager_event_t;

/** Register settings ("imu_manager") + log descriptors. No bus traffic. */
esp_err_t imu_manager_init(void);

/** Probe the chip, configure WoM, hook the INT GPIO, start the activity
 *  task. ESP_ERR_INVALID_STATE when unconfigured (§4.3 step 5). */
esp_err_t imu_manager_start(void);
esp_err_t imu_manager_stop(void);

/** Current debounced activity state. */
imu_manager_activity_t imu_manager_activity(void);

/**
 * Attach a subscriber queue (items imu_manager_event_t, caller-owned).
 * Every motion event is fanned out to every subscriber with a
 * zero-timeout send — a full queue drops THAT subscriber's copy (counted,
 * never blocks the detector). Up to IMU_MANAGER_MAX_SUBSCRIBERS;
 * ESP_ERR_NO_MEM when the table is full.
 */
esp_err_t imu_manager_subscribe(QueueHandle_t q);
esp_err_t imu_manager_unsubscribe(QueueHandle_t q);

/** Acceleration in g (accel is always on in LP mode). */
esp_err_t imu_manager_read_accel(float *ax, float *ay, float *az);

/** Die temperature in °C. */
esp_err_t imu_manager_read_temp(float *temp);

/** WHO_AM_I (0x67 for ICM-42670-P) — diagnostics. */
esp_err_t imu_manager_device_id(uint8_t *id);

/** Register GET /api/imu (composition root calls it in HTTP builds). */
esp_err_t imu_manager_register_http(void);

/** Register the `imu` CLI command with cmdline_manager. Called
 *  INTERNALLY on the settings boot apply when the `cli` setting is true
 *  (default) — main no longer wires it. */
esp_err_t imu_manager_register_cli(void);

/** ESPNetLink extension: arm ICM-42670 wake-on-motion for deep sleep
 *  (the `sleep 4` CLI mode). Stops the detector, configures WoM on INT1
 *  (active HIGH -> EXT0 wake), drains latched statuses. */
esp_err_t imu_manager_prepare_wom_wake(uint8_t threshold);

/** ESPNetLink extension (imu_manager_selftest.c, the production test):
 *  electrical check of the INT1 trace to IMU_INT_PIN without motion. The
 *  pin is driven push-pull at both polarities with every interrupt source
 *  unrouted, and IMU_INT_PIN is read each time against the opposite weak
 *  pull. @p level_active_low / @p level_active_high (nullable) receive the
 *  two readings (expected 1 then 0). ESP_OK when both match, ESP_FAIL when
 *  the pin does not follow (open or shorted trace), ESP_ERR_INVALID_STATE
 *  before the chip came up. The chip's registers are restored. */
esp_err_t imu_manager_int1_check(int *level_active_low, int *level_active_high);

#ifdef __cplusplus
}
#endif
