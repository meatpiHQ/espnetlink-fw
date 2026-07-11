/**
 * @file restart_tracker.h
 * @brief WiCAN restart tracker (core component).
 *
 * Answers "why did the device reboot?" across warm restarts without touching
 * flash: its state lives in PSRAM `.noinit` (EXT_RAM_NOINIT_ATTR) guarded by
 * a magic/version/CRC envelope. A power cycle leaves random PSRAM — the
 * envelope detects that and starts fresh; any warm reset (esp_restart, panic,
 * watchdog, brownout-less resets) preserves the full history.
 *
 * Two jobs:
 *  1. Every boot, record one history entry: reset reason, timestamp (when
 *     wall time is valid), and — if the previous run declared it — the
 *     planned reason/source of the restart.
 *  2. Give the firmware ONE sanctioned way to reboot on purpose:
 *     restart_tracker_restart(reason, source, flags). Transports that call
 *     esp_restart() directly rob the next boot of its "why".
 *
 * Lifecycle: restart_tracker_init() must run EARLY in main (right after
 * log_manager_init) so the boot record is written before anything can crash.
 * start()/stop() exist for lifecycle uniformity (Coding Standard §3) and are
 * no-ops — the component is passive after init.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RESTART_TRACKER_HISTORY_LEN 8

typedef enum
{
    RESTART_TRACKER_PLANNED_REASON_NONE = 0,
    RESTART_TRACKER_PLANNED_REASON_USER_REQUEST,
    RESTART_TRACKER_PLANNED_REASON_CONFIG_APPLY,     /* settings submit-then-reboot */
    RESTART_TRACKER_PLANNED_REASON_CONFIG_RECOVERY,
    RESTART_TRACKER_PLANNED_REASON_OTA_APPLY,
    RESTART_TRACKER_PLANNED_REASON_FACTORY_RESET,
    RESTART_TRACKER_PLANNED_REASON_SAFE_MODE,
    RESTART_TRACKER_PLANNED_REASON_POWER_WAKE,
    RESTART_TRACKER_PLANNED_REASON_INTERNAL_RECOVERY,
} restart_tracker_planned_reason_t;

typedef enum
{
    RESTART_TRACKER_SOURCE_UNKNOWN = 0,
    RESTART_TRACKER_SOURCE_WEB_UI,
    RESTART_TRACKER_SOURCE_CMDLINE,
    RESTART_TRACKER_SOURCE_CONSOLE,
    RESTART_TRACKER_SOURCE_MQTT,
    RESTART_TRACKER_SOURCE_OTA,
    RESTART_TRACKER_SOURCE_SAFE_MODE,
    RESTART_TRACKER_SOURCE_CONFIG_SERVER,
    RESTART_TRACKER_SOURCE_SLEEP_MODE,
} restart_tracker_source_t;

/** One boot in the history ring. */
typedef struct
{
    uint32_t sequence;            /**< monotonically increasing boot number   */
    int64_t  boot_timestamp;      /**< unix time at boot; 0 if clock invalid  */
    int64_t  request_timestamp;   /**< when the restart was requested (planned) */
    uint64_t request_uptime_ms;   /**< uptime at the restart request           */
    uint32_t actual_reset_reason; /**< esp_reset_reason_t of THIS boot          */
    uint32_t flags;               /**< caller-defined flags from the request    */
    uint16_t planned_reason;      /**< restart_tracker_planned_reason_t         */
    uint16_t source;              /**< restart_tracker_source_t                 */
    uint8_t  was_planned;         /**< 1 if the previous run announced it       */
    uint8_t  time_valid;          /**< 1 if boot_timestamp is trustworthy       */
    uint8_t  reserved[2];
} restart_tracker_record_t;

/** Restart intent left for the NEXT boot to consume. */
typedef struct
{
    int64_t  requested_timestamp;
    uint64_t requested_uptime_ms;
    uint32_t flags;
    uint16_t planned_reason;
    uint16_t source;
    uint8_t  valid;
    uint8_t  time_valid;
    uint8_t  reserved[2];
} restart_tracker_pending_t;

/**
 * The PSRAM-resident state. `crc32` MUST stay the last member: the CRC spans
 * [magic, offsetof(crc32)), so layout changes require a `version` bump.
 *
 * `mspi_tuning_guard` MUST stay the first member and outside the CRC: on the
 * ESP32-S3, MSPI PSRAM timing tuning writes a 64-byte test pattern at PSRAM
 * physical address 0 on EVERY boot — and `.ext_ram_noinit` starts there. If
 * this object links first in the section, only the guard is sacrificed.
 */
typedef struct
{
    uint8_t  mspi_tuning_guard[64];
    uint32_t magic;
    uint32_t version;
    uint32_t history_len;
    uint32_t boot_count;
    uint32_t unexpected_reset_count; /**< panics/watchdogs, not sw/poweron/sleep */
    uint32_t record_sequence;
    uint32_t latest_history_index;
    uint32_t next_history_index;
    restart_tracker_pending_t pending_restart;
    restart_tracker_record_t  history[RESTART_TRACKER_HISTORY_LEN];
    uint32_t crc32;
} restart_tracker_state_t;

/** Validate/adopt the PSRAM state and record this boot. Call early in main. */
esp_err_t restart_tracker_init(void);

/** Lifecycle uniformity (§3); passive component — both return ESP_OK. */
esp_err_t restart_tracker_start(void);
esp_err_t restart_tracker_stop(void);

/**
 * Announce an intentional restart WITHOUT performing it (for callers that
 * must do their own teardown before esp_restart()).
 */
esp_err_t restart_tracker_mark_planned_restart(restart_tracker_planned_reason_t reason,
                                               restart_tracker_source_t source,
                                               uint32_t flags);

/** The one sanctioned reboot: mark planned, then esp_restart(). No return. */
void restart_tracker_restart(restart_tracker_planned_reason_t reason,
                             restart_tracker_source_t source,
                             uint32_t flags) __attribute__((noreturn));

/** Snapshot the whole state (history ring included). */
esp_err_t restart_tracker_get_state(restart_tracker_state_t *out_state);

/** The record for THIS boot. ESP_ERR_NOT_FOUND before init has recorded it. */
esp_err_t restart_tracker_get_latest_record(restart_tracker_record_t *out_record);

/** Register the `restart_tracker` CLI command with cmdline_manager.
 *  Called INTERNALLY on the settings boot apply when the `cli` setting is
 *  true (default) — main no longer wires it. */
esp_err_t restart_tracker_register_cli(void);

/** Register the settings descriptor ({cli}). Init runs before
 *  settings_manager_init, so the composition root calls this separately
 *  (the log_manager_register_settings pattern). */
esp_err_t restart_tracker_register_settings(void);

/* human-readable names (for logs / status JSON) */
const char *restart_tracker_reset_reason_to_str(uint32_t esp_reset_reason);
const char *restart_tracker_planned_reason_to_str(restart_tracker_planned_reason_t reason);
const char *restart_tracker_source_to_str(restart_tracker_source_t source);

#ifdef __cplusplus
}
#endif
