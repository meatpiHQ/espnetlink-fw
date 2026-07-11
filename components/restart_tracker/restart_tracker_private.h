/**
 * @file restart_tracker_private.h
 * @brief Pure state-machine core (restart_tracker_core.c) — no IDF deps, so
 *        the host unit tests compile it directly. Time, uptime, and the reset
 *        reason are injected by the caller; restart_tracker.c is the target
 *        glue (locking, PSRAM noinit placement, cache msync, esp_* sources).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "restart_tracker.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RT_MAGIC               0x5254524BU /* "RTRK" */
#define RT_VERSION             1U
#define RT_MIN_VALID_UNIX_TIME 1704067200LL /* 2024-01-01: clock sanity floor */

/* injected boot-time inputs */
typedef struct
{
    int64_t  now_unix;        /**< time(NULL); may be pre-epoch garbage      */
    uint64_t uptime_ms;       /**< since boot                                */
    uint32_t reset_reason;    /**< esp_reset_reason_t value                  */
} rt_inputs_t;

uint32_t rt_crc32(const restart_tracker_state_t *state);
bool     rt_state_is_valid(const restart_tracker_state_t *state);
void     rt_state_reset(restart_tracker_state_t *state);

/** Adopt-or-reset, then append this boot's record (consumes any pending
 *  intent). Returns true if the previous state was invalid and got reset. */
bool rt_record_boot(restart_tracker_state_t *state, const rt_inputs_t *in);

/** Store a planned-restart intent for the next boot. */
void rt_mark_planned(restart_tracker_state_t *state, const rt_inputs_t *in,
                     restart_tracker_planned_reason_t reason,
                     restart_tracker_source_t source, uint32_t flags);

bool rt_reset_reason_is_unexpected(uint32_t reason);

#ifdef __cplusplus
}
#endif
