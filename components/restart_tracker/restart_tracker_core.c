/**
 * @file restart_tracker_core.c
 * @brief Pure restart-tracking state machine. No IDF dependencies — compiled
 *        as-is by the host unit tests. All wall-clock/uptime/reset-reason
 *        inputs are injected (rt_inputs_t).
 */
#include "restart_tracker_private.h"

#include <stddef.h>
#include <string.h>

/* Reset-reason codes mirrored from esp_reset_reason_t (values are stable
 * IDF API). Kept as plain numbers so this file stays host-buildable. */
#define RT_RST_POWERON   1U
#define RT_RST_EXT       2U
#define RT_RST_SW        3U
#define RT_RST_DEEPSLEEP 5U

/* CRC-32 (IEEE 802.3, poly 0xEDB88320) — same convention as the settings
 * codec; local so the core has zero deps. */
static uint32_t crc32_bytes(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (int bit = 0; bit < 8; bit++)
        {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }

    return ~crc;
}

uint32_t rt_crc32(const restart_tracker_state_t *state)
{
    /* span [magic, crc32): the mspi_tuning_guard head is sacrificial and
       excluded — boot-time timing tuning may overwrite it (see header) */
    size_t start = offsetof(restart_tracker_state_t, magic);

    return crc32_bytes((const uint8_t *)state + start,
                       offsetof(restart_tracker_state_t, crc32) - start);
}

bool rt_state_is_valid(const restart_tracker_state_t *state)
{
    return state->magic == RT_MAGIC &&
           state->version == RT_VERSION &&
           state->history_len == RESTART_TRACKER_HISTORY_LEN &&
           rt_crc32(state) == state->crc32;
}

void rt_state_reset(restart_tracker_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->magic = RT_MAGIC;
    state->version = RT_VERSION;
    state->history_len = RESTART_TRACKER_HISTORY_LEN;
}

bool rt_reset_reason_is_unexpected(uint32_t reason)
{
    switch (reason)
    {
        case RT_RST_POWERON:
        case RT_RST_EXT: /* EN-pin reset: flasher/reset button = deliberate */
        case RT_RST_SW:
        case RT_RST_DEEPSLEEP:
            return false;
        default:
            return true; /* panic, watchdogs, brownout, unknown */
    }
}

static bool time_is_valid(int64_t now_unix)
{
    return now_unix >= RT_MIN_VALID_UNIX_TIME;
}

bool rt_record_boot(restart_tracker_state_t *state, const rt_inputs_t *in)
{
    bool was_reset = false;

    if (!rt_state_is_valid(state))
    {
        rt_state_reset(state); /* cold boot or corrupted PSRAM */
        was_reset = true;
    }

    uint32_t slot = state->next_history_index % RESTART_TRACKER_HISTORY_LEN;
    restart_tracker_record_t *record = &state->history[slot];

    memset(record, 0, sizeof(*record));
    record->sequence = ++state->record_sequence;
    record->boot_timestamp = time_is_valid(in->now_unix) ? in->now_unix : 0;
    record->time_valid = time_is_valid(in->now_unix) ? 1U : 0U;
    record->actual_reset_reason = in->reset_reason;

    if (state->pending_restart.valid)
    {
        record->request_timestamp = state->pending_restart.requested_timestamp;
        record->request_uptime_ms = state->pending_restart.requested_uptime_ms;
        record->planned_reason = state->pending_restart.planned_reason;
        record->source = state->pending_restart.source;
        record->flags = state->pending_restart.flags;
        record->was_planned = 1U;
    }

    state->boot_count++;

    if (!record->was_planned && rt_reset_reason_is_unexpected(in->reset_reason))
    {
        state->unexpected_reset_count++;
    }

    state->latest_history_index = slot;
    state->next_history_index = (slot + 1U) % RESTART_TRACKER_HISTORY_LEN;
    memset(&state->pending_restart, 0, sizeof(state->pending_restart));
    state->crc32 = rt_crc32(state);

    return was_reset;
}

void rt_mark_planned(restart_tracker_state_t *state, const rt_inputs_t *in,
                     restart_tracker_planned_reason_t reason,
                     restart_tracker_source_t source, uint32_t flags)
{
    if (!rt_state_is_valid(state))
    {
        rt_state_reset(state);
    }

    state->pending_restart.requested_timestamp =
        time_is_valid(in->now_unix) ? in->now_unix : 0;
    state->pending_restart.requested_uptime_ms = in->uptime_ms;
    state->pending_restart.flags = flags;
    state->pending_restart.planned_reason = (uint16_t)reason;
    state->pending_restart.source = (uint16_t)source;
    state->pending_restart.valid = 1U;
    state->pending_restart.time_valid = time_is_valid(in->now_unix) ? 1U : 0U;
    state->crc32 = rt_crc32(state);
}

/* ---- name tables (pure, shared by logs and status JSON) ------------------- */

const char *restart_tracker_reset_reason_to_str(uint32_t esp_reset_reason)
{
    switch (esp_reset_reason)
    {
        case 0:  return "unknown";
        case 1:  return "poweron";
        case 2:  return "external";
        case 3:  return "software";
        case 4:  return "panic";
        case 5:  return "deepsleep";  /* ESP_RST_DEEPSLEEP */
        case 6:  return "brownout";   /* ESP_RST_BROWNOUT  */
        case 7:  return "interrupt_wdt";
        case 8:  return "task_wdt";
        case 9:  return "wdt";
        case 10: return "sdio";
        default: return "invalid";
    }
}

const char *restart_tracker_planned_reason_to_str(restart_tracker_planned_reason_t reason)
{
    switch (reason)
    {
        case RESTART_TRACKER_PLANNED_REASON_NONE:              return "none";
        case RESTART_TRACKER_PLANNED_REASON_USER_REQUEST:      return "user_request";
        case RESTART_TRACKER_PLANNED_REASON_CONFIG_APPLY:      return "config_apply";
        case RESTART_TRACKER_PLANNED_REASON_CONFIG_RECOVERY:   return "config_recovery";
        case RESTART_TRACKER_PLANNED_REASON_OTA_APPLY:         return "ota_apply";
        case RESTART_TRACKER_PLANNED_REASON_FACTORY_RESET:     return "factory_reset";
        case RESTART_TRACKER_PLANNED_REASON_SAFE_MODE:         return "safe_mode";
        case RESTART_TRACKER_PLANNED_REASON_POWER_WAKE:        return "power_wake";
        case RESTART_TRACKER_PLANNED_REASON_INTERNAL_RECOVERY: return "internal_recovery";
        default:                                               return "invalid";
    }
}

const char *restart_tracker_source_to_str(restart_tracker_source_t source)
{
    switch (source)
    {
        case RESTART_TRACKER_SOURCE_UNKNOWN:       return "unknown";
        case RESTART_TRACKER_SOURCE_WEB_UI:        return "web_ui";
        case RESTART_TRACKER_SOURCE_CMDLINE:       return "cmdline";
        case RESTART_TRACKER_SOURCE_CONSOLE:       return "console";
        case RESTART_TRACKER_SOURCE_MQTT:          return "mqtt";
        case RESTART_TRACKER_SOURCE_OTA:           return "ota";
        case RESTART_TRACKER_SOURCE_SAFE_MODE:     return "safe_mode";
        case RESTART_TRACKER_SOURCE_CONFIG_SERVER: return "config_server";
        case RESTART_TRACKER_SOURCE_SLEEP_MODE:    return "sleep_mode";
        default:                                   return "invalid";
    }
}
