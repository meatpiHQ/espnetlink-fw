/**
 * @file restart_tracker.c
 * @brief Target glue over the pure core: PSRAM `.noinit` state placement,
 *        spinlock, cache write-back (so the data actually reaches PSRAM
 *        before a reset), and the esp_* input sources. Settings live in
 *        restart_tracker_settings.c (standard §4.1).
 */
#include "restart_tracker.h"

#include <string.h>
#include <time.h>

#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include "log_manager.h"

#include "restart_tracker_private.h"

static const char *TAG = "restart_tracker";

/* Survives warm resets; random after power-on — rt_state_is_valid decides. */
static EXT_RAM_NOINIT_ATTR restart_tracker_state_t s_state;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED; /* internal: spinlock */
static bool s_recorded;

static rt_inputs_t gather_inputs(void)
{
    rt_inputs_t in =
    {
        .now_unix = (int64_t)time(NULL),
        .uptime_ms = (uint64_t)(esp_timer_get_time() / 1000LL),
        .reset_reason = (uint32_t)esp_reset_reason(),
    };

    return in;
}

/** Push the cached view out to PSRAM so it survives the next reset. */
static esp_err_t commit(void)
{
    return esp_cache_msync(&s_state, sizeof(s_state),
                           ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                           ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}

esp_err_t restart_tracker_init(void)
{
    if (s_recorded)
    {
        return ESP_OK; /* one record per boot */
    }

    static const log_descriptor_t LOG_DESC =
        { "restart_tracker", ESP_LOG_INFO };

    log_manager_register(&LOG_DESC); /* per-TAG level control (§9.2) */

    rt_inputs_t in = gather_inputs();

    /* forensics for an invalid previous state: distinguishes "memory wiped"
       (magic gone) from "write-back never landed" (CRC-only mismatch) */
    if (!rt_state_is_valid(&s_state))
    {
        ESP_LOGW(TAG, "prev state invalid: magic=%08lx ver=%lu len=%lu "
                      "crc=%08lx want=%08lx",
                 (unsigned long)s_state.magic, (unsigned long)s_state.version,
                 (unsigned long)s_state.history_len,
                 (unsigned long)s_state.crc32,
                 (unsigned long)rt_crc32(&s_state));
    }

    portENTER_CRITICAL(&s_lock);
    bool fresh = rt_record_boot(&s_state, &in);
    esp_err_t err = commit();
    restart_tracker_record_t rec =
        s_state.history[s_state.latest_history_index];
    uint32_t boots = s_state.boot_count;
    uint32_t unexpected = s_state.unexpected_reset_count;
    portEXIT_CRITICAL(&s_lock);

    s_recorded = true;

    ESP_LOGI(TAG, "boot #%lu (%s%s): reset=%s planned=%s/%s, unexpected=%lu",
             (unsigned long)boots,
             fresh ? "fresh state" : "history kept",
             rec.time_valid ? ", time ok" : "",
             restart_tracker_reset_reason_to_str(rec.actual_reset_reason),
             restart_tracker_planned_reason_to_str(
                 (restart_tracker_planned_reason_t)rec.planned_reason),
             restart_tracker_source_to_str(
                 (restart_tracker_source_t)rec.source),
             (unsigned long)unexpected);

    return err;
}

esp_err_t restart_tracker_start(void)
{
    return s_recorded ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t restart_tracker_stop(void)
{
    return ESP_OK; /* passive: nothing to stop */
}

esp_err_t restart_tracker_mark_planned_restart(restart_tracker_planned_reason_t reason,
                                               restart_tracker_source_t source,
                                               uint32_t flags)
{
    rt_inputs_t in = gather_inputs();

    portENTER_CRITICAL(&s_lock);
    rt_mark_planned(&s_state, &in, reason, source, flags);
    esp_err_t err = commit();
    portEXIT_CRITICAL(&s_lock);

    return err;
}

void restart_tracker_restart(restart_tracker_planned_reason_t reason,
                             restart_tracker_source_t source, uint32_t flags)
{
    restart_tracker_mark_planned_restart(reason, source, flags);
    ESP_LOGI(TAG, "restarting: %s/%s",
             restart_tracker_planned_reason_to_str(reason),
             restart_tracker_source_to_str(source));
    esp_restart();
}

esp_err_t restart_tracker_get_state(restart_tracker_state_t *out_state)
{
    if (out_state == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_lock);

    if (!rt_state_is_valid(&s_state))
    {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }

    *out_state = s_state;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t restart_tracker_get_latest_record(restart_tracker_record_t *out_record)
{
    if (out_record == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_lock);

    if (!rt_state_is_valid(&s_state) || s_state.boot_count == 0U)
    {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_NOT_FOUND;
    }

    *out_record = s_state.history[s_state.latest_history_index %
                                  RESTART_TRACKER_HISTORY_LEN];
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}
