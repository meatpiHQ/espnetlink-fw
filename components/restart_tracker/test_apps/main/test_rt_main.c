/**
 * @file test_rt_main.c
 * @brief On-target test app for restart_tracker: proves the PSRAM `.noinit`
 *        state survives a real esp_restart() and that the planned-restart
 *        intent is consumed by the next boot. Self-driving across two boots:
 *        boot 1 marks a planned restart and reboots; boot 2 (detected via the
 *        surviving state) verifies and prints TEST DONE.
 */
#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "restart_tracker.h"

void app_main(void)
{
    ESP_ERROR_CHECK(restart_tracker_init());
    ESP_ERROR_CHECK(restart_tracker_start());

    restart_tracker_state_t st;
    restart_tracker_record_t rec;

    ESP_ERROR_CHECK(restart_tracker_get_state(&st));
    ESP_ERROR_CHECK(restart_tracker_get_latest_record(&rec));

    printf("BOOT count=%lu seq=%lu reason=%s planned=%d\n",
           (unsigned long)st.boot_count, (unsigned long)rec.sequence,
           restart_tracker_reset_reason_to_str(rec.actual_reset_reason),
           rec.was_planned);

    if (!rec.was_planned)
    {
        /* phase 1: flashed boot (poweron/external). Announce + reboot. */
        printf("PHASE1 marking planned restart and rebooting\n");
        vTaskDelay(pdMS_TO_TICKS(300)); /* let the line flush */
        restart_tracker_restart(RESTART_TRACKER_PLANNED_REASON_USER_REQUEST,
                                RESTART_TRACKER_SOURCE_CONSOLE, 0xC0FFEE);
    }

    /* phase 2: we came back from the planned restart with history intact */
    printf("PHASE2 planned=%d reason=%s source=%s flags=0x%lX\n",
           rec.was_planned,
           restart_tracker_planned_reason_to_str(
               (restart_tracker_planned_reason_t)rec.planned_reason),
           restart_tracker_source_to_str((restart_tracker_source_t)rec.source),
           (unsigned long)rec.flags);
    printf("SURVIVED boots=%lu unexpected=%lu history_kept=%d\n",
           (unsigned long)st.boot_count,
           (unsigned long)st.unexpected_reset_count,
           st.boot_count >= 2);

    /* reset reason of a planned esp_restart() must be "software", and it
       must NOT count as unexpected */
    printf("CLASSIFY reset=%s unexpected_count=%lu\n",
           restart_tracker_reset_reason_to_str(rec.actual_reset_reason),
           (unsigned long)st.unexpected_reset_count);

    printf("TEST DONE\n");
}
