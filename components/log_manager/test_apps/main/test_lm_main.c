/**
 * @file test_lm_main.c
 * @brief On-target test app for log_manager: vprintf capture end-to-end,
 *        custom sink routing, runtime level control, drop-oldest
 *        backpressure, and PSRAM ring persistence across a real
 *        esp_restart() (two-phase, like the restart_tracker app).
 */
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "log_manager.h"
#include "settings_manager.h"

static const char *TAG = "lm_test";

#define PHASE_MARK "RINGPHASE1-UNIQUE-7B3F"

/* capture sink: counts lines and remembers whether a probe string passed */
static volatile int s_sink_lines;
static volatile bool s_sink_saw_probe;

static esp_err_t capture_write(const char *line, size_t len)
{
    (void)len;
    s_sink_lines++;

    if (strstr(line, "PROBE-42") != NULL)
    {
        s_sink_saw_probe = true;
    }

    return ESP_OK;
}

static const log_sink_t CAPTURE_SINK = { "capture", capture_write };

/* Burst ABOVE the log task's priority so it cannot drain concurrently —
 * the queue must overflow and drop-oldest (never block us). */
static volatile bool s_burst_done;

static void burst_task(void *arg)
{
    (void)arg;

    for (int i = 0; i < 300; i++)
    {
        ESP_LOGI(TAG, "burst %d", i);
    }

    s_burst_done = true;
    vTaskDelete(NULL);
}

void app_main(void)
{
    /* compose like main: log pipeline first, then settings */
    ESP_ERROR_CHECK(log_manager_init());
    ESP_ERROR_CHECK(settings_manager_init());
    ESP_ERROR_CHECK(log_manager_register_settings());
    ESP_ERROR_CHECK(settings_manager_start()); /* applies level + sink enables */
    ESP_ERROR_CHECK(log_manager_start());

    static const log_descriptor_t desc = { "lm_test", ESP_LOG_INFO };

    ESP_ERROR_CHECK(log_manager_register(&desc));
    printf("INIT ok=1\n");

    /* ---- phase detection: did the previous boot leave our marker? -------- */
    static char ring[4096];
    size_t n = 0;

    ESP_ERROR_CHECK(log_manager_ring_read(ring, sizeof(ring) - 1, &n));
    ring[n] = '\0';

    if (strstr(ring, PHASE_MARK) == NULL)
    {
        /* ---- phase 1 ------------------------------------------------------ */

        /* custom sink sees what ESP_LOGx emits */
        ESP_ERROR_CHECK(log_manager_add_sink(&CAPTURE_SINK));
        ESP_LOGI(TAG, "PROBE-42 through the pipeline");
        vTaskDelay(pdMS_TO_TICKS(200)); /* let the log task drain */
        printf("SINK saw_probe=%d lines_gt0=%d\n", s_sink_saw_probe,
               s_sink_lines > 0);

        /* runtime level: DEBUG is off at INFO, on after set_level */
        int before = s_sink_lines;

        ESP_LOGD(TAG, "invisible debug line");
        vTaskDelay(pdMS_TO_TICKS(100));

        int mid = s_sink_lines;

        log_manager_set_level("lm_test", ESP_LOG_DEBUG);
        ESP_LOGD(TAG, "visible debug line");
        vTaskDelay(pdMS_TO_TICKS(100));
        printf("LEVEL filtered=%d passed=%d\n", mid == before,
               s_sink_lines > mid);
        log_manager_set_level("lm_test", ESP_LOG_INFO);

        /* sink disable stops routing to it */
        int off_base = s_sink_lines;

        ESP_ERROR_CHECK(log_manager_sink_set_enabled("capture", false));
        ESP_LOGI(TAG, "not for the capture sink");
        vTaskDelay(pdMS_TO_TICKS(100));
        printf("DISABLE unchanged=%d\n", s_sink_lines == off_base);

        /* backpressure: burst >> queue depth must never block, only drop */
        xTaskCreate(burst_task, "burst", 4096, NULL, 5, NULL);

        while (!s_burst_done)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        vTaskDelay(pdMS_TO_TICKS(500));
        printf("BACKPRESSURE dropped_gt0=%d\n",
               log_manager_dropped_count() > 0);

        /* ring already holds everything above; add the phase marker and
           reboot to prove noinit survival */
        ESP_LOGI(TAG, "%s", PHASE_MARK);
        vTaskDelay(pdMS_TO_TICKS(200));
        printf("PHASE1 rebooting\n");
        vTaskDelay(pdMS_TO_TICKS(300));
        esp_restart();
    }

    /* ---- phase 2: the marker logged BEFORE the reset is still here -------- */
    printf("RING survived=1 has_boot_mark=%d\n",
           strstr(ring, "---- boot ----") != NULL);

    ESP_ERROR_CHECK(log_manager_ring_clear());
    ESP_ERROR_CHECK(log_manager_ring_read(ring, sizeof(ring) - 1, &n));
    printf("RINGCLEAR empty=%d\n", n == 0);

    printf("TEST DONE\n");
}
