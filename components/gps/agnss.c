/*
 * Copyright (C) 2022  Meatpi Electronics.
 * Written by Ali Slim <ali@meatpi.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

/**
 * @file agnss.c
 * @brief AGNSS assistance for Quectel LC76G GPS module
 *
 * Injects reference time (from BG95 modem) and cached position (from NVS)
 * into the LC76G via PAIR proprietary commands to accelerate time-to-first-fix.
 * Also manages the built-in EASY prediction feature.
 */

#include "gps.h"
#include "agnss_internal.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gps.h"
#include "config_manager.h"
#include "lte_upstream_pppos.h"

#include <time.h>
#include <sys/time.h>

static const char *TAG = "agnss";

static volatile bool s_time_injected = false;
static volatile bool s_position_injected = false;

void agnss_get_status(bool *time_injected, bool *position_injected)
{
    if (time_injected)
    {
        *time_injected = s_time_injected;
    }
    if (position_injected)
    {
        *position_injected = s_position_injected;
    }
}

/* -----------------------------------------------------------------------
 * PAIR command helpers — shared via agnss_internal.h
 * ----------------------------------------------------------------------- */

uint8_t agnss_nmea_checksum(const char *sentence)
{
    uint8_t cs = 0;
    /* Skip leading '$' if present */
    if (*sentence == '$')
    {
        sentence++;
    }
    while (*sentence && *sentence != '*')
    {
        cs ^= (uint8_t)*sentence++;
    }
    return cs;
}

int agnss_build_pair_cmd(char *buf, size_t buf_len, const char *body)
{
    int n = snprintf(buf, buf_len, "$%s*", body);
    if (n < 0 || (size_t)n >= buf_len - 5)
    {
        return -1;
    }
    uint8_t cs = agnss_nmea_checksum(buf);
    n = snprintf(buf, buf_len, "$%s*%02X\r\n", body, cs);
    return n;
}

/**
 * @brief Send a PAIR command to the LC76G and wait for ACK.
 *
 * The ACK format is: $PAIR001,<cmd_id>,<result>*XX\r\n
 * where result 0 = success.
 *
 * @param cmd_id   Numeric PAIR command ID (e.g. 590, 600)
 * @param cmd_str  Full command string (already checksummed, with \r\n)
 * @param timeout_ms  Maximum time to wait for ACK
 * @return ESP_OK if ACK with result 0 received, error otherwise.
 */
static esp_err_t pair_send_and_wait_ack(int cmd_id, const char *cmd_str,
                                        uint32_t timeout_ms)
{
    esp_err_t err = gps_uart_write(cmd_str, strlen(cmd_str));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "PAIR%d: uart_write failed", cmd_id);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "PAIR%d sent, waiting for ACK...", cmd_id);

    char ack_prefix[32];
    snprintf(ack_prefix, sizeof(ack_prefix), "$PAIR001,%d,", cmd_id);
    size_t prefix_len = strlen(ack_prefix);

    int64_t deadline = esp_timer_get_time() / 1000 + timeout_ms;

    while ((esp_timer_get_time() / 1000) < deadline)
    {
        uint32_t remaining = (uint32_t)(deadline - esp_timer_get_time() / 1000);
        if (remaining == 0)
        {
            break;
        }

        char line[160];
        if (gps_pair_response_wait(line, sizeof(line), remaining) != ESP_OK)
        {
            break;
        }

        if (strncmp(line, ack_prefix, prefix_len) == 0)
        {
            int result = atoi(line + prefix_len);
            if (result == 0)
            {
                ESP_LOGI(TAG, "PAIR%d ACK OK", cmd_id);
                return ESP_OK;
            }
            else
            {
                ESP_LOGW(TAG, "PAIR%d ACK error: %d", cmd_id, result);
                return ESP_FAIL;
            }
        }
    }

    ESP_LOGW(TAG, "PAIR%d ACK timeout", cmd_id);
    return ESP_ERR_TIMEOUT;
}

/* -----------------------------------------------------------------------
 * Reference time injection (system clock → $PAIR590)
 * ----------------------------------------------------------------------- */

static esp_err_t inject_reference_time(void)
{
    /* Read UTC from the system clock (set by lte_upstream_pppos via AT+QLTS) */
    if (!lte_upstream_pppos_is_time_synced())
    {
        return ESP_ERR_INVALID_STATE;
    }

    time_t now;
    time(&now);
    struct tm utc;
    gmtime_r(&now, &utc);

    int year  = utc.tm_year + 1900;
    int month = utc.tm_mon + 1;
    int day   = utc.tm_mday;
    int hour  = utc.tm_hour;
    int min   = utc.tm_min;
    int sec   = utc.tm_sec;

    if (year < 2024)
    {
        ESP_LOGW(TAG, "System clock looks invalid (year=%d)", year);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Injecting system UTC: %04d-%02d-%02d %02d:%02d:%02d",
             year, month, day, hour, min, sec);

    char body[64];
    snprintf(body, sizeof(body), "PAIR590,%d,%d,%d,%d,%d,%d",
             year, month, day, hour, min, sec);

    char cmd[80];
    agnss_build_pair_cmd(cmd, sizeof(cmd), body);

    return pair_send_and_wait_ack(590, cmd, 2000);
}

/* -----------------------------------------------------------------------
 * Reference position injection ($PAIR600)
 * ----------------------------------------------------------------------- */

static esp_err_t inject_reference_position(void)
{
    double lat, lon, alt;
    esp_err_t err = agnss_nvs_read_position(&lat, &lon, &alt);
    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "No cached position in NVS – skipping PAIR600");
        return ESP_ERR_NOT_FOUND;
    }

    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0)
    {
        ESP_LOGW(TAG, "Cached position out of range: lat=%.6f lon=%.6f", lat, lon);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Injecting cached position: lat=%.6f lon=%.6f alt=%.1f",
             lat, lon, alt);

    char body[128];
    snprintf(body, sizeof(body),
             "PAIR600,%.6f,%.6f,%.1f,50.0,50.0,0.0,100.0",
             lat, lon, alt);

    char cmd[160];
    agnss_build_pair_cmd(cmd, sizeof(cmd), body);

    return pair_send_and_wait_ack(600, cmd, 2000);
}

/* -----------------------------------------------------------------------
 * EASY prediction management
 * ----------------------------------------------------------------------- */

static esp_err_t verify_easy_enabled(void)
{
    /* $PAIR491 — query EASY status */
    char cmd[32];
    agnss_build_pair_cmd(cmd, sizeof(cmd), "PAIR491");

    esp_err_t err = gps_uart_write(cmd, strlen(cmd));
    if (err != ESP_OK)
    {
        return err;
    }

    ESP_LOGI(TAG, "Querying EASY status (PAIR491)...");

    /* Response: $PAIR491,<enabled>,<prediction_valid>*XX */
    int64_t deadline = esp_timer_get_time() / 1000 + 3000;

    while ((esp_timer_get_time() / 1000) < deadline)
    {
        uint32_t remaining = (uint32_t)(deadline - esp_timer_get_time() / 1000);
        if (remaining == 0)
        {
            break;
        }

        char line[160];
        if (gps_pair_response_wait(line, sizeof(line), remaining) != ESP_OK)
        {
            break;
        }

        if (strncmp(line, "$PAIR491,", 9) == 0)
        {
            int enabled = -1, pred_valid = -1;
            sscanf(line + 9, "%d,%d", &enabled, &pred_valid);
            ESP_LOGI(TAG, "EASY: enabled=%d prediction_valid=%d",
                     enabled, pred_valid);
            if (enabled != 1)
            {
                ESP_LOGW(TAG, "EASY not enabled – enabling now");
                char ecmd[32];
                agnss_build_pair_cmd(ecmd, sizeof(ecmd), "PAIR490,1");
                pair_send_and_wait_ack(490, ecmd, 2000);
            }
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "EASY status query timed out");
    return ESP_ERR_TIMEOUT;
}

/* -----------------------------------------------------------------------
 * EASY save
 * ----------------------------------------------------------------------- */

esp_err_t agnss_save_easy_data(void)
{
    ESP_LOGI(TAG, "Saving EASY navigation data...");

    /* For simplicity, handle >1 Hz case: set 1 Hz, restart, save, hot-start */
    /* We'll always do the safe path: set 1 Hz → ack → save → ack */

    /* Set fix rate to 1 Hz: $PAIR382,1000 */
    char cmd[48];
    agnss_build_pair_cmd(cmd, sizeof(cmd), "PAIR382,1000");
    esp_err_t err = pair_send_and_wait_ack(382, cmd, 2000);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "PAIR382 (set 1Hz) failed, trying save anyway");
    }

    /* Cold start to apply fix rate change: $PAIR003 */
    agnss_build_pair_cmd(cmd, sizeof(cmd), "PAIR003");
    err = pair_send_and_wait_ack(3, cmd, 2000);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "PAIR003 (cold start) failed, trying save anyway");
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    /* Save EASY data: $PAIR511 */
    agnss_build_pair_cmd(cmd, sizeof(cmd), "PAIR511");
    err = pair_send_and_wait_ack(511, cmd, 5000);

    /* Hot start to resume: $PAIR002 */
    char hcmd[32];
    agnss_build_pair_cmd(hcmd, sizeof(hcmd), "PAIR002");
    pair_send_and_wait_ack(2, hcmd, 2000);

    return err;
}

/* -----------------------------------------------------------------------
 * Manual time injection trigger (called from CLI / external)
 * ----------------------------------------------------------------------- */

esp_err_t agnss_try_inject_time(void)
{
    if (s_time_injected)
    {
        return ESP_OK;
    }
    esp_err_t err = inject_reference_time();
    if (err == ESP_OK)
    {
        s_time_injected = true;
        ESP_LOGI(TAG, "Reference time injected (manual trigger)");
    }
    return err;
}

/* -----------------------------------------------------------------------
 * Position cache task
 * ----------------------------------------------------------------------- */

#define CACHE_INTERVAL_DEFAULT_S  120            /* 2 minutes */
#define CACHE_DISTANCE_M         500.0          /* minimum movement in metres */

static volatile bool s_cache_task_running = false;

/* Haversine distance in metres (approximate) */
static double haversine_m(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6371000.0;
    double dlat = (lat2 - lat1) * (M_PI / 180.0);
    double dlon = (lon2 - lon1) * (M_PI / 180.0);
    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(lat1 * (M_PI / 180.0)) * cos(lat2 * (M_PI / 180.0)) *
                   sin(dlon / 2) * sin(dlon / 2);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return R * c;
}

static void cache_position_task(void *arg)
{
    (void)arg;

    double prev_lat = 0, prev_lon = 0;
    bool have_prev = false;
    int64_t last_save_ms = 0;

    int cache_s = CACHE_INTERVAL_DEFAULT_S;
    config_get_int("AGNSS_CACHE_S", &cache_s);
    
    while (s_cache_task_running)
    {
        /* Poll quickly (5 s) until first fix is cached, then slow to 30 s */
        vTaskDelay(pdMS_TO_TICKS(have_prev ? 30000 : 5000));

        gps_fix_snapshot_t snap;
        if (gps_get_fix(&snap) != ESP_OK || !snap.has_fix || !snap.valid)
        {
            continue;
        }

        int64_t cache_interval_ms = (int64_t)cache_s * 1000;

        int64_t now = esp_timer_get_time() / 1000;
        bool time_ok = (now - last_save_ms) >= cache_interval_ms;
        bool dist_ok = false;
        if (have_prev)
        {
            double d = haversine_m(prev_lat, prev_lon, snap.lat, snap.lon);
            dist_ok = d >= CACHE_DISTANCE_M;
        }

        if (!have_prev || time_ok || dist_ok)
        {
            esp_err_t err = agnss_nvs_write_position(snap.lat, snap.lon,
                                                     snap.altitude_m);
            if (err == ESP_OK)
            {
                ESP_LOGI(TAG, "Cached position: lat=%.6f lon=%.6f alt=%.1f",
                         snap.lat, snap.lon, snap.altitude_m);
                prev_lat = snap.lat;
                prev_lon = snap.lon;
                have_prev = true;
                last_save_ms = now;
            }
        }
    }

    vTaskDelete(NULL);
}

/* -----------------------------------------------------------------------
 * Background init task — gives the LC76G time to boot after power-on
 * ----------------------------------------------------------------------- */

static void agnss_init_task(void *arg)
{
    (void)arg;

    /* LC76G needs ~3 s after power-on before it responds to PAIR commands */
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* 1. Verify EASY is enabled on the LC76G */
    verify_easy_enabled();

    /* 2. Inject reference position from NVS (best-effort) */
    if (inject_reference_position() == ESP_OK)
    {
        s_position_injected = true;
    }

    /* 3. Wait for system clock to be synced, then inject immediately */
    for (int i = 0; i < 150; i++)   /* up to ~5 min (150 × 2 s) */
    {
        if (inject_reference_time() == ESP_OK)
        {
            s_time_injected = true;
            ESP_LOGI(TAG, "Reference time injected");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    vTaskDelete(NULL);
}

/* -----------------------------------------------------------------------
 * Public init
 * ----------------------------------------------------------------------- */

esp_err_t agnss_init(bool enabled)
{
    if (!enabled)
    {
        ESP_LOGI(TAG, "AGNSS disabled");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing AGNSS assistance (deferred to background)...");

    /* Start background init task */
    xTaskCreate(agnss_init_task, "agnss_init", 4096, NULL, 3, NULL);

    /* Start position caching task */
    s_cache_task_running = true;
    xTaskCreate(cache_position_task, "agnss_cache", 4096, NULL, 2, NULL);

    return ESP_OK;
}
