/**
 * @file imu_manager_http.c
 * @brief The optional /api/imu route (§9.1: feature components register
 *        their own domain routes) — main calls imu_manager_register_http()
 *        only in HTTP compositions, so the chip code carries no HTTP
 *        dependency. Documented in components/HTTP_API.md §6e.
 */
#include <stdio.h>

#include "esp_http_server.h"
#include "esp_log.h"

#include "http_server_manager.h"

#include "imu_manager.h"

static const char *TAG = "imu_manager";

static esp_err_t imu_handler(httpd_req_t *req)
{
    imu_manager_activity_t act = imu_manager_activity();
    float ax, ay, az, temp;
    char accel[80] = "null";
    char temp_s[16] = "null";
    char body[160];

    if (imu_manager_read_accel(&ax, &ay, &az) == ESP_OK)
    {
        snprintf(accel, sizeof(accel),
                 "{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}", ax, ay, az);
    }

    if (imu_manager_read_temp(&temp) == ESP_OK)
    {
        snprintf(temp_s, sizeof(temp_s), "%.1f", temp);
    }

    snprintf(body, sizeof(body),
             "{\"activity\":\"%s\",\"accel\":%s,\"temp\":%s}",
             (act == IMU_MANAGER_ACTIVE) ? "active" :
             (act == IMU_MANAGER_STATIONARY) ? "stationary" : "unknown",
             accel, temp_s);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

esp_err_t imu_manager_register_http(void)
{
    static const httpd_uri_t URIS[] =
    {
        { .uri = "/api/imu", .method = HTTP_GET, .handler = imu_handler },
    };

    esp_err_t err = http_server_manager_register_handlers(
        URIS, sizeof(URIS) / sizeof(URIS[0]));

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "/api/imu registered");
    }

    return err;
}
