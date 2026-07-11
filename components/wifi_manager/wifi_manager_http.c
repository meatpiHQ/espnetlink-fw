/**
 * @file wifi_manager_http.c
 * @brief The component's own HTTP routes (HTTP_API.md §7): /api/wifi/status
 *        and /api/wifi/scan. Feature components register their routes
 *        themselves — main calls wifi_manager_register_http() only in
 *        compositions that include http_server_manager, keeping the radio
 *        code free of any HTTP dependency.
 *
 * Config stays strictly on /api/settings/wifi_manager (api_http glue);
 * there are deliberately no bespoke config routes here.
 */
#include <stdlib.h>

#include "esp_http_server.h"
#include "esp_log.h"

#include "cJSON.h"
#include "http_server_manager.h"

#include "wifi_manager.h"

static const char *TAG = "wifi_manager";

static esp_err_t send_json(httpd_req_t *req, cJSON *obj)
{
    char *body = cJSON_PrintUnformatted(obj);

    cJSON_Delete(obj);

    if (body == NULL)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "oom");
    }

    httpd_resp_set_type(req, "application/json");

    esp_err_t err = httpd_resp_sendstr(req, body);

    free(body);
    return err;
}

static esp_err_t wifi_status_handler(httpd_req_t *req)
{
    cJSON *resp = cJSON_CreateObject();
    char ip[16] = "";
    char dns_main[16] = "";
    char dns_backup[16] = "";

    wifi_manager_get_sta_ip(ip, sizeof(ip));
    wifi_manager_get_sta_dns(dns_main, sizeof(dns_main),
                             dns_backup, sizeof(dns_backup));

    cJSON_AddBoolToObject(resp, "enabled", wifi_manager_is_enabled());
    cJSON_AddBoolToObject(resp, "sta_connected",
                          wifi_manager_is_sta_connected());
    cJSON_AddStringToObject(resp, "ip", ip);
    cJSON_AddBoolToObject(resp, "ap_started", wifi_manager_is_ap_started());
    cJSON_AddNumberToObject(resp, "clients",
                            wifi_manager_get_ap_station_count());

    char ap_ip[16] = "";

    if (wifi_manager_get_ap_ip(ap_ip, sizeof(ap_ip)) == ESP_OK)
    {
        cJSON_AddStringToObject(resp, "ap_ip", ap_ip);
    }

    cJSON *dns = cJSON_AddArrayToObject(resp, "dns");

    cJSON_AddItemToArray(dns, cJSON_CreateString(dns_main));
    cJSON_AddItemToArray(dns, cJSON_CreateString(dns_backup));
    return send_json(req, resp);
}

static esp_err_t wifi_scan_handler(httpd_req_t *req)
{
    /* blocking ≈2 s (the UI shows a spinner) — the JSON goes out verbatim */
    char *json = wifi_manager_scan_networks();

    if (json == NULL)
    {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"error\":\"scan unavailable\"}");
    }

    httpd_resp_set_type(req, "application/json");

    esp_err_t err = httpd_resp_sendstr(req, json);

    free(json);
    return err;
}

esp_err_t wifi_manager_register_http(void)
{
    static const httpd_uri_t URIS[] =
    {
        { .uri = "/api/wifi/status", .method = HTTP_GET,
          .handler = wifi_status_handler },
        { .uri = "/api/wifi/scan", .method = HTTP_GET,
          .handler = wifi_scan_handler },
    };

    esp_err_t err = http_server_manager_register_handlers(
        URIS, sizeof(URIS) / sizeof(URIS[0]));

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "/api/wifi routes registered");
    }

    return err;
}
