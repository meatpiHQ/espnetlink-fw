/**
 * @file gps_http.c
 * @brief `GET /api/gps` — the current fix snapshot for the web UI's status
 *        page (feature-component route, standard §9.1). Read-only; the
 *        JSON mirrors gps_fix_snapshot_t (the `gps` CLI's data source).
 */
#include <string.h>

#include "esp_http_server.h"
#include "esp_timer.h"

#include "cJSON.h"
#include "http_server_manager.h"

#include "gps.h"

static esp_err_t gps_get_handler(httpd_req_t *req)
{
    cJSON *o = cJSON_CreateObject();

    if (o == NULL)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "no memory");
    }

    gps_fix_snapshot_t fix;
    bool valid = (gps_get_fix(&fix) == ESP_OK && fix.valid);
    bool have_fix = valid && fix.has_fix;

    cJSON_AddBoolToObject(o, "valid", valid);
    cJSON_AddBoolToObject(o, "fix", have_fix);

    if (valid)
    {
        cJSON_AddNumberToObject(o, "satellites", fix.satellites);
        cJSON_AddNumberToObject(o, "sats_in_view", fix.sats_in_view);
        cJSON_AddNumberToObject(o, "fix_quality", fix.fix_quality);
        cJSON_AddNumberToObject(o, "fix_type", fix.fix_type);

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        cJSON_AddNumberToObject(o, "age_ms", now_ms - fix.last_update_ms);
    }

    if (have_fix)
    {
        cJSON_AddNumberToObject(o, "latitude", fix.lat);
        cJSON_AddNumberToObject(o, "longitude", fix.lon);
        cJSON_AddNumberToObject(o, "altitude_m", fix.altitude_m);
        cJSON_AddNumberToObject(o, "hdop", fix.hdop);
        cJSON_AddNumberToObject(o, "speed_kmph", fix.speed_kmph);
        cJSON_AddNumberToObject(o, "course_deg", fix.course_deg);
    }

    char *js = cJSON_PrintUnformatted(o);

    cJSON_Delete(o);

    if (js == NULL)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "serialize");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, js, strlen(js));
    free(js);
    return ESP_OK;
}

esp_err_t gps_register_http(void)
{
    static const httpd_uri_t URIS[] =
    {
        { .uri = "/api/gps", .method = HTTP_GET, .handler = gps_get_handler },
    };

    return http_server_manager_register_handlers(
        URIS, sizeof(URIS) / sizeof(URIS[0]));
}
