/**
 * @file lte_upstream_pppos_http.c
 * @brief `GET /api/lte` — the LTE uplink status for the web UI's status
 *        page (feature-component route, standard §9.1). Read-only; the
 *        JSON mirrors lte_status_t (the `lte -s` CLI's data source).
 */
#include <stdio.h>
#include <string.h>

#include "esp_http_server.h"

#include "cJSON.h"
#include "http_server_manager.h"

#include "lte_upstream_pppos.h"

static esp_err_t lte_get_handler(httpd_req_t *req)
{
    cJSON *o = cJSON_CreateObject();

    if (o == NULL)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "no memory");
    }

    lte_status_t st;
    bool valid = (lte_upstream_pppos_get_status(&st) == ESP_OK && st.valid);

    cJSON_AddBoolToObject(o, "valid", valid);
    /* before any attach: the SIM as the bring-up found it and the stage,
     * so a dongle with no SIM says so (2026-10-07, the WiCAN's checks) */
    cJSON_AddStringToObject(o, "sim", lte_upstream_pppos_sim_str(
                                          lte_upstream_pppos_sim_state()));
    cJSON_AddStringToObject(o, "stage", lte_upstream_pppos_stage_str());

    if (valid)
    {
        cJSON_AddBoolToObject(o, "connected", st.ppp_connected);
        cJSON_AddBoolToObject(o, "attached", st.attached);

        if (st.rssi != 99) /* 99 = unknown per 3GPP */
        {
            cJSON_AddNumberToObject(o, "rssi_dbm", st.rssi_dbm);
        }

        if (st.operator_name[0] != '\0')
        {
            cJSON_AddStringToObject(o, "operator", st.operator_name);
        }

        if (st.network_type[0] != '\0')
        {
            cJSON_AddStringToObject(o, "network_type", st.network_type);
        }

        if (st.ppp_connected && st.current_ip.addr != 0)
        {
            char ip[16];

            snprintf(ip, sizeof(ip), "%u.%u.%u.%u",
                     (unsigned)(st.current_ip.addr & 0xff),
                     (unsigned)((st.current_ip.addr >> 8) & 0xff),
                     (unsigned)((st.current_ip.addr >> 16) & 0xff),
                     (unsigned)((st.current_ip.addr >> 24) & 0xff));
            cJSON_AddStringToObject(o, "ip", ip);
        }
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

esp_err_t lte_upstream_pppos_register_http(void)
{
    static const httpd_uri_t URIS[] =
    {
        { .uri = "/api/lte", .method = HTTP_GET, .handler = lte_get_handler },
    };

    return http_server_manager_register_handlers(
        URIS, sizeof(URIS) / sizeof(URIS[0]));
}
