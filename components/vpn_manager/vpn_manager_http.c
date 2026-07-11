/**
 * @file vpn_manager_http.c
 * @brief The optional /api/vpn routes (§9.1): status GET + device-side
 *        keygen POST (returns ONLY the public key; the private key
 *        lands in pending settings). Config itself is the plain
 *        /api/settings/vpn_manager surface.
 */
#include <stdio.h>

#include "esp_http_server.h"
#include "esp_log.h"

#include "http_server_manager.h"

#include "vpn_manager.h"

static const char *TAG = "vpn_manager";

static const char *state_str(vpn_state_t st)
{
    switch (st)
    {
        case VPN_STATE_WAITING:    return "waiting";
        case VPN_STATE_CONNECTING: return "connecting";
        case VPN_STATE_CONNECTED:  return "connected";
        default:                   return "disabled";
    }
}

static esp_err_t vpn_get_handler(httpd_req_t *req)
{
    vpn_manager_status_t st;
    char body[288];

    (void)vpn_manager_status(&st);
    snprintf(body, sizeof(body),
             "{\"state\":\"%s\",\"type\":\"%s\",\"endpoint\":\"%s\","
             "\"ts_ip\":\"%s\",\"ts_peers\":%d,\"connects\":%lu,"
             "\"failures\":%lu,\"uptime_s\":%lu}",
             state_str(st.state),
             st.tailscale ? "tailscale" : "wireguard", st.endpoint,
             st.ts_ip, st.ts_peers,
             (unsigned long)st.connects, (unsigned long)st.failures,
             (unsigned long)st.uptime_s);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t vpn_keygen_handler(httpd_req_t *req)
{
    char pub[64];
    char body[128];

    if (vpn_manager_keygen(pub, sizeof(pub)) != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "keygen failed");
        return ESP_FAIL;
    }

    snprintf(body, sizeof(body),
             "{\"public_key\":\"%s\",\"pending_reboot\":true}", pub);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

esp_err_t vpn_manager_register_http(void)
{
    static const httpd_uri_t URIS[] =
    {
        { .uri = "/api/vpn", .method = HTTP_GET,
          .handler = vpn_get_handler },
        { .uri = "/api/vpn/keygen", .method = HTTP_POST,
          .handler = vpn_keygen_handler },
    };

    esp_err_t err = http_server_manager_register_handlers(
        URIS, sizeof(URIS) / sizeof(URIS[0]));

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "/api/vpn registered");
    }

    return err;
}
