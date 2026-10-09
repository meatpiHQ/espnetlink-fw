/**
 * @file wifi_modem_http.c
 * @brief The component's HTTP surface (route map: components/HTTP_API.md):
 *
 *   GET  /api/wifi_modem              health document for the WiCAN poller
 *                                     (uplink + GPS + AP + USB state, no
 *                                     secrets)
 *   GET  /api/wifi_modem/credentials  {ssid, password, device_id} — ONLY for
 *                                     requests that did not arrive over WiFi
 *                                     (the USB link: point-to-point, the
 *                                     WiCAN pairing channel). 403 otherwise.
 *   POST /api/wifi_modem/usb_data     {"enabled":bool} — runtime USB data
 *                                     switch; answers first, applies
 *                                     USB_DATA_APPLY_DELAY_MS later so the
 *                                     response gets out over the very link
 *                                     that is about to be cut.
 */
#include "wifi_modem.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"

#include "cJSON.h"
#include "dev_status_manager.h"
#include "gps.h"
#include "http_server_manager.h"
#include "lte_upstream_pppos.h"
#include "wifi_manager.h"

#include "wifi_modem_private.h"

static const char *TAG = "wifi_modem";

#define USB_DATA_APPLY_DELAY_MS 500
#define BODY_MAX                256

static esp_timer_handle_t s_usb_timer;
static volatile bool s_usb_pending_enable;

/* ---- helpers ------------------------------------------------------------ */

static esp_err_t send_json(httpd_req_t *req, cJSON *o)
{
    char *js = cJSON_PrintUnformatted(o);

    cJSON_Delete(o);
    if (js == NULL)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "serialize");
    }

    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_send(req, js, strlen(js));

    free(js);
    return err;
}

static uint32_t netif_ip_be(const char *ifkey)
{
    esp_netif_t *n = esp_netif_get_handle_from_ifkey(ifkey);
    esp_netif_ip_info_t ip = { 0 };

    if (n == NULL || esp_netif_get_ip_info(n, &ip) != ESP_OK)
    {
        return 0;
    }

    return ip.ip.addr;
}

/* The local (device-side) IPv4 the request arrived on; 0 when unknown. */
static uint32_t request_local_ip_be(httpd_req_t *req)
{
    struct sockaddr_storage local;
    socklen_t len = sizeof(local);

    if (getsockname(httpd_req_to_sockfd(req), (struct sockaddr *)&local,
                    &len) != 0)
    {
        return 0;
    }

    if (local.ss_family == AF_INET)
    {
        return ((const struct sockaddr_in *)&local)->sin_addr.s_addr;
    }

#if LWIP_IPV6
    if (local.ss_family == AF_INET6)
    {
        const struct sockaddr_in6 *a6 = (const struct sockaddr_in6 *)&local;
        const uint8_t *b = a6->sin6_addr.s6_addr;
        static const uint8_t MAPPED[12] =
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0xff };

        if (memcmp(b, MAPPED, sizeof(MAPPED)) == 0)
        {
            uint32_t v4;

            memcpy(&v4, b + 12, sizeof(v4));
            return v4;
        }
    }
#endif

    return 0;
}

/* ---- GET /api/wifi_modem -------------------------------------------------- */

static void add_ap(cJSON *o)
{
    cJSON *ap = cJSON_AddObjectToObject(o, "ap");

    if (ap == NULL)
    {
        return;
    }

    wifi_config_t wc;
    char ip[16] = "";

    if (esp_wifi_get_config(WIFI_IF_AP, &wc) == ESP_OK)
    {
        cJSON_AddStringToObject(ap, "ssid", (const char *)wc.ap.ssid);
    }
    if (wifi_manager_get_ap_ip(ip, sizeof(ip)) == ESP_OK)
    {
        cJSON_AddStringToObject(ap, "ip", ip);
    }
    cJSON_AddBoolToObject(ap, "started", wifi_manager_is_ap_started());
    cJSON_AddNumberToObject(ap, "clients", wifi_manager_get_ap_station_count());
}

static void add_lte(cJSON *o)
{
    cJSON *lte = cJSON_AddObjectToObject(o, "lte");

    if (lte == NULL)
    {
        return;
    }

    lte_status_t st;
    bool valid = (lte_upstream_pppos_get_status(&st) == ESP_OK && st.valid);

    cJSON_AddBoolToObject(lte, "valid", valid);
    /* the SIM and the stage are there before the monitor's first poll: a
     * dongle with no SIM has them and nothing else (2026-10-07, the
     * WiCAN's Quick Setup checks read them over the cable while pairing) */
    cJSON_AddStringToObject(lte, "sim", lte_upstream_pppos_sim_str(
                                            lte_upstream_pppos_sim_state()));
    cJSON_AddStringToObject(lte, "stage", lte_upstream_pppos_stage_str());
    if (!valid)
    {
        return;
    }

    cJSON_AddBoolToObject(lte, "attached", st.attached);
    cJSON_AddBoolToObject(lte, "connected", st.ppp_connected);
    if (st.rssi != 99)
    {
        cJSON_AddNumberToObject(lte, "rssi_dbm", st.rssi_dbm);
    }
    if (st.operator_name[0] != '\0')
    {
        cJSON_AddStringToObject(lte, "operator", st.operator_name);
    }
    if (st.network_type[0] != '\0')
    {
        cJSON_AddStringToObject(lte, "network_type", st.network_type);
    }
    if (st.ppp_connected && st.current_ip.addr != 0)
    {
        char ip[16];

        snprintf(ip, sizeof(ip), "%u.%u.%u.%u",
                 (unsigned)(st.current_ip.addr & 0xff),
                 (unsigned)((st.current_ip.addr >> 8) & 0xff),
                 (unsigned)((st.current_ip.addr >> 16) & 0xff),
                 (unsigned)((st.current_ip.addr >> 24) & 0xff));
        cJSON_AddStringToObject(lte, "ip", ip);
    }
}

static void add_gps(cJSON *o)
{
    cJSON *gps = cJSON_AddObjectToObject(o, "gps");

    if (gps == NULL)
    {
        return;
    }

    gps_fix_snapshot_t snap;
    bool valid = (gps_get_fix(&snap) == ESP_OK && snap.valid);

    cJSON_AddBoolToObject(gps, "valid", valid);
    cJSON_AddBoolToObject(gps, "fix", valid && snap.has_fix);
    if (valid)
    {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        cJSON_AddNumberToObject(gps, "satellites", snap.satellites);
        cJSON_AddNumberToObject(gps, "age_ms", now - snap.last_update_ms);
    }
}

static esp_err_t wifi_modem_get_handler(httpd_req_t *req)
{
    cJSON *o = cJSON_CreateObject();

    if (o == NULL)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "no memory");
    }

    const wifi_modem_settings_t *cfg = wifi_modem_settings_config();

    cJSON_AddStringToObject(o, "usb_mode", wmc_usb_mode_str(cfg->usb_mode));
    cJSON_AddBoolToObject(o, "usb_data", wifi_modem_usb_data_routed());
    cJSON_AddBoolToObject(o, "boot_cut", wifi_modem_boot_cut_hint());
    cJSON_AddBoolToObject(o, "ap_share", cfg->ap_share);
    cJSON_AddBoolToObject(o, "napt", wifi_modem_napt_active());
    add_ap(o);
    add_lte(o);
    add_gps(o);
    cJSON_AddNumberToObject(o, "uptime_s",
                            (double)(esp_timer_get_time() / 1000000));
    cJSON_AddStringToObject(o, "device_id", dev_status_manager_device_id());
    return send_json(req, o);
}

/* ---- GET /api/wifi_modem/credentials -------------------------------------- */

static esp_err_t credentials_get_handler(httpd_req_t *req)
{
    uint32_t local = request_local_ip_be(req);

    if (!wmc_credentials_allowed(local, netif_ip_be("WIFI_AP_DEF"),
                                 netif_ip_be("WIFI_STA_DEF")))
    {
        /* §10: the refusal is a normal event on WiFi — DEBUG, not WARN */
        ESP_LOGD(TAG, "credentials refused: request arrived over WiFi");
        return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN,
                                   "credentials are served over USB only");
    }

    wifi_config_t wc;

    if (esp_wifi_get_config(WIFI_IF_AP, &wc) != ESP_OK)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "AP not configured");
    }

    cJSON *o = cJSON_CreateObject();

    if (o == NULL)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "no memory");
    }

    cJSON_AddStringToObject(o, "ssid", (const char *)wc.ap.ssid);
    cJSON_AddStringToObject(o, "password", (const char *)wc.ap.password);
    cJSON_AddStringToObject(o, "device_id", dev_status_manager_device_id());
    cJSON_AddBoolToObject(o, "ap_started", wifi_manager_is_ap_started());
    ESP_LOGI(TAG, "AP credentials handed to the USB host");
    return send_json(req, o);
}

/* ---- POST /api/wifi_modem/usb_data ---------------------------------------- */

static void usb_apply_cb(void *arg)
{
    (void)arg;

    esp_err_t err = wifi_modem_usb_data_set(s_usb_pending_enable);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "deferred usb_data=%d: %s", (int)s_usb_pending_enable,
                 esp_err_to_name(err));
    }
}

static esp_err_t schedule_usb_apply(bool enable)
{
    if (s_usb_timer == NULL)
    {
        const esp_timer_create_args_t args =
        {
            .callback = usb_apply_cb,
            .name = "wm_usb",
        };
        esp_err_t err = esp_timer_create(&args, &s_usb_timer);

        if (err != ESP_OK)
        {
            return err;
        }
    }

    s_usb_pending_enable = enable;
    (void)esp_timer_stop(s_usb_timer);
    return esp_timer_start_once(s_usb_timer, USB_DATA_APPLY_DELAY_MS * 1000ULL);
}

static esp_err_t usb_data_post_handler(httpd_req_t *req)
{
    char body[BODY_MAX + 1];

    if (req->content_len > BODY_MAX)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "body too large");
    }

    int n = httpd_req_recv(req, body, req->content_len);

    if (n < 0)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "body");
    }
    body[n] = '\0';

    cJSON *in = cJSON_Parse(body);
    const cJSON *en = in ? cJSON_GetObjectItemCaseSensitive(in, "enabled") : NULL;

    if (!cJSON_IsBool(en))
    {
        cJSON_Delete(in);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "expected {\"enabled\":bool}");
    }

    bool enable = cJSON_IsTrue(en);

    cJSON_Delete(in);

    const wifi_modem_settings_t *cfg = wifi_modem_settings_config();

    if (!wifi_modem_settings_is_configured())
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "wifi_modem unconfigured");
    }
    if (!wmc_usb_data_switchable(cfg->usb_mode))
    {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"error\":\"usb_mode=ncm locks the USB "
                                       "data lines on\"}");
    }

    bool already = (wifi_modem_usb_data_routed() == enable);
    esp_err_t err = already ? ESP_OK : schedule_usb_apply(enable);

    if (err != ESP_OK)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   esp_err_to_name(err));
    }

    /* the request IS the owner's intent: arm/clear the boot-cut hint here
     * (httpd task = internal stack; the deferred apply runs on the
     * esp_timer task, which must not touch the settings store) */
    wifi_modem_hint_note(enable);

    cJSON *o = cJSON_CreateObject();

    if (o == NULL)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "no memory");
    }

    cJSON_AddBoolToObject(o, "usb_data", enable);
    cJSON_AddNumberToObject(o, "applies_in_ms",
                            already ? 0 : USB_DATA_APPLY_DELAY_MS);
    ESP_LOGI(TAG, "host requested usb_data=%d (%s)", (int)enable,
             already ? "already" : "applying");
    return send_json(req, o);
}

esp_err_t wifi_modem_register_http(void)
{
    static const httpd_uri_t URIS[] =
    {
        { .uri = "/api/wifi_modem", .method = HTTP_GET,
          .handler = wifi_modem_get_handler },
        { .uri = "/api/wifi_modem/credentials", .method = HTTP_GET,
          .handler = credentials_get_handler },
        { .uri = "/api/wifi_modem/usb_data", .method = HTTP_POST,
          .handler = usb_data_post_handler },
    };

    return http_server_manager_register_handlers(
        URIS, sizeof(URIS) / sizeof(URIS[0]));
}
