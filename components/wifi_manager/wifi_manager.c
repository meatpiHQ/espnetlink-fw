/**
 * @file wifi_manager.c
 * @brief Lifecycle, event handling, reconnect supervision, status and scan.
 *
 * Configuration is boot-applied via the settings descriptor
 * (wifi_manager_settings.c); STA candidate selection and the auth-failure
 * ban list are pure logic (wifi_manager_select.c). This file owns the
 * radio: netifs, esp_wifi bring-up, the event handler and the reconnect
 * task.
 */
#include "wifi_manager.h"

#include <string.h>

#include "esp_attr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"

#include "dev_status_manager.h"
#include "log_manager.h"

#include "wifi_manager_bridge.h"
#include "wifi_manager_private.h"

static const char *TAG = "wifi_manager";

/* private event-group bits (public ones are BIT0..BIT4 in the header) */
#define WM_BIT_STOP_REQ    BIT6
#define WM_BIT_TASK_EXITED BIT7

#define WM_RECONNECT_PERIOD_MS   5000
#define WM_AP_CLIENT_PAUSE_MS    10000
#define WM_RETRY_COOLDOWN_LOOPS  12 /* x 5 s = 1 minute */
#define WM_SCAN_MAX_RECORDS      32

typedef struct
{
    bool     radio_inited; /* netifs + esp_wifi_init + handlers done once */
    bool     started;
    bool     sta_connected;
    bool     ap_started;
    char     sta_ip[16];
    char     last_attempted_ssid[WM_SSID_LEN];
    int32_t  sta_retry_count;
    uint16_t ap_station_count;
} wm_status_t;

static wm_status_t       s_status EXT_RAM_BSS_ATTR;
static wm_select_state_t s_select EXT_RAM_BSS_ATTR;
static wifi_manager_callbacks_t s_callbacks EXT_RAM_BSS_ATTR;

static esp_netif_t *s_ap_netif;
static esp_netif_t *s_sta_netif;

static EventGroupHandle_t s_events;
static StaticEventGroup_t s_events_buf;      /* internal: FreeRTOS object */
static SemaphoreHandle_t  s_lock;
static StaticSemaphore_t  s_lock_buf;        /* internal: FreeRTOS object */

static TaskHandle_t s_reconnect_task;
static StaticTask_t s_reconnect_tcb;         /* internal: FreeRTOS object */
static StackType_t  s_reconnect_stack[4096] EXT_RAM_BSS_ATTR;

/* scan scratch, serialized by s_lock (reconnect task + API scans) */
static wifi_ap_record_t s_scan_records[WM_SCAN_MAX_RECORDS] EXT_RAM_BSS_ATTR;
static char s_present[WM_SCAN_MAX_RECORDS][WM_SSID_LEN] EXT_RAM_BSS_ATTR;

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

bool wm_core_started(void)
{
    return s_status.started;
}

/* ---- AP config helpers ---------------------------------------------------- */

/** Resolve the effective softAP authmode from the config's ap_auth
 *  selection. AUTO keeps the historic behaviour (WPA2 with a password,
 *  OPEN without); an explicit encrypted mode with no password was already
 *  rejected by the settings validator, but degrade to OPEN defensively. */
static wifi_auth_mode_t ap_authmode(const wm_config_t *cfg)
{
    bool has_pw = cfg->ap.password[0] != '\0';

    switch (cfg->ap_auth)
    {
        case WM_AP_AUTH_OPEN:     return WIFI_AUTH_OPEN;
        case WM_AP_AUTH_WPA2:     return has_pw ? WIFI_AUTH_WPA2_PSK
                                                : WIFI_AUTH_OPEN;
        case WM_AP_AUTH_WPA2WPA3: return has_pw ? WIFI_AUTH_WPA2_WPA3_PSK
                                                : WIFI_AUTH_OPEN;
        case WM_AP_AUTH_WPA3:     return has_pw ? WIFI_AUTH_WPA3_PSK
                                                : WIFI_AUTH_OPEN;
        case WM_AP_AUTH_AUTO:
        default:                  return has_pw ? WIFI_AUTH_WPA2_PSK
                                                : WIFI_AUTH_OPEN;
    }
}

/* ---- STA config helpers --------------------------------------------------- */

/* v6: per-network STA addressing — configure the netif for THIS
   candidate before its connect attempt (each network may live on a
   different LAN, so a fallback carries its own static/DHCP choice).
   esp_netif still raises IP_EVENT_STA_GOT_IP for a static netif when
   the link comes up, so the connect state machine is unchanged. */
static void apply_sta_addressing(const wm_config_t *wcfg, size_t idx)
{
    const wm_addr_t *a = &wcfg->sta_addr[idx];

    if (s_sta_netif == NULL)
    {
        return;
    }

    if (a->is_static)
    {
        esp_netif_ip_info_t ip = { 0 };

        ip.ip.addr = htonl(a->ip);
        ip.netmask.addr = htonl(a->netmask);
        ip.gw.addr = htonl(a->gw);

        esp_netif_dhcpc_stop(s_sta_netif);
        esp_netif_set_ip_info(s_sta_netif, &ip);

        /* static addressing loses the DHCP-provided DNS: the global
           override wins, else this network's gateway (the got-ip
           handler still installs the 1.1.1.1 backup) */
        uint32_t dns_host = wcfg->sta_dns ? wcfg->sta_dns : a->gw;

        if (dns_host != 0)
        {
            esp_netif_dns_info_t dns = { 0 };

            dns.ip.type = ESP_IPADDR_TYPE_V4;
            dns.ip.u_addr.ip4.addr = htonl(dns_host);
            esp_netif_set_dns_info(s_sta_netif, ESP_NETIF_DNS_MAIN, &dns);
        }

        ESP_LOGI(TAG, "STA static IP " IPSTR " for '%s'",
                 IP2STR(&ip.ip), wcfg->sta[idx].ssid);
    }
    else
    {
        /* back to DHCP: drop any stale static address, restart the
           client (ALREADY_STARTED is the common no-op) */
        esp_netif_dhcp_status_t st;

        if (esp_netif_dhcpc_get_status(s_sta_netif, &st) == ESP_OK &&
            st == ESP_NETIF_DHCP_STOPPED)
        {
            esp_netif_ip_info_t zero = { 0 };

            esp_netif_set_ip_info(s_sta_netif, &zero);
        }

        (void)esp_netif_dhcpc_start(s_sta_netif);
    }
}

static esp_err_t apply_sta_network(const wm_config_t *wcfg, size_t idx)
{
    const wm_network_t *net = &wcfg->sta[idx];
    wifi_config_t cfg = { 0 };

    cfg.sta.threshold.authmode =
        (net->password[0] != '\0') ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
    cfg.sta.rm_enabled = 1;
    cfg.sta.btm_enabled = 1;
    cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    cfg.sta.pmf_cfg.capable = true;
    cfg.sta.pmf_cfg.required = false;

    strncpy((char *)cfg.sta.ssid, net->ssid, sizeof(cfg.sta.ssid) - 1);
    strncpy((char *)cfg.sta.password, net->password,
            sizeof(cfg.sta.password) - 1);

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &cfg);

    if (err == ESP_OK)
    {
        apply_sta_addressing(wcfg, idx);
        strncpy(s_status.last_attempted_ssid, net->ssid, WM_SSID_LEN - 1);
        s_status.last_attempted_ssid[WM_SSID_LEN - 1] = '\0';
    }

    return err;
}

/** Blocking scan into s_scan_records (caller holds s_lock). Returns count. */
static uint16_t do_scan(void)
{
    wifi_scan_config_t scan_cfg =
    {
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };

    esp_err_t err = ESP_FAIL;

    for (int attempt = 0; attempt < 3; attempt++)
    {
        err = esp_wifi_scan_start(&scan_cfg, true);

        if (err == ESP_OK)
        {
            break;
        }

        if (err != ESP_ERR_WIFI_STATE)
        {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(200 + attempt * 200));
    }

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "scan failed: %s", esp_err_to_name(err));
        return 0;
    }

    uint16_t count = WM_SCAN_MAX_RECORDS;

    if (esp_wifi_scan_get_ap_records(&count, s_scan_records) != ESP_OK)
    {
        return 0;
    }

    return count;
}

/** Temporarily ensure STA is available for scanning in AP-only mode. */
static bool enter_scan_mode(void)
{
    wifi_mode_t mode;

    if (esp_wifi_get_mode(&mode) == ESP_OK && mode == WIFI_MODE_AP)
    {
        if (esp_wifi_set_mode(WIFI_MODE_APSTA) == ESP_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            return true;
        }
    }

    return false;
}

static void leave_scan_mode(bool switched)
{
    if (switched)
    {
        esp_wifi_set_mode(WIFI_MODE_AP);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/** Pick the best visible candidate (or rotate blindly) and connect. */
/** @return true when a connect attempt was actually issued (the retry
 *  counter must only advance on real attempts, not deferred cycles). */
static bool select_and_connect(void)
{
    const wm_config_t *cfg = wm_settings_config();

    if (cfg->sta_count == 0)
    {
        return false;
    }

    if (cfg->sta_count == 1)
    {
        /* single network: no scan; the sequential path applies the same
         * ban policy incl. the banned-only WM_BANNED_RETRY_MS trickle
         * (a wrong password here may be right back home — same-SSID-
         * different-location, meatpi 2026-07-08) */
        int only = wm_select_sequential(&s_select, cfg->sta,
                                        cfg->sta_count, now_ms());

        if (only < 0)
        {
            ESP_LOGD(TAG, "'%s' banned; deferring connect",
                     cfg->sta[0].ssid);
            return false;
        }

        apply_sta_network(cfg, 0);
        esp_wifi_connect();
        return true;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);

    bool     switched = enter_scan_mode();
    uint16_t found = do_scan();
    int      pick = -1;

    if (found > 0)
    {
        for (uint16_t i = 0; i < found; i++)
        {
            strncpy(s_present[i], (const char *)s_scan_records[i].ssid,
                    WM_SSID_LEN - 1);
            s_present[i][WM_SSID_LEN - 1] = '\0';
        }

        pick = wm_select_from_scan(&s_select, cfg->sta, cfg->sta_count,
                                   s_present, found, now_ms());
    }

    if (pick < 0)
    {
        /* scan failed OR nothing visible matched: rotate blind —
         * hidden SSIDs never appear in scan results (their records
         * carry an empty name) and dense airspace can truncate the
         * record table past a configured network */
        pick = wm_select_sequential(&s_select, cfg->sta, cfg->sta_count,
                                    now_ms());

        if (pick >= 0)
        {
            ESP_LOGD(TAG, "nothing visible matched; blind attempt on "
                          "'%s' (hidden?)", cfg->sta[pick].ssid);
        }
    }

    leave_scan_mode(switched);
    xSemaphoreGive(s_lock);

    if (pick >= 0)
    {
        /* DEBUG: repeats every retry cycle during an outage (§10 hot path) */
        ESP_LOGD(TAG, "connecting to candidate %d: %s", pick,
                 cfg->sta[pick].ssid);
        apply_sta_network(cfg, (size_t)pick);
        esp_wifi_connect();
        return true;
    }

    ESP_LOGD(TAG, "no eligible network this cycle; deferring connect");
    return false;
}

/* ---- event handling --------------------------------------------------------- */

static bool reason_is_auth_related(uint8_t reason)
{
    switch (reason)
    {
        case WIFI_REASON_AUTH_EXPIRE:
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        case WIFI_REASON_INVALID_PMKID:
        case WIFI_REASON_MIC_FAILURE:
        case WIFI_REASON_AUTH_FAIL:
        case WIFI_REASON_HANDSHAKE_TIMEOUT: /* wrong PSK reports this (204) */
            return true;
        default:
            return false;
    }
}

/* roam-to-preferred bookkeeping (reconnect task + got-ip handler) */
static volatile int s_connected_idx = -1; /* config index we're on      */
static uint32_t     s_last_roam_ms;

/* ---- network-trust admission gate (meatpi 2026-07-08) ---------------------
 * Untrusted (shared/office) networks must not expose configuration: when
 * the CURRENT STA network is marked untrusted, inbound admin requests
 * that arrived VIA the STA address are refused. The device's own AP and
 * the USB link classify as non-STA and keep full admin — that is the
 * recovery path. Outbound clients (autopid HTTP posts, MQTT) don't go
 * through httpd and are unaffected. Wired into http_server_manager's
 * request gate by main (composition root). */
bool wifi_manager_http_request_allowed(int sockfd)
{
    int idx = s_connected_idx;
    const wm_config_t *cfg = wm_settings_config();

    if (idx < 0 || (size_t)idx >= cfg->sta_count || cfg->trusted[idx])
    {
        return true;            /* no STA session, or a trusted network */
    }

    if (s_sta_netif == NULL)
    {
        return true;
    }

    esp_netif_ip_info_t ip;

    if (esp_netif_get_ip_info(s_sta_netif, &ip) != ESP_OK ||
        ip.ip.addr == 0)
    {
        return true;            /* no STA address to arrive on          */
    }

    struct sockaddr_storage local;
    socklen_t len = sizeof(local);

    if (getsockname(sockfd, (struct sockaddr *)&local, &len) != 0)
    {
        return false;           /* can't classify on an untrusted net:
                                   fail CLOSED                          */
    }

    if (local.ss_family == AF_INET)
    {
        const struct sockaddr_in *a = (const struct sockaddr_in *)&local;

        return a->sin_addr.s_addr != ip.ip.addr;
    }

#if LWIP_IPV6
    if (local.ss_family == AF_INET6)
    {
        /* lwIP dual-stack accepts v4 clients as v4-mapped v6 */
        const struct sockaddr_in6 *a6 =
            (const struct sockaddr_in6 *)&local;
        const uint8_t *b = (const uint8_t *)&a6->sin6_addr;
        static const uint8_t V4MAP[12] =
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFF, 0xFF };

        if (memcmp(b, V4MAP, 12) == 0)
        {
            uint32_t v4;

            memcpy(&v4, b + 12, 4);
            return v4 != ip.ip.addr;
        }

        return false;           /* native v6 on untrusted STA: closed   */
    }
#endif

    return false;
}

static void on_sta_got_ip(const ip_event_got_ip_t *event)
{
    const wm_config_t *cfg = wm_settings_config();

    s_status.sta_connected = true;
    s_status.sta_retry_count = 0;

    /* which candidate did we land on? (drives roam-to-preferred) */
    s_connected_idx = -1;

    for (size_t i = 0; i < cfg->sta_count; i++)
    {
        if (strcmp(cfg->sta[i].ssid, s_status.last_attempted_ssid) == 0)
        {
            s_connected_idx = (int)i;
            break;
        }
    }

    s_last_roam_ms = now_ms();

    xSemaphoreTake(s_lock, portMAX_DELAY);
    snprintf(s_status.sta_ip, sizeof(s_status.sta_ip), IPSTR,
             IP2STR(&event->ip_info.ip));
    xSemaphoreGive(s_lock);

    ESP_LOGI(TAG, "STA got IP " IPSTR, IP2STR(&event->ip_info.ip));

    wm_select_on_success(&s_select, s_status.last_attempted_ssid);

    xEventGroupSetBits(s_events, WIFI_MANAGER_BIT_STA_CONNECTED);
    xEventGroupClearBits(s_events, WIFI_MANAGER_BIT_STA_DISCONNECTED);
    dev_status_manager_set(DEV_STATUS_BIT_STA_CONNECTED);

    if (s_callbacks.sta_connected != NULL)
    {
        s_callbacks.sta_connected();
    }

    /* v6: custom DNS override in DHCP mode — the lease just overwrote
       MAIN DNS, so re-assert the user's server on every got-ip. (For a
       static network the client is stopped; the attempt set it once.) */
    bool cur_static = s_connected_idx >= 0 &&
                      cfg->sta_addr[s_connected_idx].is_static;

    if (s_sta_netif != NULL && !cur_static && cfg->sta_dns != 0)
    {
        esp_netif_dns_info_t dns = { 0 };

        dns.ip.type = ESP_IPADDR_TYPE_V4;
        dns.ip.u_addr.ip4.addr = htonl(cfg->sta_dns);

        if (esp_netif_set_dns_info(s_sta_netif, ESP_NETIF_DNS_MAIN,
                                   &dns) == ESP_OK)
        {
            ESP_LOGI(TAG, "DNS override: " IPSTR,
                     IP2STR(&dns.ip.u_addr.ip4));
        }
    }

    /* Resilience: if DHCP gave no backup DNS, install a public one. */
    if (s_sta_netif != NULL)
    {
        esp_netif_dns_info_t dns = { 0 };

        if (esp_netif_get_dns_info(s_sta_netif, ESP_NETIF_DNS_BACKUP, &dns)
                == ESP_OK &&
            dns.ip.type == ESP_IPADDR_TYPE_V4 &&
            dns.ip.u_addr.ip4.addr == 0)
        {
            dns.ip.type = ESP_IPADDR_TYPE_V4;
            dns.ip.u_addr.ip4.addr = ESP_IP4TOADDR(1, 1, 1, 1);

            if (esp_netif_set_dns_info(s_sta_netif, ESP_NETIF_DNS_BACKUP,
                                       &dns) == ESP_OK)
            {
                ESP_LOGI(TAG, "backup DNS missing; set 1.1.1.1");
            }
        }
    }

    /* APSTA housekeeping */
    if (cfg->mode == WM_MODE_APSTA && cfg->ap_auto_disable)
    {
        ESP_LOGI(TAG, "STA up; auto-disabling AP");
        esp_wifi_set_mode(WIFI_MODE_STA);
        return;
    }

    if (cfg->mode == WM_MODE_APSTA)
    {
        /* keep AP on the STA channel to avoid the radio hopping */
        wifi_config_t sta_cfg;
        wifi_config_t ap_cfg;

        if (esp_wifi_get_config(WIFI_IF_STA, &sta_cfg) == ESP_OK &&
            esp_wifi_get_config(WIFI_IF_AP, &ap_cfg) == ESP_OK &&
            sta_cfg.sta.channel != 0 &&
            ap_cfg.ap.channel != sta_cfg.sta.channel)
        {
            ESP_LOGI(TAG, "moving AP to STA channel %d", sta_cfg.sta.channel);
            ap_cfg.ap.channel = sta_cfg.sta.channel;
            esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
        }

        /* flag STA/AP subnet overlap so the UI can warn */
        esp_netif_ip_info_t ap_ip;

        if (s_ap_netif != NULL &&
            esp_netif_get_ip_info(s_ap_netif, &ap_ip) == ESP_OK)
        {
            uint32_t sta_ip = event->ip_info.ip.addr;
            uint32_t sta_mask = event->ip_info.netmask.addr;
            bool overlap =
                ((ap_ip.ip.addr & sta_mask) == (sta_ip & sta_mask)) ||
                ((sta_ip & ap_ip.netmask.addr) ==
                 (ap_ip.ip.addr & ap_ip.netmask.addr));

            if (overlap)
            {
                ESP_LOGW(TAG, "STA and AP subnets overlap");
                xEventGroupSetBits(s_events, WIFI_MANAGER_BIT_STA_AP_OVERLAP);
                dev_status_manager_set(DEV_STATUS_BIT_STA_AP_OVERLAP);
            }
            else
            {
                xEventGroupClearBits(s_events,
                                     WIFI_MANAGER_BIT_STA_AP_OVERLAP);
                dev_status_manager_clear(DEV_STATUS_BIT_STA_AP_OVERLAP);
            }
        }
    }
}

static void on_sta_disconnected(const wifi_event_sta_disconnected_t *event)
{
    const wm_config_t *cfg = wm_settings_config();

    s_status.sta_connected = false;
    s_connected_idx = -1;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_status.sta_ip[0] = '\0';
    xSemaphoreGive(s_lock);

    xEventGroupClearBits(s_events, WIFI_MANAGER_BIT_STA_CONNECTED |
                                   WIFI_MANAGER_BIT_STA_AP_OVERLAP);
    xEventGroupSetBits(s_events, WIFI_MANAGER_BIT_STA_DISCONNECTED);
    dev_status_manager_clear(DEV_STATUS_BIT_STA_CONNECTED |
                             DEV_STATUS_BIT_STA_AP_OVERLAP);

    if (event != NULL)
    {
        ESP_LOGD(TAG, "STA disconnected, reason %d (ssid '%s')",
                 event->reason, s_status.last_attempted_ssid);

        if (reason_is_auth_related(event->reason))
        {
            wm_select_on_auth_fail(&s_select, s_status.last_attempted_ssid,
                                   now_ms());
        }
    }

    if (s_callbacks.sta_disconnected != NULL)
    {
        s_callbacks.sta_disconnected();
    }

    /* if AP was auto-disabled while STA was up, bring it back (unless the
       interface policy holds it suspended) */
    if (cfg->mode == WM_MODE_APSTA && cfg->ap_auto_disable &&
        !wm_suspend_ap_active())
    {
        wifi_mode_t mode;

        if (esp_wifi_get_mode(&mode) == ESP_OK && mode == WIFI_MODE_STA)
        {
            ESP_LOGI(TAG, "STA down; re-enabling AP");
            esp_wifi_set_mode(WIFI_MODE_APSTA);
        }
    }
}

static void wm_event_handler(void *arg, esp_event_base_t base,
                             int32_t event_id, void *data)
{
    (void)arg;

    if (base == WIFI_EVENT)
    {
        switch (event_id)
        {
            case WIFI_EVENT_STA_START:
                if (wm_settings_config()->sta_auto_reconnect)
                {
                    select_and_connect();
                }
                break;

            case WIFI_EVENT_STA_DISCONNECTED:
                on_sta_disconnected(data);
                break;

            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "AP started");
                s_status.ap_started = true;
                xEventGroupSetBits(s_events, WIFI_MANAGER_BIT_AP_STARTED);
                dev_status_manager_set(DEV_STATUS_BIT_AP_ENABLED);
                break;

            case WIFI_EVENT_AP_STOP:
                ESP_LOGI(TAG, "AP stopped");
                s_status.ap_started = false;
                s_status.ap_station_count = 0;
                xEventGroupClearBits(s_events,
                                     WIFI_MANAGER_BIT_AP_STARTED |
                                     WIFI_MANAGER_BIT_STA_AP_OVERLAP);
                dev_status_manager_clear(DEV_STATUS_BIT_AP_ENABLED |
                                         DEV_STATUS_BIT_STA_AP_OVERLAP);
                break;

            case WIFI_EVENT_AP_STACONNECTED:
            {
                const wifi_event_ap_staconnected_t *e = data;

                s_status.ap_station_count++;
                ESP_LOGI(TAG, "AP client " MACSTR " joined (%u total)",
                         MAC2STR(e->mac), s_status.ap_station_count);

                if (s_callbacks.ap_station_connected != NULL)
                {
                    s_callbacks.ap_station_connected(e->mac, e->aid);
                }
                break;
            }

            case WIFI_EVENT_AP_STADISCONNECTED:
            {
                const wifi_event_ap_stadisconnected_t *e = data;

                if (s_status.ap_station_count > 0)
                {
                    s_status.ap_station_count--;
                }

                ESP_LOGI(TAG, "AP client " MACSTR " left (%u total)",
                         MAC2STR(e->mac), s_status.ap_station_count);

                if (s_callbacks.ap_station_disconnected != NULL)
                {
                    s_callbacks.ap_station_disconnected(e->mac, e->aid);
                }
                break;
            }

            default:
                break;
        }
    }
    else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        on_sta_got_ip(data);
    }
}

/* ---- reconnect task ----------------------------------------------------------- */

static void reconnect_task(void *arg)
{
    const wm_config_t *cfg = wm_settings_config();
    int cooldown_loops = 0;
    int backoff_loops = 0;

    (void)arg;
    ESP_LOGD(TAG, "reconnect task up");

    while (true)
    {
        EventBits_t bits = xEventGroupWaitBits(
            s_events, WM_BIT_STOP_REQ, pdFALSE, pdFALSE,
            pdMS_TO_TICKS(WM_RECONNECT_PERIOD_MS));

        if ((bits & WM_BIT_STOP_REQ) != 0)
        {
            break;
        }

        if (!s_status.started || wm_suspend_sta_active())
        {
            continue; /* suspended: the interface policy owns STA now */
        }

        if (s_status.sta_connected)
        {
            backoff_loops = 0; /* fresh curve for the next outage */

            /* roam-to-preferred (meatpi: users prefer HOME over the car
             * hotspot): while parked on a FALLBACK, periodically look
             * for a higher-priority network and migrate to it */
            int cur = s_connected_idx;

            if (cfg->sta_roam_interval_s > 0 && cur > 0 &&
                s_status.ap_station_count == 0 &&
                (uint32_t)(now_ms() - s_last_roam_ms) >=
                    cfg->sta_roam_interval_s * 1000u)
            {
                s_last_roam_ms = now_ms();

                xSemaphoreTake(s_lock, portMAX_DELAY);

                uint16_t found = do_scan();
                int      better = -1;

                if (found > 0)
                {
                    for (uint16_t i = 0; i < found; i++)
                    {
                        strncpy(s_present[i],
                                (const char *)s_scan_records[i].ssid,
                                WM_SSID_LEN - 1);
                        s_present[i][WM_SSID_LEN - 1] = '\0';
                    }

                    better = wm_select_better(&s_select, cfg->sta,
                                              cfg->sta_count, cur,
                                              s_present, found, now_ms());
                }

                xSemaphoreGive(s_lock);

                if (better >= 0)
                {
                    ESP_LOGI(TAG, "preferred network '%s' visible; "
                                  "leaving '%s' to roam to it",
                             cfg->sta[better].ssid, cfg->sta[cur].ssid);
                    esp_wifi_disconnect();
                    /* the disconnect event + this loop reconnect via the
                     * normal selection path, which prefers `better` */
                }
            }

            continue;
        }

        if (cooldown_loops > 0)
        {
            cooldown_loops--;
            continue;
        }

        if (backoff_loops > 0)
        {
            backoff_loops--;
            continue;
        }

        /* don't yank the radio's channel while someone is on our AP */
        if (s_status.ap_station_count > 0)
        {
            ESP_LOGD(TAG, "AP has %u clients; pausing STA reconnect",
                     s_status.ap_station_count);
            xEventGroupWaitBits(s_events, WM_BIT_STOP_REQ, pdFALSE, pdFALSE,
                                pdMS_TO_TICKS(WM_AP_CLIENT_PAUSE_MS -
                                              WM_RECONNECT_PERIOD_MS));
            continue;
        }

        if (cfg->sta_max_retry >= 0 &&
            s_status.sta_retry_count >= cfg->sta_max_retry)
        {
            ESP_LOGW(TAG, "retry limit (%ld) hit; cooling down 1 min",
                     (long)cfg->sta_max_retry);
            s_status.sta_retry_count = 0;
            cooldown_loops = WM_RETRY_COOLDOWN_LOOPS;
            continue;
        }

        if (select_and_connect())
        {
            /* count only REAL attempts — deferred cycles (banned/idle)
             * must not burn through a finite sta_max_retry */
            s_status.sta_retry_count++;
            backoff_loops =
                wm_backoff_skip_loops(s_status.sta_retry_count);
            ESP_LOGD(TAG, "reconnect attempt %ld (next in %d s)",
                     (long)s_status.sta_retry_count,
                     (backoff_loops + 1) *
                         (WM_RECONNECT_PERIOD_MS / 1000));
        }
    }

    ESP_LOGD(TAG, "reconnect task exiting");
    xEventGroupSetBits(s_events, WM_BIT_TASK_EXITED);
    s_reconnect_task = NULL;
    vTaskDelete(NULL);
}

/* ---- lifecycle -------------------------------------------------------------------- */

esp_err_t wifi_manager_init(void)
{
    static const log_descriptor_t LOG_DESC = { "wifi_manager", ESP_LOG_INFO };

    log_manager_register(&LOG_DESC); /* per-TAG level control (§9.2) */

    if (s_events == NULL)
    {
        s_events = xEventGroupCreateStatic(&s_events_buf);
        s_lock = xSemaphoreCreateMutexStatic(&s_lock_buf);
    }

    return wm_settings_register();
}

esp_err_t wifi_manager_start(void)
{
    if (!wm_settings_is_configured())
    {
        ESP_LOGE(TAG, "unconfigured (settings boot pass failed); not starting");
        return ESP_ERR_INVALID_STATE; /* Coding Standard §3 / §4.3 step 5 */
    }

    if (s_status.started)
    {
        return ESP_OK;
    }

    const wm_config_t *cfg = wm_settings_config();

    wm_select_init(&s_select); /* first sequential pick = the primary */

    if (cfg->mode == WM_MODE_OFF)
    {
        ESP_LOGI(TAG, "mode off; radio stays down");
        s_status.started = true;
        return ESP_OK;
    }

    esp_err_t err;

    if (!s_status.radio_inited)
    {
        /* tolerate main having created these already */
        err = esp_netif_init();

        if (err != ESP_OK)
        {
            return err;
        }

        err = esp_event_loop_create_default();

        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        {
            return err;
        }

        s_ap_netif = esp_netif_create_default_wifi_ap();
        s_sta_netif = esp_netif_create_default_wifi_sta();

        if (s_ap_netif == NULL || s_sta_netif == NULL)
        {
            return ESP_ERR_NO_MEM;
        }

        wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();

        /* WiFi memory profile (v5): the driver buffer counts are runtime
           fields, so a setting trades internal RAM for throughput with no
           recompile. Only the internal-heap consumers are touched; the
           dynamic RX/TX pools stay (they land in PSRAM) and rx_ba_win is
           left at its default to avoid the buffer/BA-window constraint. */
        switch (cfg->ram_profile)
        {
            case WM_RAM_LEAN:
                init_cfg.static_rx_buf_num = 6;  /* 10 default */
                init_cfg.static_tx_buf_num = 4;  /* 8  default */
                init_cfg.cache_tx_buf_num = 16;  /* 32 default */
                break;
            case WM_RAM_CUSTOM:
                init_cfg.static_rx_buf_num = cfg->wifi_static_rx;
                init_cfg.static_tx_buf_num = cfg->wifi_static_tx;
                init_cfg.cache_tx_buf_num = cfg->wifi_cache_tx;
                break;
            case WM_RAM_FULL:
            default:
                break; /* IDF defaults */
        }

        ESP_LOGI(TAG, "wifi ram profile=%d (rx=%d tx=%d cache=%d)",
                 (int)cfg->ram_profile, init_cfg.static_rx_buf_num,
                 init_cfg.static_tx_buf_num, init_cfg.cache_tx_buf_num);

        err = esp_wifi_init(&init_cfg);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_wifi_init: %s", esp_err_to_name(err));
            return err;
        }

        err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                         wm_event_handler, NULL);

        if (err == ESP_OK)
        {
            err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             wm_event_handler, NULL);
        }

        if (err != ESP_OK)
        {
            return err;
        }

        static const wifi_country_t world =
            { .cc = "01", .schan = 1, .nchan = 14, .policy = WIFI_COUNTRY_POLICY_AUTO };

        esp_wifi_set_country(&world);
        esp_wifi_set_storage(WIFI_STORAGE_RAM); /* settings own persistence */
        s_status.radio_inited = true;
    }

    /* mode */
    wifi_mode_t mode = WIFI_MODE_APSTA;

    if (cfg->mode == WM_MODE_STA)
    {
        mode = WIFI_MODE_STA;
    }
    else if (cfg->mode == WM_MODE_AP)
    {
        mode = WIFI_MODE_AP;
    }

    err = esp_wifi_set_mode(mode);

    if (err != ESP_OK)
    {
        return err;
    }

    /* STA side */
    if (cfg->mode == WM_MODE_STA || cfg->mode == WM_MODE_APSTA)
    {
        dev_status_manager_set(DEV_STATUS_BIT_STA_ENABLED);

        if (cfg->hostname[0] != '\0')
        {
            esp_netif_set_hostname(s_sta_netif, cfg->hostname);
        }

        /* v6: per-network addressing is applied inside apply_sta_network
           (each candidate carries its own static/DHCP choice) */
        if (cfg->sta_count > 0)
        {
            apply_sta_network(cfg, 0);
        }
    }

    /* AP side */
    if (cfg->mode == WM_MODE_AP || cfg->mode == WM_MODE_APSTA)
    {
        wifi_config_t ap_cfg = { 0 };

        strncpy((char *)ap_cfg.ap.ssid, cfg->ap.ssid,
                sizeof(ap_cfg.ap.ssid) - 1);
        strncpy((char *)ap_cfg.ap.password, cfg->ap.password,
                sizeof(ap_cfg.ap.password) - 1);
        ap_cfg.ap.ssid_len = strlen(cfg->ap.ssid);
        ap_cfg.ap.channel = cfg->ap_channel;
        ap_cfg.ap.max_connection = cfg->ap_max_connections;
        ap_cfg.ap.ssid_hidden = cfg->ap_hidden ? 1 : 0;
        ap_cfg.ap.authmode = ap_authmode(cfg);
        ap_cfg.ap.beacon_interval = 100;

        /* WPA3-SAE (and the WPA2/WPA3 transitional mix) need PMF; the
           hash-to-element PWE interops with modern clients */
        if (ap_cfg.ap.authmode == WIFI_AUTH_WPA3_PSK ||
            ap_cfg.ap.authmode == WIFI_AUTH_WPA2_WPA3_PSK)
        {
            ap_cfg.ap.pmf_cfg.capable = true;
            ap_cfg.ap.pmf_cfg.required =
                (ap_cfg.ap.authmode == WIFI_AUTH_WPA3_PSK);
            ap_cfg.ap.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
        }

        err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);

        if (err != ESP_OK)
        {
            return err;
        }

        esp_netif_ip_info_t ip = { 0 };
        /* AP gateway IP (v4, configurable). A blank/invalid setting parsed
           to 0 in on_apply; fall back to the legacy WiCAN 192.168.80.1 that
           existing apps/docs hardcode and that avoids colliding with the
           common home-LAN subnets (a 192.168.0.x AP is unroutable from a
           client whose other interface sits on a 192.168.0.x LAN). A /24 is
           assumed; the DHCP pool follows the gateway automatically. */
        uint32_t host =
            cfg->ap_ip ? cfg->ap_ip : (uint32_t)0xC0A85001; /* 192.168.80.1 */

        ip.ip.addr = htonl(host);
        ip.gw.addr = ip.ip.addr;
        ip.netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0);

        esp_netif_dhcps_stop(s_ap_netif);
        esp_netif_set_ip_info(s_ap_netif, &ip);
        esp_netif_dhcps_start(s_ap_netif);
    }

    err = esp_wifi_start();

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_start: %s", esp_err_to_name(err));
        return err;
    }

    /* AP channel bandwidth (v4) — after start so the interface exists */
    if (cfg->mode == WM_MODE_AP || cfg->mode == WM_MODE_APSTA)
    {
        esp_wifi_set_bandwidth(WIFI_IF_AP,
                               cfg->ap_ht40 ? WIFI_BW40 : WIFI_BW20);
    }

    switch (cfg->power_save)
    {
        case WM_PS_MIN: esp_wifi_set_ps(WIFI_PS_MIN_MODEM); break;
        case WM_PS_MAX: esp_wifi_set_ps(WIFI_PS_MAX_MODEM); break;
        default:        esp_wifi_set_ps(WIFI_PS_NONE);      break;
    }

    s_status.started = true;
    xEventGroupClearBits(s_events, WM_BIT_STOP_REQ | WM_BIT_TASK_EXITED);
    xEventGroupSetBits(s_events, WIFI_MANAGER_BIT_ENABLED);

    if ((cfg->mode == WM_MODE_STA || cfg->mode == WM_MODE_APSTA) &&
        cfg->sta_auto_reconnect && cfg->sta_count > 0 &&
        s_reconnect_task == NULL)
    {
        s_reconnect_task = xTaskCreateStatic(
            reconnect_task, "wm_reconnect",
            sizeof(s_reconnect_stack) / sizeof(s_reconnect_stack[0]), NULL,
            5, s_reconnect_stack, &s_reconnect_tcb);
    }

    ESP_LOGI(TAG, "started (mode=%d, %u STA candidates)", (int)cfg->mode,
             (unsigned)cfg->sta_count);
    return ESP_OK;
}

esp_err_t wifi_manager_stop(void)
{
    if (!s_status.started)
    {
        return ESP_OK;
    }

    if (s_reconnect_task != NULL)
    {
        xEventGroupSetBits(s_events, WM_BIT_STOP_REQ);
        xEventGroupWaitBits(s_events, WM_BIT_TASK_EXITED, pdTRUE, pdFALSE,
                            pdMS_TO_TICKS(2 * WM_RECONNECT_PERIOD_MS));
    }

    if (s_status.sta_connected)
    {
        esp_wifi_disconnect();
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (s_status.radio_inited)
    {
        esp_wifi_stop();
    }

    s_status.started = false;
    s_status.sta_connected = false;
    s_status.ap_started = false;
    s_status.ap_station_count = 0;
    s_status.sta_retry_count = 0;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_status.sta_ip[0] = '\0';
    xSemaphoreGive(s_lock);

    xEventGroupClearBits(s_events, WIFI_MANAGER_BIT_ENABLED |
        WIFI_MANAGER_BIT_STA_CONNECTED | WIFI_MANAGER_BIT_STA_DISCONNECTED |
        WIFI_MANAGER_BIT_AP_STARTED | WIFI_MANAGER_BIT_STA_AP_OVERLAP);
    dev_status_manager_clear(DEV_STATUS_BIT_STA_CONNECTED |
                             DEV_STATUS_BIT_STA_ENABLED |
                             DEV_STATUS_BIT_AP_ENABLED |
                             DEV_STATUS_BIT_STA_AP_OVERLAP);

    ESP_LOGI(TAG, "stopped");
    return ESP_OK;
}

/* ---- bridge to wifi_manager_status.c (see wifi_manager_bridge.h) ----------- */

esp_err_t wm_status_copy_ip(char *buf, size_t buf_len)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    strlcpy(buf, s_status.sta_ip, buf_len);
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

bool wm_status_flag(int which)
{
    if (s_events == NULL)
    {
        return false;
    }

    EventBits_t mask = (which == 0) ? WIFI_MANAGER_BIT_ENABLED
                     : (which == 1) ? WIFI_MANAGER_BIT_STA_CONNECTED
                     : (which == 2) ? WIFI_MANAGER_BIT_AP_STARTED : 0;

    return (xEventGroupGetBits(s_events) & mask) != 0;
}

uint16_t wm_status_ap_clients(void)
{
    return s_status.ap_station_count;
}

esp_netif_t *wm_sta_netif(void)
{
    return s_sta_netif;
}

esp_netif_t *wm_ap_netif(void)
{
    return s_ap_netif;
}

EventGroupHandle_t wm_event_group(void)
{
    return s_events;
}

void wm_set_callbacks(const wifi_manager_callbacks_t *cbs)
{
    if (cbs != NULL)
    {
        s_callbacks = *cbs;
    }
    else
    {
        memset(&s_callbacks, 0, sizeof(s_callbacks));
    }
}

static bool s_scan_switched; /* AP->APSTA restore flag, guarded by s_lock */

esp_err_t wm_scan_locked(wifi_ap_record_t **records, uint16_t *count)
{
    if (!s_status.started || !s_status.radio_inited)
    {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_scan_switched = enter_scan_mode();

    uint16_t found = do_scan();

    *records = s_scan_records;
    *count = found;
    return ESP_OK; /* caller releases via wm_scan_unlock() */
}

void wm_scan_unlock(void)
{
    leave_scan_mode(s_scan_switched);
    s_scan_switched = false;
    xSemaphoreGive(s_lock);
}
