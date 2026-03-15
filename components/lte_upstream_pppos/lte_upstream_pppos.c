#include <string.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

#include "esp_console.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"

#include "lte_upstream_pppos.h"
#include "lte_upstream_pppos_usb.h"

#include "hw_config.h"
#include "driver/gpio.h"
#include "usb_dev_ethernet.h"
#include "esp_timer.h"
#include "cJSON.h"

#include <time.h>
#include <sys/time.h>

static const char *TAG = "lte_pppos";

static EventGroupHandle_t s_evt;
static const EventBits_t BIT_CONNECTED        = BIT0;
static const EventBits_t BIT_DISCONNECTED     = BIT1;
static const EventBits_t BIT_NETWORK_DETACHED = BIT2;

#if CONFIG_PPP_SUPPORT
#include "esp_modem_api.h"

static esp_modem_dce_t *s_dce;
static esp_netif_t *s_netif_ppp;
static bool s_enable_usb_share;
static volatile bool s_monitor_running;
static TaskHandle_t s_monitor_task;

static portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;
static lte_status_t s_status;   /* protected by s_status_mux */

/* --- reconnect supervisor --- */
static TaskHandle_t s_supervisor_task = NULL;
static volatile bool s_supervisor_running = false;
static volatile bool s_stop_requested = false;
static volatile bool s_start_in_progress = false;

/* Deep-copied config strings for use on reconnect */
static lte_upstream_pppos_config_t s_saved_cfg;
static char s_saved_apn[64];
static char s_saved_user[32];
static char s_saved_pass[32];
static char s_saved_pin[16];

/* -----------------------------------------------------------------------
 * System time synchronisation from modem (AT+QLTS=2)
 * ----------------------------------------------------------------------- */

static volatile bool s_time_synced = false;

/**
 * @brief Query modem for network time and set the ESP32 system clock.
 *
 * AT+QLTS=2 returns: +QLTS: "2025/06/14,12:34:56+32,0"
 * where +32 is timezone in quarter-hours.  We convert to UTC and call
 * settimeofday() so the whole system has a valid clock.
 */
static void lte_do_time_sync(void)
{
    if (!s_dce)
    {
        return;
    }

    char resp[128] = {0};
    esp_err_t err = esp_modem_at(s_dce, "AT+QLTS=2", resp, 3000);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "AT+QLTS=2 failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "QLTS response: %s", resp);

    const char *p = strchr(resp, '"');
    if (!p)
    {
        ESP_LOGW(TAG, "QLTS: no timestamp in response");
        return;
    }
    p++;

    int y, mo, d, h, mi, s, tz = 0;
    int n = sscanf(p, "%d/%d/%d,%d:%d:%d%d", &y, &mo, &d, &h, &mi, &s, &tz);
    if (n < 6)
    {
        ESP_LOGW(TAG, "QLTS: failed to parse time fields");
        return;
    }

    struct tm tm_utc = {
        .tm_year = y - 1900,
        .tm_mon  = mo - 1,
        .tm_mday = d,
        .tm_hour = h,
        .tm_min  = mi,
        .tm_sec  = s,
    };

    /* tz is in quarter-hours from UTC; mktime expects UTC input */
    time_t epoch = mktime(&tm_utc);
    if (epoch == (time_t)-1)
    {
        ESP_LOGW(TAG, "QLTS: mktime failed");
        return;
    }

    /* Subtract timezone offset to get UTC */
    epoch -= (time_t)tz * 15 * 60;

    struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    s_time_synced = true;

    /* Log the resulting UTC time */
    struct tm result;
    gmtime_r(&epoch, &result);
    ESP_LOGI(TAG, "System time set to UTC %04d-%02d-%02d %02d:%02d:%02d",
             result.tm_year + 1900, result.tm_mon + 1, result.tm_mday,
             result.tm_hour, result.tm_min, result.tm_sec);
}

static void lte_time_sync_task(void *arg)
{
    (void)arg;
    lte_do_time_sync();
    vTaskDelete(NULL);
}

/* Convert 3GPP TS 27.007 access-technology code to a short label */
static const char *lte_act_to_str(int act)
{
    switch (act)
    {
    case 0:  return "GSM";
    case 1:  return "GSM Compact";
    case 2:  return "3G";
    case 3:  return "EDGE";
    case 4:  return "HSDPA";
    case 5:  return "HSUPA";
    case 6:  return "HSPA+";
    case 7:  return "LTE";
    case 8:  return "EC-GSM-IoT";
    case 9:  return "NB-IoT";
    case 10: return "LTE (5GCN)";
    case 11: return "5G NR";
    case 12: return "5G NG-RAN";
    case 13: return "LTE-NR";
    default: return "unknown";
    }
}

/* Query AT+QNWINFO (Quectel-specific) for the actual serving network type.
 * Parses the first quoted token from the response:
 *   +QNWINFO: "CAT-M1","50501","LTE BAND 28",9410
 * Writes up to (out_len-1) chars into out.  Returns ESP_OK on success,
 * or an error code if the modem does not support the command. */
static esp_err_t query_qnwinfo(char *out, size_t out_len)
{
    char resp[128] = {0};
    esp_err_t err = esp_modem_at(s_dce, "AT+QNWINFO", resp, 2000);
    if (err != ESP_OK)
    {
        return err;
    }
    /* Find the first quoted token */
    const char *p = strchr(resp, '"');
    if (!p)
    {
        return ESP_ERR_NOT_FOUND;
    }
    p++; /* skip opening quote */
    const char *end = strchr(p, '"');
    if (!end)
    {
        return ESP_ERR_NOT_FOUND;
    }
    size_t len = (size_t)(end - p);
    if (len >= out_len)
    {
        len = out_len - 1;
    }
    memcpy(out, p, len);
    out[len] = '\0';
    return ESP_OK;
}

static void on_ppp_changed(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;
    (void)event_data;

    switch (event_id)
    {
    case NETIF_PPP_ERRORNONE:
        ESP_LOGI(TAG, "PPP status: OK");
        break;
    case NETIF_PPP_ERRORPARAM:
        ESP_LOGE(TAG, "PPP error: invalid parameter");
        break;
    case NETIF_PPP_ERROROPEN:
        ESP_LOGE(TAG, "PPP error: unable to open session");
        break;
    case NETIF_PPP_ERRORDEVICE:
        ESP_LOGE(TAG, "PPP error: invalid I/O device");
        break;
    case NETIF_PPP_ERRORALLOC:
        ESP_LOGE(TAG, "PPP error: alloc failed");
        break;
    case NETIF_PPP_ERRORUSER:
        ESP_LOGI(TAG, "PPP status: user interrupt (expected during mode changes/stop)");
        break;
    case NETIF_PPP_ERRORCONNECT:
        ESP_LOGW(TAG, "PPP error: connection lost");
        break;
    case NETIF_PPP_ERRORAUTHFAIL:
        ESP_LOGE(TAG, "PPP error: auth failed (check APN/user/pass and PPP auth config)");
        break;
    case NETIF_PPP_ERRORPROTOCOL:
        ESP_LOGE(TAG, "PPP error: protocol error");
        break;
    case NETIF_PPP_ERRORPEERDEAD:
        ESP_LOGE(TAG, "PPP error: peer not responding (timeout)");
        break;
    case NETIF_PPP_ERRORIDLETIMEOUT:
        ESP_LOGW(TAG, "PPP error: idle timeout");
        break;
    case NETIF_PPP_ERRORCONNECTTIME:
        ESP_LOGE(TAG, "PPP error: connect time");
        break;
    case NETIF_PPP_ERRORLOOPBACK:
        ESP_LOGE(TAG, "PPP error: loopback");
        break;
    default:
        ESP_LOGI(TAG, "PPP status event %ld", (long)event_id);
        break;
    }
}

/* Wait until the SIM application is ready (AT+CIMI returns a valid IMSI).
 * On BG95 after a cold power-on the SIM stack needs ~2-5 s to initialise.
 * Without waiting, AT+CPIN? may still report READY but AT+CIMI fails and
 * the modem cannot attach to any operator.  Returns ESP_OK once ready,
 * ESP_ERR_TIMEOUT if the SIM does not come up within sim_wait_ms. */
static esp_err_t wait_for_sim_ready(esp_modem_dce_t *dce, uint32_t sim_wait_ms)
{
    const int64_t deadline = esp_timer_get_time() / 1000 + sim_wait_ms;
    int attempt = 0;
    while ((esp_timer_get_time() / 1000) < deadline)
    {
        char imsi[32] = {0};
        if (esp_modem_at(dce, "AT+CIMI", imsi, 2000) == ESP_OK && imsi[0] >= '0' && imsi[0] <= '9')
        {
            ESP_LOGI(TAG, "SIM ready (IMSI: %s) after %d attempts", imsi, attempt + 1);
            return ESP_OK;
        }
        attempt++;
        ESP_LOGD(TAG, "SIM not ready yet (attempt %d)...", attempt);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGW(TAG, "SIM not ready after %u ms – continuing anyway", (unsigned)sim_wait_ms);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t wait_for_cellular_attach(esp_modem_dce_t *dce, uint32_t timeout_ms,
                                           const char *pin,
                                           bool reboot_on_timeout,
                                           uint32_t reboot_wait_ms)
{
    /* Wait for SIM application to be fully initialised before polling
     * registration.  Allow up to 30 s; deduct the wait from the overall
     * registration deadline so we don't extend the total timeout. */
    const int64_t t_start = esp_timer_get_time() / 1000;
    wait_for_sim_ready(dce, 30000);
    const uint32_t sim_wait_elapsed = (uint32_t)(esp_timer_get_time() / 1000 - t_start);
    if (sim_wait_elapsed < timeout_ms)
    {
        timeout_ms -= sim_wait_elapsed;
    }
    else
    {
        return ESP_ERR_TIMEOUT;
    }

    const int64_t deadline = esp_timer_get_time() / 1000 + timeout_ms;
    bool pin_ok = true;
    if (esp_modem_read_pin(dce, &pin_ok) == ESP_OK && !pin_ok)
    {
        if (pin && pin[0] != '\0')
        {
            ESP_LOGI(TAG, "SIM PIN required – entering PIN");
            esp_err_t perr = esp_modem_set_pin(dce, (char *)pin);
            if (perr != ESP_OK)
            {
                ESP_LOGE(TAG, "SIM PIN entry failed: %s", esp_err_to_name(perr));
                return perr;
            }
            /* Give the modem a moment to verify the PIN and unlock */
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        else
        {
            ESP_LOGE(TAG, "SIM PIN required but no PIN configured");
            return ESP_ERR_INVALID_STATE;
        }
    }

    /* ---- Check if modem is already registered before touching radio config ---- */
    {
        int reg_state = 0;
        if (esp_modem_get_network_registration_state(dce, &reg_state) == ESP_OK &&
            (reg_state == 1 || reg_state == 5))
        {
            ESP_LOGI(TAG, "Already registered (CEREG state=%d) – skipping radio config", reg_state);
            return ESP_OK;
        }
    }

    /* ---- one-shot diagnostics before the registration poll loop ---- */
    {
        char diag_buf[128] = {0};

        /* --- Throughput-oriented radio profile (best-effort) ---
         * Only SET a QCFG param when the current value differs from the
         * desired one.  Blindly writing nwscanmode / iotopmode can trigger
         * a network detach + reattach on BG95, adding ~50 s to startup.
         *
         * BG95 notes:
         *  - iotopmode=0 -> Cat-M1 only (avoid NB-IoT fallback for data tests)
         *  - nwscanmode=3 -> LTE-only scan path
         *  - CPSMS=0 / CEDRXS=0 -> disable power-saving that can throttle data
         */

        /* Read-then-set for QCFG params that can cause network re-scan */
        static const struct {
            const char *query;       /* read command */
            const char *needle;      /* expected substring in response if already correct */
            const char *set_cmd;     /* write command (only if needle not found) */
        } qcfg_checks[] = {
            { "AT+QCFG=\"iotopmode\"",  ",0",  "AT+QCFG=\"iotopmode\",0" },
            { "AT+QCFG=\"nwscanmode\"", ",3",  "AT+QCFG=\"nwscanmode\",3" },
        };
        for (size_t i = 0; i < sizeof(qcfg_checks) / sizeof(qcfg_checks[0]); i++)
        {
            memset(diag_buf, 0, sizeof(diag_buf));
            esp_err_t qerr = esp_modem_at(dce, qcfg_checks[i].query, diag_buf, 2000);
            if (qerr == ESP_OK && strstr(diag_buf, qcfg_checks[i].needle))
            {
                ESP_LOGI(TAG, "%s -> %s (already correct)", qcfg_checks[i].query, diag_buf);
            }
            else
            {
                /* Value differs or query failed – set it */
                memset(diag_buf, 0, sizeof(diag_buf));
                esp_err_t serr = esp_modem_at(dce, qcfg_checks[i].set_cmd, diag_buf, 2500);
                if (serr == ESP_OK)
                    ESP_LOGI(TAG, "%s -> %s", qcfg_checks[i].set_cmd, diag_buf[0] ? diag_buf : "OK");
                else
                    ESP_LOGW(TAG, "%s failed: %s", qcfg_checks[i].set_cmd, esp_err_to_name(serr));
            }
        }

        /* PSM and eDRX are safe to set unconditionally (no network re-scan) */
        static const char *safe_cmds[] = {
            "AT+CPSMS=0",
            "AT+CEDRXS=0",
        };
        for (size_t i = 0; i < sizeof(safe_cmds) / sizeof(safe_cmds[0]); i++)
        {
            memset(diag_buf, 0, sizeof(diag_buf));
            esp_err_t perr = esp_modem_at(dce, safe_cmds[i], diag_buf, 2500);
            if (perr == ESP_OK)
                ESP_LOGI(TAG, "%s -> %s", safe_cmds[i], diag_buf[0] ? diag_buf : "OK");
            else
                ESP_LOGW(TAG, "%s failed: %s", safe_cmds[i], esp_err_to_name(perr));
        }

        /* Verify SIM: IMSI – if this fails the SIM is locked, absent, or dead */
        if (esp_modem_at(dce, "AT+CIMI", diag_buf, 2000) == ESP_OK)
            ESP_LOGI(TAG, "SIM IMSI  : %s", diag_buf[0] ? diag_buf : "(empty)");
        else
            ESP_LOGW(TAG, "AT+CIMI failed – SIM may be absent or locked");

        memset(diag_buf, 0, sizeof(diag_buf));
        /* ICCID */
        if (esp_modem_at(dce, "AT+CCID", diag_buf, 2000) == ESP_OK)
            ESP_LOGI(TAG, "SIM ICCID : %s", diag_buf[0] ? diag_buf : "(empty)");

        memset(diag_buf, 0, sizeof(diag_buf));
        /* Read back PDP context to confirm APN was stored */
        if (esp_modem_at(dce, "AT+CGDCONT?", diag_buf, 2000) == ESP_OK)
            ESP_LOGI(TAG, "AT+CGDCONT? -> %s", diag_buf[0] ? diag_buf : "(empty)");
        else
            ESP_LOGW(TAG, "AT+CGDCONT? failed");

        memset(diag_buf, 0, sizeof(diag_buf));
        /* Configured LTE bands (BG95-specific) */
        if (esp_modem_at(dce, "AT+QCFG=\"band\"", diag_buf, 2000) == ESP_OK)
            ESP_LOGI(TAG, "AT+QCFG=band -> %s", diag_buf[0] ? diag_buf : "(empty)");

        memset(diag_buf, 0, sizeof(diag_buf));
        /* RAT scan sequence / mode */
        if (esp_modem_at(dce, "AT+QCFG=\"nwscanseq\"", diag_buf, 2000) == ESP_OK)
            ESP_LOGI(TAG, "AT+QCFG=nwscanseq -> %s", diag_buf[0] ? diag_buf : "(empty)");

        memset(diag_buf, 0, sizeof(diag_buf));
        if (esp_modem_at(dce, "AT+QCFG=\"nwscanmode\"", diag_buf, 2000) == ESP_OK)
            ESP_LOGI(TAG, "AT+QCFG=nwscanmode -> %s", diag_buf[0] ? diag_buf : "(empty)");

        memset(diag_buf, 0, sizeof(diag_buf));
        if (esp_modem_at(dce, "AT+QCFG=\"iotopmode\"", diag_buf, 2000) == ESP_OK)
            ESP_LOGI(TAG, "AT+QCFG=iotopmode -> %s", diag_buf[0] ? diag_buf : "(empty)");
    }

    int poll_count = 0;
    while ((esp_timer_get_time() / 1000) < deadline)
    {
        if (s_stop_requested)
        {
            ESP_LOGW(TAG, "LTE attach cancelled by stop request");
            return ESP_ERR_INVALID_STATE;
        }

        int rssi = 0, ber = 0;
        (void)esp_modem_get_signal_quality(dce, &rssi, &ber);

        /* Detailed Quectel signal quality every 10 s */
        if ((poll_count % 5) == 0)
        {
            char qcsq_buf[64] = {0};
            if (esp_modem_at(dce, "AT+QCSQ", qcsq_buf, 1000) == ESP_OK && qcsq_buf[0])
                ESP_LOGI(TAG, "AT+QCSQ -> %s", qcsq_buf);
        }

        /* Primary check: AT+CEREG? (EPS/LTE registration – states 1=home, 5=roaming).
         * AT+CGATT? only covers GPRS PS attach and stays 0 on LTE-only networks,
         * so we prefer CEREG and fall back to CGATT if CEREG is unavailable. */
        int reg_state = 0;
        bool registered = false;
        if (esp_modem_get_network_registration_state(dce, &reg_state) == ESP_OK)
        {
            registered = (reg_state == 1 || reg_state == 5);
            if (registered)
            {
                ESP_LOGI(TAG, "LTE registered (CEREG state=%d, rssi=%d ber=%d)", reg_state, rssi, ber);
                return ESP_OK;
            }
            ESP_LOGW(TAG, "Waiting for LTE registration... (CEREG state=%d rssi=%d ber=%d)",
                     reg_state, rssi, ber);
        }
        else
        {
            /* CEREG not supported – fall back to AT+CGATT? */
            int attached = 0;
            if (esp_modem_get_network_attachment_state(dce, &attached) == ESP_OK && attached == 1)
            {
                ESP_LOGI(TAG, "Cellular attached via CGATT (rssi=%d ber=%d)", rssi, ber);
                return ESP_OK;
            }
            ESP_LOGW(TAG, "Waiting for cellular attach... (rssi=%d ber=%d)", rssi, ber);
        }

        poll_count++;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    /* ---- timeout diagnostics: scan visible operators ---- */
    ESP_LOGE(TAG, "Registration timed out – dumping diagnostics:");
    {
        char diag_buf[256] = {0};
        if (esp_modem_at(dce, "AT+CEREG?", diag_buf, 1000) == ESP_OK)
            ESP_LOGI(TAG, "  CEREG?       : %s", diag_buf[0] ? diag_buf : "(empty)");
        memset(diag_buf, 0, sizeof(diag_buf));
        if (esp_modem_at(dce, "AT+CREG?", diag_buf, 1000) == ESP_OK)
            ESP_LOGI(TAG, "  CREG?        : %s", diag_buf[0] ? diag_buf : "(empty)");
        memset(diag_buf, 0, sizeof(diag_buf));
        if (esp_modem_at(dce, "AT+QCSQ", diag_buf, 1000) == ESP_OK)
            ESP_LOGI(TAG, "  QCSQ         : %s", diag_buf[0] ? diag_buf : "(empty)");
        memset(diag_buf, 0, sizeof(diag_buf));
        /* Operator scan – can take up to 3 min; use 180 s timeout */
        ESP_LOGI(TAG, "  Starting operator scan (AT+COPS=?) – may take up to 3 min...");
        if (esp_modem_at(dce, "AT+COPS=?", diag_buf, 180000) == ESP_OK)
            ESP_LOGI(TAG, "  COPS=?       : %s", diag_buf[0] ? diag_buf : "(no operators found)");
        else
            ESP_LOGW(TAG, "  COPS=? timed out or failed");
    }

    if (reboot_on_timeout)
    {
        char resp[64] = {0};
        const uint32_t wait_ms = (reboot_wait_ms > 0) ? reboot_wait_ms : 15000;
        ESP_LOGW(TAG, "Registration timeout: issuing soft reboot (AT+CFUN=1,1), then waiting %u ms", (unsigned)wait_ms);
        esp_err_t rerr = esp_modem_at(dce, "AT+CFUN=1,1", resp, 2000);
        if (rerr == ESP_OK)
        {
            ESP_LOGI(TAG, "AT+CFUN=1,1 -> %s", resp[0] ? resp : "OK");
        }
        else
        {
            ESP_LOGW(TAG, "AT+CFUN=1,1 failed: %s", esp_err_to_name(rerr));
        }
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }

    return ESP_ERR_TIMEOUT;
}

static void on_ip_event(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;

    if (event_id == IP_EVENT_PPP_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        esp_netif_t *netif = event->esp_netif;

        esp_netif_dns_info_t dns_info;
        uint32_t dns_main_addr = 0;

        gpio_set_level(LED_RED_PIN, 0); /* active-low: LED ON */
        ESP_LOGI(TAG, "PPP GOT IP");
        ESP_LOGI(TAG, "IP      : " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Netmask : " IPSTR, IP2STR(&event->ip_info.netmask));
        ESP_LOGI(TAG, "Gateway : " IPSTR, IP2STR(&event->ip_info.gw));
        if (esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns_info) == ESP_OK)
        {
            ESP_LOGI(TAG, "DNS1    : " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
            dns_main_addr = dns_info.ip.u_addr.ip4.addr;
        }
        if (esp_netif_get_dns_info(netif, ESP_NETIF_DNS_BACKUP, &dns_info) == ESP_OK)
        {
            ESP_LOGI(TAG, "DNS2    : " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
        }

        portENTER_CRITICAL(&s_status_mux);
        s_status.current_ip = event->ip_info.ip;
        portEXIT_CRITICAL(&s_status_mux);

        if (s_enable_usb_share)
        {
            esp_err_t share_err = usb_dev_ethernet_enable_sharing(dns_main_addr);
            if (share_err != ESP_OK)
            {
                ESP_LOGW(TAG, "usb_dev_ethernet_enable_sharing failed: %s", esp_err_to_name(share_err));
            }
        }

        /* Sync system clock from modem network time (runs in a short-lived
         * task because AT commands block and event handlers must be fast) */
        xTaskCreate(lte_time_sync_task, "lte_tsync", 3072, NULL, 5, NULL);

        if (s_evt)
        {
            xEventGroupClearBits(s_evt, BIT_DISCONNECTED);
            xEventGroupSetBits(s_evt, BIT_CONNECTED);
        }
    }
    else if (event_id == IP_EVENT_PPP_LOST_IP)
    {
        gpio_set_level(LED_RED_PIN, 1); /* active-low: LED OFF */
        ESP_LOGW(TAG, "PPP LOST IP");
        portENTER_CRITICAL(&s_status_mux);
        s_status.current_ip.addr = 0;
        portEXIT_CRITICAL(&s_status_mux);
        if (s_evt)
        {
            xEventGroupClearBits(s_evt, BIT_CONNECTED);
            xEventGroupSetBits(s_evt, BIT_DISCONNECTED);
        }
    }
}

static esp_err_t register_handlers(void)
{
    esp_err_t err;

    err = esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &on_ip_event, NULL);
    if (err != ESP_OK)
    {
        return err;
    }

    err = esp_event_handler_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, &on_ppp_changed, NULL);
    if (err != ESP_OK)
    {
        (void)esp_event_handler_unregister(IP_EVENT, ESP_EVENT_ANY_ID, &on_ip_event);
        return err;
    }

    return ESP_OK;
}

static void unregister_handlers(void)
{
    (void)esp_event_handler_unregister(IP_EVENT, ESP_EVENT_ANY_ID, &on_ip_event);
    (void)esp_event_handler_unregister(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, &on_ppp_changed);
}

static void lte_monitor_task(void *arg)
{
    (void)arg;
    static const char *TAG_MON = "lte_monitor";
    char op_name[64] = {0};
    int op_act = 0;
    char net_type[32] = {0};  /* populated by QNWINFO or lte_act_to_str fallback */
    uint32_t poll_count = 0;

    while (s_monitor_running)
    {
        /* Sleep 15 s in 100 ms chunks so stop() can interrupt quickly */
        for (int i = 0; i < 150 && s_monitor_running; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (!s_monitor_running || !s_dce)
        {
            break;
        }

        poll_count++;

        /* --- Gather metrics --- */

        int rssi = 99, ber = 99;
        if (esp_modem_get_signal_quality(s_dce, &rssi, &ber) == ESP_OK)
        {
            int rssi_dbm = (rssi < 99) ? (rssi * 2 - 113) : 0;
            ESP_LOGI(TAG_MON, "Signal: RSSI=%d (%d dBm), BER=%d", rssi, rssi_dbm, ber);
        }
        else
        {
            ESP_LOGW(TAG_MON, "Signal quality query failed");
        }

        /* Use AT+CEREG? for LTE/EPS registration (states 1=home, 5=roaming).
         * AT+CGATT? only covers GPRS-PS and returns 0 on LTE-only networks,
         * which would incorrectly keep BIT_NETWORK_DETACHED set and prevent
         * the supervisor from ever seeing a healthy connection. */
        int reg_state = 0;
        bool net_registered = false;
        if (esp_modem_get_network_registration_state(s_dce, &reg_state) == ESP_OK)
        {
            net_registered = (reg_state == 1 || reg_state == 5);
            ESP_LOGI(TAG_MON, "Network: %s (CEREG state=%d)",
                     net_registered ? "registered" : "not registered", reg_state);
        }
        else
        {
            /* CEREG unavailable – fall back to AT+CGATT? */
            int attached = 0;
            if (esp_modem_get_network_attachment_state(s_dce, &attached) == ESP_OK)
            {
                net_registered = (attached == 1);
                ESP_LOGI(TAG_MON, "Network: %s (CGATT)", net_registered ? "attached" : "detached");
            }
        }
        if (s_evt)
        {
            if (net_registered)
                xEventGroupClearBits(s_evt, BIT_NETWORK_DETACHED);
            else
                xEventGroupSetBits(s_evt, BIT_NETWORK_DETACHED);
        }

        bool ppp_up = false;
        if (s_evt)
        {
            EventBits_t bits = xEventGroupGetBits(s_evt);
            ppp_up = (bits & BIT_CONNECTED) && !(bits & BIT_DISCONNECTED);
            ESP_LOGI(TAG_MON, "PPP: %s", ppp_up ? "connected" : "disconnected");
        }

        /* Re-sync system time every ~60 s (alongside operator query) or
         * on the very first poll to catch the case where GOT_IP fired
         * before the modem had NITZ time available. */
        if (!s_time_synced || poll_count % 4 == 1)
        {
            lte_do_time_sync();
        }

        /* Operator name & network type – query every 4th poll (~60 s) */
        if (poll_count % 4 == 1)
        {
            if (esp_modem_get_operator_name(s_dce, op_name, &op_act) == ESP_OK)
            {
                /* Prefer AT+QNWINFO (Quectel) for the actual serving technology.
                 * It correctly distinguishes CAT-M1 vs NB-IoT vs GSM where the
                 * 3GPP +COPS act code can be ambiguous (e.g. act=8 on BG95). */
                char qnw[32] = {0};
                if (query_qnwinfo(qnw, sizeof(qnw)) == ESP_OK && qnw[0] != '\0')
                {
                    strncpy(net_type, qnw, sizeof(net_type) - 1);
                    net_type[sizeof(net_type) - 1] = '\0';
                    ESP_LOGI(TAG_MON, "Operator: \"%s\"  Network type: %s (QNWINFO, act=%d)",
                             op_name, net_type, op_act);
                }
                else
                {
                    /* QNWINFO not supported – fall back to 3GPP act code */
                    strncpy(net_type, lte_act_to_str(op_act), sizeof(net_type) - 1);
                    net_type[sizeof(net_type) - 1] = '\0';
                    ESP_LOGI(TAG_MON, "Operator: \"%s\"  Network type: %s (act=%d)",
                             op_name, net_type, op_act);
                }
            }
        }

        /* --- Commit to shared status struct --- */
        portENTER_CRITICAL(&s_status_mux);
        s_status.valid         = true;
        s_status.rssi          = rssi;
        s_status.rssi_dbm      = (rssi < 99) ? (rssi * 2 - 113) : 0;
        s_status.ber           = ber;
        s_status.attached      = net_registered;
        s_status.ppp_connected = ppp_up;
        s_status.operator_act  = op_act;
        /* op_name / net_type only change every 4 polls; copy every time (cheap) */
        memcpy(s_status.operator_name, op_name, sizeof(s_status.operator_name));
        strncpy(s_status.network_type, net_type[0] ? net_type : lte_act_to_str(op_act),
                sizeof(s_status.network_type) - 1);
        s_status.network_type[sizeof(s_status.network_type) - 1] = '\0';
        portEXIT_CRITICAL(&s_status_mux);
    }

    s_monitor_task = NULL;
    vTaskDelete(NULL);
}

/* -----------------------------------------------------------------------
 * lte_save_config – deep-copy caller's config so we can reconnect later.
 * ----------------------------------------------------------------------- */
static void lte_save_config(const lte_upstream_pppos_config_t *cfg)
{
    s_saved_cfg = *cfg;
    s_saved_apn[0] = s_saved_user[0] = s_saved_pass[0] = s_saved_pin[0] = '\0';

    if (cfg->apn)
    {
        strncpy(s_saved_apn, cfg->apn, sizeof(s_saved_apn) - 1);
        s_saved_cfg.apn = s_saved_apn;
    }
    if (cfg->user)
    {
        strncpy(s_saved_user, cfg->user, sizeof(s_saved_user) - 1);
        s_saved_cfg.user = s_saved_user;
    }
    if (cfg->pass)
    {
        strncpy(s_saved_pass, cfg->pass, sizeof(s_saved_pass) - 1);
        s_saved_cfg.pass = s_saved_pass;
    }
    if (cfg->pin)
    {
        strncpy(s_saved_pin, cfg->pin, sizeof(s_saved_pin) - 1);
        s_saved_cfg.pin = s_saved_pin;
    }
    /* Supervisor manages its own reconnect timing; disable blocking wait. */
    s_saved_cfg.connect_timeout_ms = 0;
}

/* -----------------------------------------------------------------------
 * lte_inner_stop – tears down DCE / netif / event-group.
 * Does NOT touch s_supervisor_task / s_supervisor_running.
 * ----------------------------------------------------------------------- */
static void lte_inner_stop(void)
{
    s_enable_usb_share = false;

    s_monitor_running = false;
    for (int i = 0; i < 20 && s_monitor_task != NULL; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (s_monitor_task != NULL)
    {
        vTaskDelete(s_monitor_task);
        s_monitor_task = NULL;
    }

    if (s_dce)
    {
        esp_modem_destroy(s_dce);
        s_dce = NULL;
    }
    if (s_netif_ppp)
    {
        esp_netif_destroy(s_netif_ppp);
        s_netif_ppp = NULL;
    }

    unregister_handlers();

    if (s_evt)
    {
        vEventGroupDelete(s_evt);
        s_evt = NULL;
    }

    portENTER_CRITICAL(&s_status_mux);
    s_status = (lte_status_t){0};
    portEXIT_CRITICAL(&s_status_mux);
}

/* Forward declaration – lte_supervisor_task calls lte_inner_start. */
static esp_err_t lte_inner_start(const lte_upstream_pppos_config_t *cfg);

/* -----------------------------------------------------------------------
 * lte_supervisor_task – watches for PPP failure and drives reconnect.
 *
 * Triggers (all require status.valid):
 *  1. BIT_DISCONNECTED set         → act immediately (skip patience window)
 *  2. ppp_connected=false          → 5 consecutive polls (~75 s) then reconnect
 *  3. ppp_connected=true but
 *     attached=false               → 5 consecutive polls (~75 s) then reconnect
 *     (stale PPP session: network dropped but no IP-lost event yet)
 *
 * Backoff: 5 s → 10 s → 20 s → 40 s → 60 s (max), reset on full health.
 * ----------------------------------------------------------------------- */
static void lte_supervisor_task(void *arg)
{
    (void)arg;
    static const char *TAG_SUP = "lte_sup";

    int      consecutive_down     = 0;
    int      consecutive_detached = 0;
    int      consecutive_failures = 0;
    unsigned backoff_ms           = 5000;
    const unsigned BACKOFF_MAX    = 30000;
    /* Threshold after which a hung/crashing modem gets a hard power-cycle */
    const int POWER_CYCLE_THRESHOLD = 3;
    /* Remember original setting – supervisor may temporarily override it */
    const bool orig_init_modem_manager = s_saved_cfg.init_modem_manager;

    while (s_supervisor_running)
    {
        /* Sleep 15 s in 100 ms chunks – same pattern as lte_monitor_task.
         * Using vTaskDelay in small increments ensures the IDLE task runs
         * and feeds the task watchdog, unlike a single long xEventGroupWaitBits. */
        EventBits_t bits = 0;
        for (int i = 0; i < 150 && s_supervisor_running; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            /* Wake early if BIT_DISCONNECTED is already set */
            if (s_evt && (xEventGroupGetBits(s_evt) & BIT_DISCONNECTED))
            {
                bits = BIT_DISCONNECTED;
                break;
            }
        }

        if (!s_supervisor_running)
        {
            break;
        }

        const bool explicit_disconnect = (bits & BIT_DISCONNECTED) != 0;

        portENTER_CRITICAL(&s_status_mux);
        const bool ppp_up  = s_status.ppp_connected;
        const bool valid   = s_status.valid;
        const bool net_ok  = s_status.attached;
        portEXIT_CRITICAL(&s_status_mux);

        if (valid && ppp_up && net_ok)
        {
            /* Truly healthy – reset all counters */
            consecutive_down     = 0;
            consecutive_detached = 0;
            consecutive_failures = 0;
            backoff_ms = 5000;
            continue;
        }

        bool do_reconnect = false;

        if (explicit_disconnect)
        {
            /* Explicit IP-lost event – act immediately */
            ESP_LOGW(TAG_SUP, "BIT_DISCONNECTED received");
            consecutive_down     = 0;
            consecutive_detached = 0;
            do_reconnect = true;
        }
        else if (valid && !ppp_up)
        {
            consecutive_detached = 0;
            consecutive_down++;
            ESP_LOGD(TAG_SUP, "PPP down poll %d/5", consecutive_down);
            if (consecutive_down >= 5)
            {
                consecutive_down = 0;
                do_reconnect = true;
            }
        }
        else if (valid && ppp_up && !net_ok)
        {
            /* PPP appears up but network is detached – stale session */
            consecutive_down = 0;
            consecutive_detached++;
            ESP_LOGW(TAG_SUP, "Network detached while PPP up (%d/5 polls)",
                     consecutive_detached);
            if (consecutive_detached >= 5)
            {
                consecutive_detached = 0;
                do_reconnect = true;
            }
        }
        else
        {
            /* Status not yet valid – modem is still starting up, or the initial
             * lte_inner_start failed before the monitor task could run.
             * Count consecutive polls so we don't spin here forever; trigger
             * a reconnect after the patience threshold. */
            consecutive_down++;
            ESP_LOGD(TAG_SUP, "Modem status not valid yet (%d/5 polls)", consecutive_down);
            if (consecutive_down >= 5)
            {
                consecutive_down = 0;
                ESP_LOGW(TAG_SUP, "No valid modem status after 5 polls – triggering reconnect");
                do_reconnect = true;
            }
        }

        if (!do_reconnect)
        {
            continue;
        }

        ESP_LOGW(TAG_SUP, "PPP connection lost – reconnecting in %u ms", backoff_ms);

        for (unsigned w = 0; w < backoff_ms && s_supervisor_running; w += 500)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        if (!s_supervisor_running)
        {
            break;
        }

        ESP_LOGI(TAG_SUP, "Tearing down LTE stack...");
        lte_inner_stop();

        /* After POWER_CYCLE_THRESHOLD consecutive failures the modem is likely
         * hung (bad state, crashed firmware, hardware issue).  Force a hard
         * power-cycle via modem_manager so we never stay stuck forever. */
        if (consecutive_failures >= POWER_CYCLE_THRESHOLD)
        {
            ESP_LOGW(TAG_SUP, "Modem unresponsive after %d attempts – forcing power-cycle",
                     consecutive_failures);
            s_saved_cfg.init_modem_manager = true;
        }

        ESP_LOGI(TAG_SUP, "Restarting LTE stack (attempt %d)...", consecutive_failures + 1);
        esp_err_t err = lte_inner_start(&s_saved_cfg);

        /* Always restore original modem-manager setting after the attempt */
        s_saved_cfg.init_modem_manager = orig_init_modem_manager;

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG_SUP, "LTE stack restarted – waiting for IP");
            consecutive_failures = 0;
            backoff_ms = 5000;
        }
        else
        {
            consecutive_failures++;
            unsigned next = (backoff_ms * 2 > BACKOFF_MAX) ? BACKOFF_MAX : backoff_ms * 2;
            ESP_LOGE(TAG_SUP, "LTE restart failed (attempt %d): %s – next retry in %u ms",
                     consecutive_failures, esp_err_to_name(err), next);
            backoff_ms = next;
        }
    }

    s_supervisor_task = NULL;
    vTaskDelete(NULL);
}

/* -----------------------------------------------------------------------
 * lte_inner_start – sets up DCE / netif / event-group.
 * Does NOT touch s_supervisor_task / s_supervisor_running.
 * Error paths call lte_inner_stop() for clean self-rollback.
 * ----------------------------------------------------------------------- */
static esp_err_t lte_inner_start(const lte_upstream_pppos_config_t *cfg)
{
    if (s_dce || s_netif_ppp)
    {
        return ESP_ERR_INVALID_STATE;
    }

    s_enable_usb_share = cfg->enable_usb_ncm_sharing;

    ESP_LOGI(TAG, "PPPoS start: apn='%s' hwfc=%s baud=%d share_usb=%s", cfg->apn,
             cfg->hw_flow_control ? "on" : "off",
             (cfg->baud_rate > 0) ? cfg->baud_rate : 0,
             cfg->enable_usb_ncm_sharing ? "yes" : "no");

    esp_err_t err;

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }

    if (cfg->init_modem_manager)
    {
        ESP_LOGI(TAG, "Initializing modem via modem_manager...");
        err = modem_mgr_init(&cfg->modem);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "modem_mgr_init failed: %s", esp_err_to_name(err));
            return err;
        }
    }

    if (!s_evt)
    {
        s_evt = xEventGroupCreate();
        if (!s_evt)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    xEventGroupClearBits(s_evt, BIT_CONNECTED | BIT_DISCONNECTED | BIT_NETWORK_DETACHED);

    err = register_handlers();
    if (err != ESP_OK)
    {
        vEventGroupDelete(s_evt);
        s_evt = NULL;
        return err;
    }

    esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();
    s_netif_ppp = esp_netif_new(&netif_ppp_config);
    if (!s_netif_ppp)
    {
        unregister_handlers();
        vEventGroupDelete(s_evt);
        s_evt = NULL;
        return ESP_ERR_NO_MEM;
    }

    esp_netif_ppp_config_t ppp_events = {
        .ppp_phase_event_enabled = true,
        .ppp_error_event_enabled = true,
#ifdef CONFIG_LWIP_ENABLE_LCP_ECHO
        .ppp_lcp_echo_disabled = true,   /* saves ~1–2% bandwidth on eMTC */
#endif
    };
    (void)esp_netif_ppp_set_params(s_netif_ppp, &ppp_events);

    if (cfg->user && cfg->pass)
    {
#if CONFIG_LWIP_PPP_PAP_SUPPORT
        (void)esp_netif_ppp_set_auth(s_netif_ppp, NETIF_PPP_AUTHTYPE_PAP, cfg->user, cfg->pass);
#else
        ESP_LOGW(TAG, "PPP user/pass provided but PAP support is disabled. Skipping auth.");
#endif
    }

    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(cfg->apn);
    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();

    /* Increase DTE internal ring buffer from the default 512 B to 32 KB.
     * PPPoS feeds data to lwIP in chunks of this size; the tiny default
     * causes excessive per-chunk overhead and caps throughput ~10-15 kbps.
     * Similarly raise the UART TX buffer so upload bursts don't stall.
     * task_stack_size default is 4096 B — raise to 8192 for CMUX overhead.
     * task_priority default is 5 — raise to 10 to reduce scheduling latency
     * vs other tasks (tcpip task runs at 18; keep below that). */
    dte_config.dte_buffer_size                    = 32768;
    dte_config.uart_config.rx_buffer_size         = 32768;
    dte_config.uart_config.tx_buffer_size         = 16384;
    dte_config.task_stack_size                    = 8192;
    dte_config.task_priority                      = 10;

    dte_config.uart_config.port_num   = cfg->modem.uart_port;
    dte_config.uart_config.tx_io_num  = cfg->modem.tx_pin;
    dte_config.uart_config.rx_io_num  = cfg->modem.rx_pin;
    dte_config.uart_config.rts_io_num = cfg->modem.rts_pin;
    dte_config.uart_config.cts_io_num = cfg->modem.cts_pin;

    dte_config.uart_config.flow_control = cfg->hw_flow_control
        ? ESP_MODEM_FLOW_CONTROL_HW : ESP_MODEM_FLOW_CONTROL_NONE;

    if (cfg->baud_rate > 0)
    {
        dte_config.uart_config.baud_rate = cfg->baud_rate;
    }

    if (cfg->use_usb) {
        /* USB CDC-ACM path: use BG95 native USB port as DTE. */
        ESP_LOGI(TAG, "Creating USB CDC-ACM DTE (VID=0x%04X PID=0x%04X iface=%u)",
                 cfg->usb_vid  ? cfg->usb_vid  : LTE_USB_VID_QUECTEL,
                 cfg->usb_pid  ? cfg->usb_pid  : LTE_USB_PID_BG95,
                 cfg->usb_interface ? cfg->usb_interface : LTE_USB_IFACE_MODEM);
        s_dce = lte_pppos_usb_create_dce(
            &dte_config, &dce_config, s_netif_ppp,
            cfg->usb_sel_gpio,
            cfg->usb_sel_level,
            cfg->usb_vid  ? cfg->usb_vid  : LTE_USB_VID_QUECTEL,
            cfg->usb_pid  ? cfg->usb_pid  : LTE_USB_PID_BG95,
            cfg->usb_interface ? cfg->usb_interface : LTE_USB_IFACE_MODEM,
            cfg->connect_timeout_ms);
    } else {
        s_dce = esp_modem_new(&dte_config, &dce_config, s_netif_ppp);
    }
    if (!s_dce)
    {
        ESP_LOGE(TAG, "%s returned NULL",
                 cfg->use_usb ? "lte_pppos_usb_create_dce" : "esp_modem_new");
        lte_inner_stop();
        return ESP_FAIL;
    }

    /* modem_manager already configured and persisted AT+IFC=2,2 when
     * init_modem_manager=true, so skip the redundant DCE-level call. */
    if (cfg->hw_flow_control && !cfg->init_modem_manager)
    {
        (void)esp_modem_set_flow_control(s_dce, 2, 2);
    }

    /* Set PDP context (APN) before attach so the modem knows which APN to
     * use when registering on the data network.  Without this AT+CGATT stays
     * 0 because the modem has no PDP context configured and the APN in
     * dce_config is only applied during the later PPP dial phase. */
    {
        esp_modem_PdpContext_t pdp_ctx = {
            .context_id    = 1,
            .protocol_type = "IP",
            .apn           = cfg->apn,
        };
        esp_err_t pdp_err = esp_modem_set_pdp_context(s_dce, &pdp_ctx);
        if (pdp_err != ESP_OK)
        {
            ESP_LOGW(TAG, "esp_modem_set_pdp_context failed: %s (continuing)", esp_err_to_name(pdp_err));
        }
        else
        {
            ESP_LOGI(TAG, "PDP context set: APN='%s' – modem will register automatically", cfg->apn);
            /* Do NOT send AT+CGATT=0/1 here. On BG95, AT+CGATT=0 triggers a
             * radio stack reset which asserts UART BREAK for ~20 ms, corrupting
             * the esp_modem DTE receive buffer and causing all subsequent AT
             * commands to fail. modem_manager has just power-cycled the modem,
             * so there is no stale registration to clear; AT+CGDCONT alone is
             * sufficient and the modem will attach using the new APN on its own. */
        }
    }

    err = wait_for_cellular_attach(
        s_dce,
        cfg->reg_timeout_ms > 0 ? cfg->reg_timeout_ms : 120000,
        cfg->pin,
        cfg->reboot_on_reg_timeout,
        cfg->reboot_wait_ms);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Cellular not attached: %s", esp_err_to_name(err));
        lte_inner_stop();
        return err;
    }

    /* Choose CMUX (PPP + AT on same UART) or direct DATA mode (PPP only, no AT).
     * Direct mode avoids CMUX framing overhead and can deliver higher throughput. */
    bool use_cmux = cfg->use_cmux;  /* default: false → direct mode unless explicitly set */

    if (use_cmux)
    {
        err = esp_modem_set_mode(s_dce, ESP_MODEM_MODE_CMUX);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_modem_set_mode(CMUX) failed: %s", esp_err_to_name(err));
            lte_inner_stop();
            return err;
        }
        ESP_LOGI(TAG, "CMUX mode active – PPP on data channel, AT commands on control channel");
    }
    else
    {
        err = esp_modem_set_mode(s_dce, ESP_MODEM_MODE_DATA);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_modem_set_mode(DATA) failed: %s", esp_err_to_name(err));
            lte_inner_stop();
            return err;
        }
        ESP_LOGI(TAG, "Direct PPP data mode active (no AT commands available)");
    }

    s_monitor_running = true;
    if (use_cmux)
    {
        if (xTaskCreate(lte_monitor_task, "lte_monitor", 4096, NULL, 4, &s_monitor_task) != pdPASS)
        {
            ESP_LOGW(TAG, "Failed to create lte_monitor task");
            s_monitor_task = NULL;
        }
    }
    else
    {
        ESP_LOGI(TAG, "Monitor task skipped (no AT in direct DATA mode)");
    }

    ESP_LOGI(TAG, "PPPoS dial started (waiting for IP events)...");

    if (cfg->connect_timeout_ms > 0)
    {
        err = lte_upstream_pppos_wait_for_ip(cfg->connect_timeout_ms);
        if (err != ESP_OK)
        {
            return err;
        }
    }

    return ESP_OK;
}

#endif // CONFIG_PPP_SUPPORT

esp_err_t lte_upstream_pppos_start(const lte_upstream_pppos_config_t *cfg)
{
    if (cfg == NULL || cfg->apn == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

#if !CONFIG_PPP_SUPPORT
    (void)cfg;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (s_dce || s_netif_ppp || s_supervisor_task || s_start_in_progress)
    {
        return ESP_ERR_INVALID_STATE;
    }

    s_stop_requested = false;
    s_start_in_progress = true;
    lte_save_config(cfg);

    /* Configure LED_RED_PIN as output, start with LED off (active-low) */
    gpio_reset_pin(LED_RED_PIN);
    gpio_set_direction(LED_RED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_RED_PIN, 1);

    esp_err_t err = lte_inner_start(cfg);
    s_start_in_progress = false;
    if (s_stop_requested)
    {
        ESP_LOGI(TAG, "LTE start aborted by stop request");
        return ESP_ERR_INVALID_STATE;
    }

    if (err != ESP_OK)
    {
        /* lte_inner_stop() was already called inside lte_inner_start on failure.
         * Fall through to start the supervisor so it can retry the connection
         * rather than giving up permanently. */
        ESP_LOGW(TAG, "Initial LTE start failed (%s) – supervisor will retry", esp_err_to_name(err));
    }

    s_supervisor_running = true;
    if (xTaskCreate(lte_supervisor_task, "lte_sup", 8192, NULL, 4, &s_supervisor_task) != pdPASS)
    {
        ESP_LOGW(TAG, "Failed to create lte_supervisor task – no auto-reconnect");
        s_supervisor_task = NULL;
        return err;  /* no supervisor → propagate the inner-start error */
    }

    return ESP_OK;
#endif
}

esp_err_t lte_upstream_pppos_wait_for_ip(uint32_t timeout_ms)
{
#if !CONFIG_PPP_SUPPORT
    (void)timeout_ms;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!s_evt)
    {
        return ESP_ERR_INVALID_STATE;
    }

    EventBits_t bits = xEventGroupWaitBits(
        s_evt,
        BIT_CONNECTED | BIT_DISCONNECTED,
        pdFALSE,
        pdFALSE,
        (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms));

    if (bits & BIT_CONNECTED)
    {
        return ESP_OK;
    }
    if (bits & BIT_DISCONNECTED)
    {
        return ESP_FAIL;
    }

    return ESP_ERR_TIMEOUT;
#endif
}

esp_err_t lte_upstream_pppos_stop(void)
{
#if !CONFIG_PPP_SUPPORT
    return ESP_ERR_NOT_SUPPORTED;
#else
    s_stop_requested = true;

    for (int i = 0; i < 50 && s_start_in_progress; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (s_start_in_progress)
    {
        ESP_LOGW(TAG, "Timed out waiting for LTE startup to stop");
        return ESP_ERR_TIMEOUT;
    }

    /* Signal supervisor to exit and wait up to 4 s before force-killing */
    s_supervisor_running = false;
    for (int i = 0; i < 40 && s_supervisor_task != NULL; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (s_supervisor_task != NULL)
    {
        vTaskDelete(s_supervisor_task);
        s_supervisor_task = NULL;
    }

    lte_inner_stop();
    s_stop_requested = false;
    return ESP_OK;
#endif
}

esp_err_t lte_upstream_pppos_get_status(lte_status_t *out)
{
    if (!out)
    {
        return ESP_ERR_INVALID_ARG;
    }

#if !CONFIG_PPP_SUPPORT
    return ESP_ERR_NOT_SUPPORTED;
#else
    portENTER_CRITICAL(&s_status_mux);
    *out = s_status;
    bool valid = s_status.valid;
    portEXIT_CRITICAL(&s_status_mux);

    return valid ? ESP_OK : ESP_ERR_INVALID_STATE;
#endif
}

esp_err_t lte_upstream_pppos_send_at(const char *cmd, char *resp,
                                     size_t resp_len, uint32_t timeout_ms)
{
#if !CONFIG_PPP_SUPPORT
    (void)cmd; (void)resp; (void)resp_len; (void)timeout_ms;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!s_dce)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (!resp || resp_len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    resp[0] = '\0';
    return esp_modem_at(s_dce, cmd, resp, timeout_ms);
#endif
}

bool lte_upstream_pppos_is_time_synced(void)
{
#if !CONFIG_PPP_SUPPORT
    return false;
#else
    return s_time_synced;
#endif
}

void lte_upstream_pppos_mark_time_synced(void)
{
#if CONFIG_PPP_SUPPORT
    s_time_synced = true;
#endif
}

/* -----------------------------------------------------------------------
 * PSM T3412/T3324 timer decoder (3GPP TS 24.008 Table 10.5.163a/172)
 * Input : 8-char binary string e.g. "00100001"
 * Returns: seconds, or -1 if deactivated.
 * ----------------------------------------------------------------------- */
static int32_t psm_decode_timer(const char *bits8)
{
    if (!bits8 || strlen(bits8) < 8) { return -1; }
    int unit_bits = ((bits8[0]-'0')<<2)|((bits8[1]-'0')<<1)|(bits8[2]-'0');
    int value = 0;
    for (int i = 3; i < 8; i++) { value = (value << 1) | (bits8[i] - '0'); }
    if (unit_bits == 7) { return -1; } /* deactivated */
    static const int32_t t3412_sec[] = { 10*60, 3600, 10*3600, 2, 30, 60, 320*3600, -1 };
    return t3412_sec[unit_bits] * value;
}

static void psm_print_duration(int32_t secs)
{
    if (secs < 0)  { printf("deactivated"); return; }
    if (secs == 0) { printf("0 s"); return; }
    int h = secs / 3600;  secs %= 3600;
    int m = secs / 60;    secs %= 60;
    if (h) printf("%dh ", h);
    if (m) printf("%dm ", m);
    if (secs || (!h && !m)) printf("%ds", (int)secs);
}

static int cmd_lte(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("Usage: lte -s | -r | -n | -p | -o | -i | -q | -j\n");
        printf("  -s  full status\n");
        printf("  -r  signal quality (RSSI/BER)\n");
        printf("  -n  network attachment\n");
        printf("  -p  PPP connection status\n");
        printf("  -o  operator name\n");
        printf("  -i  current IP address\n");
        printf("  -q  radio quality + PSM/eDRX power-saving state\n");
        printf("  -j  all parameters as JSON\n");
        return 0;
    }

    const char *flag = argv[1];

    lte_status_t st = {0};
    esp_err_t err = lte_upstream_pppos_get_status(&st);

    if (err == ESP_ERR_NOT_SUPPORTED)
    {
        printf("LTE: PPP support not compiled in\n");
        return 1;
    }

    /* -i works before the monitor has run; uses event bits directly */
    if (strcmp(flag, "-i") == 0)
    {
        if (!s_evt)
        {
            printf("No IP (LTE not started)\n");
            return 0;
        }
        EventBits_t bits = xEventGroupGetBits(s_evt);
        if ((bits & BIT_CONNECTED) && !(bits & BIT_DISCONNECTED))
        {
            printf("IP: " IPSTR "\n", IP2STR(&st.current_ip));
        }
        else
        {
            const char *reason =
                (bits & BIT_NETWORK_DETACHED) ? "network detached" :
                (bits & BIT_DISCONNECTED)     ? "PPP disconnected" :
                                                "connecting";
            printf("No IP (%s)\n", reason);
        }
        return 0;
    }

    /* -j works at any stage – queries modem directly for debug info */
    if (strcmp(flag, "-j") == 0)
    {
        /* Strip double-quote characters from a string in-place so raw AT
         * responses don't produce ugly escaped quotes in the JSON output. */
        #define STRIP_QUOTES(s) do { \
            char *_r = (s), *_w = (s); \
            while (*_r) { if (*_r != '"') *_w++ = *_r; _r++; } \
            *_w = '\0'; \
        } while(0)

        cJSON *root = cJSON_CreateObject();
        if (!root)
        {
            printf("{\"error\":\"out of memory\"}\n");
            return 1;
        }

        /* --- stage / state --- */
        const char *stage = "not_started";
        bool has_ip = false;
        if (s_evt)
        {
            EventBits_t bits = xEventGroupGetBits(s_evt);
            has_ip = (bits & BIT_CONNECTED) && !(bits & BIT_DISCONNECTED);
            if (has_ip)
                stage = "connected";
            else if (bits & BIT_DISCONNECTED)
                stage = "ppp_disconnected";
            else if (bits & BIT_NETWORK_DETACHED)
                stage = "network_detached";
            else
                stage = "connecting";
        }
        cJSON_AddStringToObject(root, "stage", stage);

        /* --- cached monitor status (may be all-zero if monitor hasn't run) --- */
        cJSON_AddBoolToObject(root, "status_valid", st.valid);
        cJSON_AddNumberToObject(root, "rssi", st.rssi);
        cJSON_AddNumberToObject(root, "rssi_dbm", st.rssi_dbm);
        cJSON_AddNumberToObject(root, "ber", st.ber);
        cJSON_AddBoolToObject(root, "attached", st.attached);
        cJSON_AddBoolToObject(root, "ppp_connected", st.ppp_connected);

        char ip_str[16];
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&st.current_ip));
        cJSON_AddStringToObject(root, "ip", ip_str);

        cJSON_AddStringToObject(root, "operator", st.operator_name);
        cJSON_AddNumberToObject(root, "operator_act", st.operator_act);
        cJSON_AddStringToObject(root, "network_type",
                                st.network_type[0] ? st.network_type : "");

        /* --- live AT queries (work any time the DCE exists, even before PPP) --- */
#if CONFIG_PPP_SUPPORT
        cJSON_AddBoolToObject(root, "modem_available", s_dce != NULL);
        if (s_dce)
        {
            char resp[256];

            /* SIM IMSI */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+CIMI", resp, 2000) == ESP_OK && resp[0])
                cJSON_AddStringToObject(root, "imsi", resp);
            else
                cJSON_AddNullToObject(root, "imsi");

            /* SIM ICCID */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+CCID", resp, 2000) == ESP_OK && resp[0])
            {
                /* Strip +CCID: prefix if present */
                char *p = resp;
                if (strncmp(p, "+CCID: ", 7) == 0) p += 7;
                while (*p == ' ') p++;
                cJSON_AddStringToObject(root, "iccid", p);
            }
            else
                cJSON_AddNullToObject(root, "iccid");

            /* IMEI */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+GSN", resp, 2000) == ESP_OK && resp[0])
                cJSON_AddStringToObject(root, "imei", resp);
            else
                cJSON_AddNullToObject(root, "imei");

            /* Modem model */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+CGMM", resp, 2000) == ESP_OK && resp[0])
                cJSON_AddStringToObject(root, "modem_model", resp);

            /* Firmware version */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+QGMR", resp, 2000) == ESP_OK && resp[0])
                cJSON_AddStringToObject(root, "modem_fw", resp);

            /* SIM PIN status */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+CPIN?", resp, 2000) == ESP_OK && resp[0])
            {
                char *p = resp;
                if (strncmp(p, "+CPIN: ", 7) == 0) p += 7;
                cJSON_AddStringToObject(root, "sim_status", p);
            }
            else
                cJSON_AddNullToObject(root, "sim_status");

            /* Registration state: CEREG */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+CEREG?", resp, 2000) == ESP_OK && resp[0])
            {
                char *p = resp;
                while (*p == '\r' || *p == '\n' || *p == ' ') p++;
                cJSON_AddStringToObject(root, "cereg", p);
            }

            /* Registration state: CREG */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+CREG?", resp, 2000) == ESP_OK && resp[0])
            {
                char *p = resp;
                while (*p == '\r' || *p == '\n' || *p == ' ') p++;
                cJSON_AddStringToObject(root, "creg", p);
            }

            /* Signal quality */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+CSQ", resp, 2000) == ESP_OK && resp[0])
            {
                char *p = resp;
                while (*p == '\r' || *p == '\n' || *p == ' ') p++;
                cJSON_AddStringToObject(root, "csq", p);
            }

            /* QCSQ (extended signal) */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+QCSQ", resp, 3000) == ESP_OK)
            {
                int rsrp = 0, sinr = 0, rsrq = 0;
                if (sscanf(resp, "+QCSQ: \"%*[^\"]\",%*d,%d,%d,%d",
                           &rsrp, &sinr, &rsrq) == 3)
                {
                    cJSON_AddNumberToObject(root, "rsrp", rsrp);
                    cJSON_AddNumberToObject(root, "sinr_raw", sinr);
                    cJSON_AddNumberToObject(root, "sinr_db", sinr * 0.2);
                    cJSON_AddNumberToObject(root, "rsrq", rsrq);
                }
            }

            /* QNWINFO (network type) */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+QNWINFO", resp, 2000) == ESP_OK && resp[0])
            {
                char *p = resp;
                while (*p == '\r' || *p == '\n' || *p == ' ') p++;
                STRIP_QUOTES(p);
                cJSON_AddStringToObject(root, "qnwinfo", p);
            }

            /* Serving cell */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+QENG=\"servingcell\"", resp, 3000) == ESP_OK && resp[0])
            {
                char *p = resp;
                while (*p == '\r' || *p == '\n' || *p == ' ') p++;
                STRIP_QUOTES(p);
                cJSON_AddStringToObject(root, "serving_cell", p);
            }

            /* PDP context */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+CGDCONT?", resp, 2000) == ESP_OK && resp[0])
            {
                char *p = resp;
                while (*p == '\r' || *p == '\n' || *p == ' ') p++;
                STRIP_QUOTES(p);
                cJSON_AddStringToObject(root, "pdp_context", p);
            }

            /* Operator */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+COPS?", resp, 2000) == ESP_OK && resp[0])
            {
                char *p = resp;
                while (*p == '\r' || *p == '\n' || *p == ' ') p++;
                STRIP_QUOTES(p);
                cJSON_AddStringToObject(root, "cops", p);
            }

            /* PSM */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+CPSMS?", resp, 3000) == ESP_OK)
            {
                int mode = -1;
                char tau_bits[16] = {0}, act_bits[16] = {0};
                sscanf(resp, "+CPSMS: %d,,\"%*[^\"]\",\"%15[^\"]\",\"%15[^\"]\"",
                       &mode, tau_bits, act_bits);
                if (mode < 0) sscanf(resp, "+CPSMS: %d", &mode);
                cJSON_AddBoolToObject(root, "psm_enabled", mode == 1);
                if (tau_bits[0])
                    cJSON_AddNumberToObject(root, "psm_tau_s",
                                            psm_decode_timer(tau_bits));
                if (act_bits[0])
                    cJSON_AddNumberToObject(root, "psm_active_s",
                                            psm_decode_timer(act_bits));
            }

            /* eDRX */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+CEDRXRDP", resp, 3000) == ESP_OK)
            {
                int act_type = -1;
                char nw_edrx[8] = {0};
                bool edrx_full = (sscanf(resp,
                    "+CEDRXRDP: %d,\"%*[^\"]\",\"%7[^\"]\",\"%*[^\"]\"",
                    &act_type, nw_edrx) == 2);
                if (!edrx_full)
                    sscanf(resp, "+CEDRXRDP: %d", &act_type);
                if (edrx_full && nw_edrx[0] && strcmp(nw_edrx, "0000") != 0)
                {
                    int n = 0;
                    for (int i = 0; i < 4; i++)
                        n = (n << 1) | (nw_edrx[i] - '0');
                    cJSON_AddNumberToObject(root, "edrx_cycle_s",
                                            (double)(1 << n) * 10.24);
                }
                else
                {
                    cJSON_AddNullToObject(root, "edrx_cycle_s");
                }
            }

            /* Band config */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+QCFG=\"band\"", resp, 2000) == ESP_OK && resp[0])
            {
                char *p = resp;
                while (*p == '\r' || *p == '\n' || *p == ' ') p++;
                STRIP_QUOTES(p);
                cJSON_AddStringToObject(root, "band_cfg", p);
            }

            /* Scan mode */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+QCFG=\"nwscanmode\"", resp, 2000) == ESP_OK && resp[0])
            {
                char *p = resp;
                while (*p == '\r' || *p == '\n' || *p == ' ') p++;
                STRIP_QUOTES(p);
                cJSON_AddStringToObject(root, "nwscanmode", p);
            }

            /* IoT operation mode */
            memset(resp, 0, sizeof(resp));
            if (esp_modem_at(s_dce, "AT+QCFG=\"iotopmode\"", resp, 2000) == ESP_OK && resp[0])
            {
                char *p = resp;
                while (*p == '\r' || *p == '\n' || *p == ' ') p++;
                STRIP_QUOTES(p);
                cJSON_AddStringToObject(root, "iotopmode", p);
            }
        }
#else
        cJSON_AddBoolToObject(root, "modem_available", false);
#endif /* CONFIG_PPP_SUPPORT */

        #undef STRIP_QUOTES

        char *json_str = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);
        if (json_str)
        {
            printf("%s\n", json_str);
            cJSON_free(json_str);
        }
        else
        {
            printf("{\"error\":\"json print failed\"}\n");
        }
        return 0;
    }

    if (err == ESP_ERR_INVALID_STATE || !st.valid)
    {
        if (!s_evt)
        {
            printf("LTE: not started\n");
        }
        else
        {
            EventBits_t bits = xEventGroupGetBits(s_evt);
            const char *ppp =
                (bits & BIT_CONNECTED)    ? "connected" :
                (bits & BIT_DISCONNECTED) ? "disconnected" :
                                            "connecting";
            const char *net =
                (bits & BIT_NETWORK_DETACHED) ? ", network detached" : "";
            printf("LTE: starting up (PPP %s%s) – signal data available after first monitor poll (~15s)\n",
                   ppp, net);
        }
        return 1;
    }

    if (strcmp(flag, "-r") == 0)
    {
        if (st.rssi == 99)
        {
            printf("RSSI: unknown\n");
        }
        else
        {
            printf("RSSI: %d (%d dBm), BER: %d\n", st.rssi, st.rssi_dbm, st.ber);
        }
        return 0;
    }

    if (strcmp(flag, "-n") == 0)
    {
        printf("Network: %s\n", st.attached ? "attached" : "detached");
        return 0;
    }

    if (strcmp(flag, "-p") == 0)
    {
        printf("PPP: %s\n", st.ppp_connected ? "connected" : "disconnected");
        return 0;
    }

    if (strcmp(flag, "-o") == 0)
    {
        if (st.operator_name[0] != '\0')
        {
            printf("Operator: %s (act=%d)\n", st.operator_name, st.operator_act);
        }
        else
        {
            printf("Operator: (not yet queried)\n");
        }
        return 0;
    }

    if (strcmp(flag, "-s") == 0)
    {
        printf("--- LTE Status ---\n");

        if (st.rssi == 99)
        {
            printf("  Signal   : unknown\n");
        }
        else
        {
            printf("  Signal   : RSSI=%d (%d dBm)  BER=%d\n", st.rssi, st.rssi_dbm, st.ber);
        }

        printf("  Network  : %s\n", st.attached ? "attached" : "detached");
        printf("  Net type : %s\n", st.network_type[0] ? st.network_type : "(not yet queried)");
        printf("  PPP      : %s\n", st.ppp_connected ? "connected" : "disconnected");

        if (st.operator_name[0] != '\0')
        {
            printf("  Operator : %s  (act=%d)\n", st.operator_name, st.operator_act);
        }
        else
        {
            printf("  Operator : (not yet queried)\n");
        }

        return 0;
    }

    if (strcmp(flag, "-q") == 0)
    {
        if (!s_dce)
        {
            printf("lte -q: modem not running\n");
            return 1;
        }

        char resp[256];

        /* --- Extended signal quality --- */
        printf("--- Radio Quality ---\n");
        memset(resp, 0, sizeof(resp));
        if (esp_modem_at(s_dce, "AT+QCSQ", resp, 3000) == ESP_OK)
        {
            char *p = resp; while (*p == '\r' || *p == '\n' || *p == ' ') p++;
            printf("  QCSQ     : %s\n", p);
            int rsrp = 0, sinr = 0, rsrq = 0;
            if (sscanf(resp, "+QCSQ: \"%*[^\"]\",%*d,%d,%d,%d", &rsrp, &sinr, &rsrq) == 3)
            {
                printf("  RSRP     : %d dBm    (good > -100)\n", rsrp);
                /* BG95 SINR unit = 0.2 dB; actual dB = raw * 0.2 */
                printf("  SINR     : %.1f dB  (good > 0, raw=%d)\n", sinr * 0.2f, sinr);
                printf("  RSRQ     : %d dB    (good > -15)\n",  rsrq);
                if (rsrp < -110) printf("  WARNING  : RSRP very low - poor coverage may limit speed\n");
                if (sinr < 0)    printf("  WARNING  : Negative SINR - heavy interference\n");
            }
        }
        else
        {
            printf("  QCSQ     : (not supported)\n");
        }

        memset(resp, 0, sizeof(resp));
        if (esp_modem_at(s_dce, "AT+QENG=\"servingcell\"", resp, 3000) == ESP_OK)
        {
            char *p = resp; while (*p == '\r' || *p == '\n' || *p == ' ') p++;
            printf("  QENG     : %s\n", p);
        }

        /* --- PSM --- */
        printf("\n--- PSM (Power Saving Mode) ---\n");
        memset(resp, 0, sizeof(resp));
        if (esp_modem_at(s_dce, "AT+CPSMS?", resp, 3000) == ESP_OK)
        {
            char *p = resp; while (*p == '\r' || *p == '\n' || *p == ' ') p++;
            printf("  CPSMS    : %s\n", p);
            int mode = -1;
            char tau_bits[16] = {0}, act_bits[16] = {0};
            sscanf(resp, "+CPSMS: %d,,\"%*[^\"]\",\"%15[^\"]\",\"%15[^\"]\"",
                   &mode, tau_bits, act_bits);
            if (mode < 0) sscanf(resp, "+CPSMS: %d", &mode);
            printf("  Mode     : %s\n", mode == 1 ? "ENABLED" :
                                        mode == 0 ? "disabled" : "unknown");
            if (tau_bits[0])
            {
                int32_t tau_s = psm_decode_timer(tau_bits);
                printf("  TAU T3412: %s -> ", tau_bits);
                psm_print_duration(tau_s);
                printf(" (modem sleeps between these periods)\n");
            }
            if (act_bits[0])
            {
                int32_t act_s = psm_decode_timer(act_bits);
                printf("  Active T3324: %s -> ", act_bits);
                psm_print_duration(act_s);
                printf(" (modem stays awake after TAU for this long)\n");
                if (act_s >= 0 && act_s < 10)
                    printf("  WARNING  : Active time < 10s - PSM may kill throughput on first TX\n");
            }
            if (mode == 1)
                printf("  TIP      : Disable PSM with AT+CPSMS=0 to maximise throughput\n");
        }
        else
        {
            printf("  CPSMS    : (not supported)\n");
        }

        /* --- eDRX --- */
        printf("\n--- eDRX (extended Discontinuous Reception) ---\n");
        memset(resp, 0, sizeof(resp));
        if (esp_modem_at(s_dce, "AT+CEDRXRDP", resp, 3000) == ESP_OK)
        {
            char *p = resp; while (*p == '\r' || *p == '\n' || *p == ' ') p++;
            printf("  CEDRXRDP : %s\n", p);
            int act_type = -1;
            char nw_edrx[8] = {0}, ptw[8] = {0};
            /* Try full 4-field form first; fall back to single-int form (+CEDRXRDP: 0) */
            bool edrx_full = (sscanf(resp, "+CEDRXRDP: %d,\"%*[^\"]\",\"%7[^\"]\",\"%7[^\"]\"",
                                     &act_type, nw_edrx, ptw) == 3);
            if (!edrx_full)
            {
                sscanf(resp, "+CEDRXRDP: %d", &act_type);
            }
            if (!edrx_full || nw_edrx[0] == '\0' || strcmp(nw_edrx, "0000") == 0)
            {
                printf("  eDRX     : not negotiated / disabled by network\n");
            }
            else
            {
                int n = 0;
                for (int i = 0; i < 4; i++) n = (n << 1) | (nw_edrx[i] - '0');
                float cycle_s = (float)(1 << n) * 10.24f;
                printf("  NW eDRX cycle : %.1f s\n", cycle_s);
                if (cycle_s > 20.0f)
                    printf("  WARNING  : Long eDRX cycle (%.0fs) adds latency/stall on first packet\n",
                           cycle_s);
            }
        }
        else
        {
            printf("  CEDRXRDP : (not supported)\n");
        }

        return 0;
    }

    printf("lte: unknown flag '%s'. Use -s, -r, -n, -p, -o, -i, -q or -j\n", flag);
    return 1;
}

void lte_upstream_pppos_console_register(void)
{
    static bool registered = false;
    if (registered)
    {
        return;
    }
    registered = true;

    const esp_console_cmd_t cmd = {
        .command = "lte",
        .help    = "LTE modem status. Flags: -s full status, -r signal/RSSI, -n network attachment, -p PPP status, -o operator, -i IP address, -q radio quality/PSM/eDRX, -j JSON",
        .hint    = "-s|-r|-n|-p|-o|-i|-q|-j",
        .func    = &cmd_lte,
    };
    (void)esp_console_cmd_register(&cmd);
}
