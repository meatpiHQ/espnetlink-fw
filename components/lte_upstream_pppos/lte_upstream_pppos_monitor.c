/**
 * @file lte_upstream_pppos_monitor.c
 * @brief Background health tasks: 15 s status monitor (signal / registration /
 *        operator polls), the reconnect supervisor and modem time sync.
 *
 * Moved verbatim from lte_upstream_pppos.c (§1 split), together with the
 * §2 static TCB/PSRAM-stack creation blocks (now lte_monitor_start() /
 * lte_supervisor_start(), called from the engine's original call sites).
 * Shared engine state comes from lte_upstream_pppos_private.h.
 */

#include <string.h>
#include <stdio.h>

#include <time.h>
#include <sys/time.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"

#include "lte_upstream_pppos.h"

#include "lte_upstream_pppos_private.h"

#if CONFIG_PPP_SUPPORT
#include "esp_modem_api.h"

static const char *TAG = "lte_upstream_pppos"; /* §10: TAG == component dir */

/* -----------------------------------------------------------------------
 * System time synchronisation from modem (AT+QLTS=2)
 * ----------------------------------------------------------------------- */

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

void lte_time_sync_task(void *arg)
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

static void lte_monitor_task(void *arg)
{
    (void)arg;
    /* §10: one TAG per component; periodic poll results log at DEBUG (the
     * 15 s status loop must not flood INFO — transitions log elsewhere). */
    static const char *TAG_MON = "lte_upstream_pppos";
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
            ESP_LOGD(TAG_MON, "Signal: RSSI=%d (%d dBm), BER=%d", rssi, rssi_dbm, ber);
        }
        else
        {
            ESP_LOGD(TAG_MON, "Signal quality query failed");
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
            ESP_LOGD(TAG_MON, "Network: %s (CEREG state=%d)",
                     net_registered ? "registered" : "not registered", reg_state);
        }
        else
        {
            /* CEREG unavailable – fall back to AT+CGATT? */
            int attached = 0;
            if (esp_modem_get_network_attachment_state(s_dce, &attached) == ESP_OK)
            {
                net_registered = (attached == 1);
                ESP_LOGD(TAG_MON, "Network: %s (CGATT)", net_registered ? "attached" : "detached");
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
            ESP_LOGD(TAG_MON, "PPP: %s", ppp_up ? "connected" : "disconnected");
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

/**
 * @brief Create the LTE status monitor task (creation block moved verbatim
 *        from lte_inner_start; the caller stores the handle in s_monitor_task).
 */
TaskHandle_t lte_monitor_start(void)
{
    /* §2: static stack in PSRAM (AT status polls only — no flash writes). */
    static StaticTask_t s_monitor_tcb;
    static StackType_t s_monitor_stack[4096] EXT_RAM_BSS_ATTR;

    return xTaskCreateStatic(lte_monitor_task, "lte_monitor",
                             4096, NULL, 4, s_monitor_stack,
                             &s_monitor_tcb);
}

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

/**
 * @brief Create the reconnect supervisor task (creation block moved verbatim
 *        from lte_upstream_pppos_start; the caller stores the handle in
 *        s_supervisor_task).
 */
TaskHandle_t lte_supervisor_start(void)
{
    /* §2: static stack in PSRAM (start/stop orchestration — no flash). */
    static StaticTask_t s_supervisor_tcb;
    static StackType_t s_supervisor_stack[8192] EXT_RAM_BSS_ATTR;

    return xTaskCreateStatic(lte_supervisor_task, "lte_sup",
                             8192, NULL, 4, s_supervisor_stack,
                             &s_supervisor_tcb);
}

#endif /* CONFIG_PPP_SUPPORT */
