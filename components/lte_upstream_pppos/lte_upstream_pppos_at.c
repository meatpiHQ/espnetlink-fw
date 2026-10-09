/**
 * @file lte_upstream_pppos_at.c
 * @brief Modem AT setup before dial: SIM readiness (AT+CIMI), PIN entry,
 *        QCFG radio profile (iotopmode/nwscanmode), PSM/eDRX disable,
 *        SIM/PDP/band diagnostics and the LTE registration poll — plus
 *        the public raw-AT passthrough lte_upstream_pppos_send_at().
 *
 * Moved verbatim from lte_upstream_pppos.c (§1 split). Shared engine
 * state comes from lte_upstream_pppos_private.h.
 */

#include <string.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "lte_upstream_pppos.h"

#include "lte_upstream_pppos_private.h"

#if CONFIG_PPP_SUPPORT
#include "esp_modem_api.h"

static const char *TAG = "lte_upstream_pppos"; /* §10: TAG == component dir */

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

esp_err_t wait_for_cellular_attach(esp_modem_dce_t *dce, uint32_t timeout_ms,
                                           const char *pin,
                                           bool reboot_on_timeout,
                                           uint32_t reboot_wait_ms)
{
    /* Wait for SIM application to be fully initialised before polling
     * registration.  Allow up to 30 s; deduct the wait from the overall
     * registration deadline so we don't extend the total timeout. */
    const int64_t t_start = esp_timer_get_time() / 1000;
    const bool sim_ready = wait_for_sim_ready(dce, 30000) == ESP_OK;
    const uint32_t sim_wait_elapsed = (uint32_t)(esp_timer_get_time() / 1000 - t_start);

    /* what the SIM check found, for the status surfaces (a locked SIM
     * answers AT+CPIN? but no AT+CIMI: the PIN read below tells it from a
     * missing one) */
    bool pin_ok = true;
    const bool pin_known = esp_modem_read_pin(dce, &pin_ok) == ESP_OK;

    lte_sim_note(sim_ready ? LTE_SIM_READY
                           : (pin_known && !pin_ok) ? LTE_SIM_PIN
                                                    : LTE_SIM_MISSING);

    if (sim_wait_elapsed < timeout_ms)
    {
        timeout_ms -= sim_wait_elapsed;
    }
    else
    {
        return ESP_ERR_TIMEOUT;
    }

    const int64_t deadline = esp_timer_get_time() / 1000 + timeout_ms;
    if (pin_known && !pin_ok)
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
            lte_sim_note(LTE_SIM_READY);
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

#endif /* CONFIG_PPP_SUPPORT */

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
