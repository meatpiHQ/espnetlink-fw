/**
 * @file lte_upstream_pppos_cli.c
 * @brief `lte` console command: status/signal/network/PPP/operator/IP
 *        summaries, radio-quality + PSM/eDRX report and the JSON dump.
 *
 * Moved verbatim from lte_upstream_pppos.c (§1 split). Engine state it
 * reads (event group, DCE handle) comes from lte_upstream_pppos_private.h.
 * No logging here, so no TAG (§10: the component TAG lives in the engine).
 */

#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_console.h"
#include "cmdline_manager.h"
#include "esp_err.h"
#include "esp_netif.h"

#include "lte_upstream_pppos.h"
#include "cJSON.h"

#include "lte_upstream_pppos_private.h"

#if CONFIG_PPP_SUPPORT
#include "esp_modem_api.h"
#endif

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

        /* --- stage / state (the one spelling GET /api/lte uses too) --- */
        cJSON_AddStringToObject(root, "stage", lte_upstream_pppos_stage_str());
        /* the SIM as the bring-up found it (sim_status below is the live
         * AT+CPIN? answer; this one is also there without a console) */
        cJSON_AddStringToObject(root, "sim", lte_upstream_pppos_sim_str(
                                                 lte_upstream_pppos_sim_state()));

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
    (void)cmdline_manager_register(&cmd);
}
