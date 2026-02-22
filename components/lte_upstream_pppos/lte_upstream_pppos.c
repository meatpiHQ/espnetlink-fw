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

#include "usb_dev_ethernet.h"
#include "esp_timer.h"

static const char *TAG = "lte_pppos";

static EventGroupHandle_t s_evt;
static const EventBits_t BIT_CONNECTED = BIT0;
static const EventBits_t BIT_DISCONNECTED = BIT1;

#if CONFIG_PPP_SUPPORT
#include "esp_modem_api.h"

static esp_modem_dce_t *s_dce;
static esp_netif_t *s_netif_ppp;
static bool s_enable_usb_share;
static volatile bool s_monitor_running;
static TaskHandle_t s_monitor_task;

static portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;
static lte_status_t s_status;   /* protected by s_status_mux */

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

static esp_err_t wait_for_cellular_attach(esp_modem_dce_t *dce, uint32_t timeout_ms)
{
    const int64_t deadline = esp_timer_get_time() / 1000 + timeout_ms;
    bool pin_ok = true;
    if (esp_modem_read_pin(dce, &pin_ok) == ESP_OK && !pin_ok)
    {
        ESP_LOGE(TAG, "SIM requires PIN (CPIN not ready)");
        return ESP_ERR_INVALID_STATE;
    }

    while ((esp_timer_get_time() / 1000) < deadline)
    {
        int rssi = 0, ber = 0;
        (void)esp_modem_get_signal_quality(dce, &rssi, &ber);

        int attached = 0;
        esp_err_t err = esp_modem_get_network_attachment_state(dce, &attached);
        if (err == ESP_OK && attached == 1)
        {
            ESP_LOGI(TAG, "Cellular attached (rssi=%d ber=%d)", rssi, ber);
            return ESP_OK;
        }

        ESP_LOGW(TAG, "Waiting for cellular attach... (rssi=%d ber=%d)", rssi, ber);
        // Best-effort request attach in case it's detached
        (void)esp_modem_set_network_attachment_state(dce, 1);
        vTaskDelay(pdMS_TO_TICKS(2000));
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

        if (s_enable_usb_share)
        {
            esp_err_t share_err = usb_dev_ethernet_enable_sharing(dns_main_addr);
            if (share_err != ESP_OK)
            {
                ESP_LOGW(TAG, "usb_dev_ethernet_enable_sharing failed: %s", esp_err_to_name(share_err));
            }
        }

        if (s_evt)
        {
            xEventGroupSetBits(s_evt, BIT_CONNECTED);
        }
    }
    else if (event_id == IP_EVENT_PPP_LOST_IP)
    {
        ESP_LOGW(TAG, "PPP LOST IP");
        if (s_evt)
        {
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

        int attached = 0;
        if (esp_modem_get_network_attachment_state(s_dce, &attached) == ESP_OK)
        {
            ESP_LOGI(TAG_MON, "Network: %s", attached ? "attached" : "detached");
        }

        bool ppp_up = false;
        if (s_evt)
        {
            EventBits_t bits = xEventGroupGetBits(s_evt);
            ppp_up = (bits & BIT_CONNECTED) && !(bits & BIT_DISCONNECTED);
            ESP_LOGI(TAG_MON, "PPP: %s", ppp_up ? "connected" : "disconnected");
        }

        /* Operator name – query every 4th poll (~60 s) */
        if (poll_count % 4 == 1)
        {
            if (esp_modem_get_operator_name(s_dce, op_name, &op_act) == ESP_OK)
            {
                ESP_LOGI(TAG_MON, "Operator: \"%s\" (act=%d)", op_name, op_act);
            }
        }

        /* --- Commit to shared status struct --- */
        portENTER_CRITICAL(&s_status_mux);
        s_status.valid         = true;
        s_status.rssi          = rssi;
        s_status.rssi_dbm      = (rssi < 99) ? (rssi * 2 - 113) : 0;
        s_status.ber           = ber;
        s_status.attached      = (attached == 1);
        s_status.ppp_connected = ppp_up;
        s_status.operator_act  = op_act;
        /* op_name only changes every 4 polls; copy it every time (cheap) */
        memcpy(s_status.operator_name, op_name, sizeof(s_status.operator_name));
        portEXIT_CRITICAL(&s_status_mux);
    }

    s_monitor_task = NULL;
    vTaskDelete(NULL);
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

    xEventGroupClearBits(s_evt, BIT_CONNECTED | BIT_DISCONNECTED);

    err = register_handlers();
    if (err != ESP_OK)
    {
        return err;
    }

    esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();
    s_netif_ppp = esp_netif_new(&netif_ppp_config);
    if (!s_netif_ppp)
    {
        unregister_handlers();
        return ESP_ERR_NO_MEM;
    }

    esp_netif_ppp_config_t ppp_events = {
        .ppp_phase_event_enabled = true,
        .ppp_error_event_enabled = true,
#ifdef CONFIG_LWIP_ENABLE_LCP_ECHO
        .ppp_lcp_echo_disabled = false,
#endif
    };
    (void)esp_netif_ppp_set_params(s_netif_ppp, &ppp_events);

    if (cfg->user && cfg->pass)
    {
#if CONFIG_LWIP_PPP_PAP_SUPPORT
        (void)esp_netif_ppp_set_auth(s_netif_ppp, NETIF_PPP_AUTHTYPE_PAP, cfg->user, cfg->pass);
#else
        ESP_LOGW(TAG, "PPP user/pass provided but PAP support is disabled (CONFIG_LWIP_PPP_PAP_SUPPORT=n). Skipping auth.");
#endif
    }

    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(cfg->apn);
    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();

    dte_config.uart_config.port_num = cfg->modem.uart_port;
    dte_config.uart_config.tx_io_num = cfg->modem.tx_pin;
    dte_config.uart_config.rx_io_num = cfg->modem.rx_pin;
    dte_config.uart_config.rts_io_num = cfg->modem.rts_pin;
    dte_config.uart_config.cts_io_num = cfg->modem.cts_pin;

    dte_config.uart_config.flow_control = cfg->hw_flow_control ? ESP_MODEM_FLOW_CONTROL_HW : ESP_MODEM_FLOW_CONTROL_NONE;

    if (cfg->baud_rate > 0)
    {
        dte_config.uart_config.baud_rate = cfg->baud_rate;
    }

    s_dce = esp_modem_new(&dte_config, &dce_config, s_netif_ppp);
    if (!s_dce)
    {
        ESP_LOGE(TAG, "esp_modem_new returned NULL");
        esp_netif_destroy(s_netif_ppp);
        s_netif_ppp = NULL;
        unregister_handlers();
        return ESP_FAIL;
    }

    if (cfg->hw_flow_control)
    {
        (void)esp_modem_set_flow_control(s_dce, 2, 2);
    }

    // Wait for network attach before starting PPP (otherwise most carriers drop immediately).
    err = wait_for_cellular_attach(s_dce, 60000);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Cellular not attached: %s", esp_err_to_name(err));
        (void)lte_upstream_pppos_stop();
        return err;
    }

    err = esp_modem_set_mode(s_dce, ESP_MODEM_MODE_CMUX);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_modem_set_mode(CMUX) failed: %s", esp_err_to_name(err));
        (void)lte_upstream_pppos_stop();
        return err;
    }

    ESP_LOGI(TAG, "CMUX mode active – PPP on data channel, AT commands on control channel");

    s_monitor_running = true;
    if (xTaskCreate(lte_monitor_task, "lte_monitor", 4096, NULL, 4, &s_monitor_task) != pdPASS)
    {
        ESP_LOGW(TAG, "Failed to create lte_monitor task");
        s_monitor_task = NULL;
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
    s_enable_usb_share = false;

    /* Stop monitor task before destroying DCE */
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

static int cmd_lte(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("Usage: lte -s | -r | -n\n");
        printf("  -s  full status\n");
        printf("  -r  signal quality (RSSI/BER)\n");
        printf("  -n  network attachment\n");
        return 0;
    }

    lte_status_t st = {0};
    esp_err_t err = lte_upstream_pppos_get_status(&st);

    if (err == ESP_ERR_NOT_SUPPORTED)
    {
        printf("LTE: PPP support not compiled in\n");
        return 1;
    }
    if (err == ESP_ERR_INVALID_STATE || !st.valid)
    {
        printf("LTE: no data yet (monitor task not started or first poll pending)\n");
        return 1;
    }

    const char *flag = argv[1];

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

    printf("lte: unknown flag '%s'. Use -s, -r, or -n\n", flag);
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
        .help    = "LTE modem status. Flags: -s full status, -r signal/RSSI, -n network attachment",
        .hint    = "-s|-r|-n",
        .func    = &cmd_lte,
    };
    (void)esp_console_cmd_register(&cmd);
}
