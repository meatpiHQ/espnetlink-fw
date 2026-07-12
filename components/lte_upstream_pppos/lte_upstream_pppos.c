#include <string.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

#include "esp_attr.h"
#include "esp_console.h"
#include "cmdline_manager.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"

#include "lte_upstream_pppos.h"
#include "lte_upstream_pppos_usb.h"

#include "hw_config.h"
#include "driver/gpio.h"
#include "dev_status_manager.h"
#include "usb_dev_ethernet.h"
#include "esp_timer.h"
#include "cJSON.h"

#include <time.h>
#include <sys/time.h>

#include "lte_upstream_pppos_private.h"

static const char *TAG = "lte_upstream_pppos";

EventGroupHandle_t s_evt;
const EventBits_t BIT_CONNECTED        = BIT0;
const EventBits_t BIT_DISCONNECTED     = BIT1;
const EventBits_t BIT_NETWORK_DETACHED = BIT2;

#if CONFIG_PPP_SUPPORT
#include "esp_modem_api.h"

esp_modem_dce_t *s_dce;
static esp_netif_t *s_netif_ppp;
static bool s_enable_usb_share;
volatile bool s_monitor_running;
TaskHandle_t s_monitor_task;

portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;
lte_status_t s_status;   /* protected by s_status_mux */

/* --- reconnect supervisor --- */
TaskHandle_t s_supervisor_task = NULL;
volatile bool s_supervisor_running = false;
volatile bool s_stop_requested = false;
static volatile bool s_start_in_progress = false;

/* Deep-copied config strings for use on reconnect */
lte_upstream_pppos_config_t s_saved_cfg;
static char s_saved_apn[64];
static char s_saved_user[32];
static char s_saved_pass[32];
static char s_saved_pin[16];

volatile bool s_time_synced = false;

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

        /* ESPNetLink: publish the WAN uplink into dev_status. The
         * eth_connected bit's WiCAN semantic is "the uplink this dongle
         * provides" (their usb_host side sets it for THIS device's
         * RNDIS) — on the dongle itself, LTE PPP is that same uplink.
         * Consumers gating on DEV_STATUS_NETWORK_CONNECTED_MASK
         * (vpn_manager, ha_webhooks, /api/status network_connected)
         * work unmodified. */
        dev_status_manager_set(DEV_STATUS_BIT_ETH_CONNECTED);
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

        dev_status_manager_clear(DEV_STATUS_BIT_ETH_CONNECTED);
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
void lte_inner_stop(void)
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

/* -----------------------------------------------------------------------
 * lte_inner_start – sets up DCE / netif / event-group.
 * Does NOT touch s_supervisor_task / s_supervisor_running.
 * Error paths call lte_inner_stop() for clean self-rollback.
 * ----------------------------------------------------------------------- */
esp_err_t lte_inner_start(const lte_upstream_pppos_config_t *cfg)
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
    /* UART rings live in INTERNAL RAM (CONFIG_UART_ISR_IN_IRAM forces the
     * driver to allocate internal). 32K/16K stopped fitting once esp_wifi
     * joined the build (bench-hit 2026-07-11: "UART driver malloc error" ->
     * esp_modem_new NULL). 16K/8K is ample: the BG95 peaks well under
     * 400 kbps and CTS/RTS flow control holds it off while the ring drains. */
    dte_config.uart_config.rx_buffer_size         = 16384;
    dte_config.uart_config.tx_buffer_size         = 8192;
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
        s_monitor_task = lte_monitor_start();
        if (s_monitor_task == NULL)
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
    s_supervisor_task = lte_supervisor_start();
    if (s_supervisor_task == NULL)
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
