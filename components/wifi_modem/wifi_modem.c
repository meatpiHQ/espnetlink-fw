/**
 * @file wifi_modem.c
 * @brief wifi_modem engine: lifecycle, boot-time USB data-cut, AP NAPT into
 *        the PPP uplink, DHCP DNS offer, upstream-DNS tracking.
 *
 * Sequence (main.c):
 *   settings_manager_start -> wifi_modem_provision()        (AP password)
 *                          -> wifi_modem_apply_usb_route()  (mux 1/1)
 *   ... wifi_manager_start -> wifi_modem_start()            (NAPT/DNS)
 *   usb_dev_ethernet_start only if wifi_modem_usb_data_enabled()
 *
 * NAPT is lwIP-global: ip_napt_enable(<LAN-side address>, 1) marks the AP
 * netif as the NATed side; traffic leaves through the default route (the
 * PPP netif once it has an IP). Same convention usb_dev_ethernet uses for
 * its 192.168.7.1 netif.
 */
#include "wifi_modem.h"

#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/lwip_napt.h"
#include "lwip/tcpip.h"

#include "hw_config.h"
#include "log_manager.h"
#include "wifi_manager.h"

#include "wifi_modem_private.h"

static const char *TAG = "wifi_modem";

#define SHARE_TASK_STACK 4096
#define AP_ADDR_WAIT_S   60

static bool s_started;
static volatile bool s_usb_cut;            /* data lines driven to 1/1 */
static volatile bool s_stop;
static volatile bool s_napt_active;
static uint32_t s_ap_ip;                  /* network order, 0 = not armed */
static uint32_t s_upstream_dns;           /* network order */
static bool s_ip_handler_registered;
static TaskHandle_t s_share_task;

/* §2: static stack in PSRAM (esp_netif ipc + socket setup only — no flash) */
static StaticTask_t s_share_tcb;                              /* internal: TCB */
static StackType_t s_share_stack[SHARE_TASK_STACK] EXT_RAM_BSS_ATTR;

/* ---- lifecycle --------------------------------------------------------- */

esp_err_t wifi_modem_init(void)
{
    static const log_descriptor_t LOG_DESC = { "wifi_modem", ESP_LOG_INFO };

    log_manager_register(&LOG_DESC); /* per-TAG level control (§9.2) */

    return wifi_modem_settings_register();
}

esp_err_t wifi_modem_provision(bool *reboot_required)
{
    return wifi_modem_provision_with(esp_random, reboot_required);
}

/* ---- USB route (boot + runtime) ----------------------------------------- */

/* SEL1/SEL2 = 1/1 = every USB data path disconnected; VBUS still powers
 * us. The deep-sleep path holds these pins (gpio_hold_en) and IDF v6's
 * gpio_reset_pin does NOT release a hold — clear it first. INPUT_OUTPUT
 * keeps the readback honest (the DIP is a weak pull). */
static void mux_cut(void)
{
    (void)gpio_hold_dis(USB_SEL_1);
    (void)gpio_hold_dis(USB_SEL_2);
    gpio_set_direction(USB_SEL_1, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(USB_SEL_1, 1);
    gpio_set_direction(USB_SEL_2, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(USB_SEL_2, 1);
    s_usb_cut = true;
}

/* Release the drive: the DIP switch pull selects the route again and the
 * host re-enumerates the device (a plug-in from the stack's point of view). */
static void mux_release(void)
{
    (void)gpio_hold_dis(USB_SEL_1);
    (void)gpio_hold_dis(USB_SEL_2);
    gpio_set_direction(USB_SEL_1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(USB_SEL_1, GPIO_FLOATING);
    gpio_set_direction(USB_SEL_2, GPIO_MODE_INPUT);
    gpio_set_pull_mode(USB_SEL_2, GPIO_FLOATING);
    s_usb_cut = false;
}

esp_err_t wifi_modem_apply_usb_route(void)
{
    if (!wifi_modem_settings_is_configured())
    {
        return ESP_ERR_INVALID_STATE;
    }

    const wifi_modem_settings_t *cfg = wifi_modem_settings_config();

    if (cfg->usb_mode != WIFI_MODEM_USB_POWER_ONLY)
    {
        s_usb_cut = false;
        ESP_LOGI(TAG, "usb_mode=%s: mux left on the DIP route (SEL1=%d SEL2=%d)",
                 wmc_usb_mode_str(cfg->usb_mode), gpio_get_level(USB_SEL_1),
                 gpio_get_level(USB_SEL_2));
        return ESP_OK;
    }

    mux_cut();
    ESP_LOGW(TAG, "usb_mode=power_only: USB DATA CUT at the mux (readback "
                  "SEL1=%d SEL2=%d) - internet/GPS served over the WiFi AP",
             gpio_get_level(USB_SEL_1), gpio_get_level(USB_SEL_2));
    return ESP_OK;
}

bool wifi_modem_usb_data_routed(void)
{
    return !s_usb_cut;
}

esp_err_t wifi_modem_usb_data_set(bool enabled)
{
    if (!wifi_modem_settings_is_configured())
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (!wmc_usb_data_switchable(wifi_modem_settings_config()->usb_mode))
    {
        return ESP_ERR_NOT_SUPPORTED; /* usb_mode=ncm: locked on */
    }

    if (enabled == !s_usb_cut)
    {
        return ESP_OK; /* already there */
    }

    if (enabled)
    {
        mux_release();
        ESP_LOGI(TAG, "USB data RESTORED to the DIP route (SEL1=%d SEL2=%d)",
                 gpio_get_level(USB_SEL_1), gpio_get_level(USB_SEL_2));
    }
    else
    {
        mux_cut();
        ESP_LOGI(TAG, "USB data CUT at the mux by the host (readback SEL1=%d "
                      "SEL2=%d) - cable is power only",
                 gpio_get_level(USB_SEL_1), gpio_get_level(USB_SEL_2));
    }

    return ESP_OK;
}

/* ---- AP sharing -------------------------------------------------------- */

bool wifi_modem_napt_active(void)
{
    return s_napt_active;
}

uint32_t wifi_modem_upstream_dns(void)
{
    return s_upstream_dns;
}

static void napt_set_in_tcpip(void *arg)
{
    bool enable = (arg != NULL);

#if CONFIG_LWIP_IPV4_NAPT
    ip_napt_enable(s_ap_ip, enable ? 1 : 0);
    s_napt_active = enable;
#else
    (void)enable;
    ESP_LOGW(TAG, "CONFIG_LWIP_IPV4_NAPT is off: AP clients get no internet");
#endif
}

/* DHCP: offer the AP address as the resolver (the UDP/53 proxy forwards
 * to the PPP-learned DNS). Option SET needs the server stopped; the
 * stop/start window is a few ms right after wifi_manager_start, before
 * any client can have joined. */
static esp_err_t dhcps_offer_dns(esp_netif_t *ap, uint32_t ap_ip)
{
    esp_err_t err;
    uint8_t offer = 1;
    esp_netif_dns_info_t dns = { 0 };

    dns.ip.type = ESP_IPADDR_TYPE_V4;
    dns.ip.u_addr.ip4.addr = ap_ip;

    esp_netif_dhcps_stop(ap);
    err = esp_netif_dhcps_option(ap, ESP_NETIF_OP_SET,
                                 ESP_NETIF_DOMAIN_NAME_SERVER,
                                 &offer, sizeof(offer));
    if (err == ESP_OK)
    {
        err = esp_netif_set_dns_info(ap, ESP_NETIF_DNS_MAIN, &dns);
    }

    esp_err_t start_err = esp_netif_dhcps_start(ap);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dhcps DNS offer: %s", esp_err_to_name(err));
        return err;
    }
    if (start_err != ESP_OK)
    {
        ESP_LOGE(TAG, "dhcps restart: %s", esp_err_to_name(start_err));
        return start_err;
    }
    return ESP_OK;
}

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id,
                        void *data)
{
    (void)arg;
    (void)base;

    if (id == IP_EVENT_PPP_GOT_IP)
    {
        const ip_event_got_ip_t *e = data;
        esp_netif_dns_info_t dns = { 0 };

        if (e != NULL && e->esp_netif != NULL &&
            esp_netif_get_dns_info(e->esp_netif, ESP_NETIF_DNS_MAIN,
                                   &dns) == ESP_OK &&
            dns.ip.type == ESP_IPADDR_TYPE_V4 && dns.ip.u_addr.ip4.addr != 0)
        {
            s_upstream_dns = dns.ip.u_addr.ip4.addr;
            wifi_modem_dns_set_upstream(s_upstream_dns);
            ESP_LOGI(TAG, "PPP up: upstream DNS " IPSTR,
                     IP2STR(&dns.ip.u_addr.ip4));
        }
        else
        {
            ESP_LOGW(TAG, "PPP up without a DNS server: proxy falls back "
                          "to 8.8.8.8");
        }
    }
    else if (id == IP_EVENT_PPP_LOST_IP)
    {
        ESP_LOGI(TAG, "PPP down: AP clients lose the uplink until it returns");
    }
}

/* The AP netif carries its address from wifi_manager_start, but stay
 * tolerant: poll briefly, then arm NAPT + DNS offer + proxy. Exits once
 * armed (or on stop) — no long-lived task. */
static void share_task(void *arg)
{
    (void)arg;

    esp_netif_t *ap = NULL;
    esp_netif_ip_info_t ip = { 0 };

    for (int i = 0; i < AP_ADDR_WAIT_S && !s_stop; i++)
    {
        ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
        if (ap != NULL && esp_netif_get_ip_info(ap, &ip) == ESP_OK &&
            ip.ip.addr != 0)
        {
            break;
        }
        ap = NULL;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (ap == NULL)
    {
        if (!s_stop)
        {
            ESP_LOGE(TAG, "AP netif never got an address: sharing NOT armed");
        }
    }
    else
    {
        s_ap_ip = ip.ip.addr;
        (void)dhcps_offer_dns(ap, ip.ip.addr);
        (void)wifi_modem_dns_start(ip.ip.addr);
        tcpip_callback_with_block(napt_set_in_tcpip, (void *)1, 1);

        ESP_LOGI(TAG, "AP sharing armed: NAPT on " IPSTR
                      ", DNS offered = AP (proxy)", IP2STR(&ip.ip));
    }

    s_share_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t wifi_modem_start(void)
{
    if (!wifi_modem_settings_is_configured())
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_started)
    {
        return ESP_OK;
    }

    const wifi_modem_settings_t *cfg = wifi_modem_settings_config();

    if (cfg->usb_mode == WIFI_MODEM_USB_POWER_ONLY && !wifi_manager_mode_has_ap())
    {
        ESP_LOGW(TAG, "usb_mode=power_only but the WiFi mode has no AP: "
                      "nothing can reach this device");
    }

    if (!cfg->ap_share || !wifi_manager_mode_has_ap())
    {
        ESP_LOGI(TAG, "AP sharing disabled (ap_share=%d has_ap=%d)",
                 (int)cfg->ap_share, (int)wifi_manager_mode_has_ap());
        s_started = true;
        return ESP_OK;
    }

    esp_err_t err = esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                               on_ip_event, NULL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "IP_EVENT handler: %s", esp_err_to_name(err));
        return err;
    }
    s_ip_handler_registered = true;

    s_stop = false;
    s_share_task = xTaskCreateStaticPinnedToCore(share_task, "wifi_modem",
                                                 SHARE_TASK_STACK, NULL, 5,
                                                 s_share_stack, &s_share_tcb, 0);
    if (s_share_task == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    return ESP_OK;
}

esp_err_t wifi_modem_stop(void)
{
    if (!s_started)
    {
        return ESP_OK;
    }

    s_stop = true;

    /* the share task polls at 1 Hz and exits on s_stop */
    for (int i = 0; i < 30 && s_share_task != NULL; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (s_napt_active)
    {
        tcpip_callback_with_block(napt_set_in_tcpip, NULL, 1);
    }
    wifi_modem_dns_stop();

    if (s_ip_handler_registered)
    {
        (void)esp_event_handler_unregister(IP_EVENT, ESP_EVENT_ANY_ID,
                                           on_ip_event);
        s_ip_handler_registered = false;
    }

    s_ap_ip = 0;
    s_started = false;
    ESP_LOGI(TAG, "stopped");
    return ESP_OK;
}
