#include "usb_dev_ethernet.h"
#include "private/usbnet_lwip.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "esp_log.h"

#include "lwip/tcpip.h"
#if CONFIG_LWIP_IPV4_NAPT
#include "lwip/lwip_napt.h"
#endif

#include "dhserver.h"

#include "private/usbnet_state.h"
#include "private/usbnet_usb.h"

static const char *TAG = "usb_dev_eth";

void usb_dev_ethernet_set_console_enabled(bool enabled)
{
    usbnet_usb_set_console_enabled(enabled);
}

typedef struct
{
    uint32_t dns_ipv4_addr;
} usbnet_sharing_cfg_t;

static void usbnet_enable_sharing_in_tcpip(void *arg)
{
    usbnet_sharing_cfg_t *cfg = (usbnet_sharing_cfg_t *)arg;
    usbnet_state_t *st = usbnet_state();

    if (cfg && cfg->dns_ipv4_addr != 0)
    {
        st->upstream_dns.addr = cfg->dns_ipv4_addr;
        // Keep DHCP option 6 as 192.168.7.1; DNS proxy forwards upstream.
        st->dhcp_cfg.dns = st->ipaddr;
    }

    dhserv_free();
    err_t dh_err = dhserv_init(&st->dhcp_cfg);
    if (dh_err != ERR_OK)
    {
        ESP_LOGE(TAG, "DHCP server restart failed: dhserv_init err=%d", (int)dh_err);
    }

#if CONFIG_LWIP_IPV4_NAPT
    ip_napt_enable(st->ipaddr.addr, 1);
#else
    ESP_LOGW(TAG, "NAT not enabled: set CONFIG_LWIP_IPV4_NAPT=y to share upstream internet");
#endif

    free(cfg);
}

esp_err_t usb_dev_ethernet_start(void)
{
    usbnet_usb_start();
    return ESP_OK;
}

esp_err_t usb_dev_ethernet_enable_sharing(uint32_t dns_ipv4_addr)
{
    usbnet_state_t *st = usbnet_state();
    if (!st->lwip_ready)
    {
        /* USB interface not yet up – store request and apply it once lwip starts. */
        st->pending_share = true;
        st->pending_dns.addr = dns_ipv4_addr;
        ESP_LOGI(TAG, "USB not ready yet – sharing will be enabled once USB is configured");
        return ESP_OK;
    }

    usbnet_sharing_cfg_t *cfg = (usbnet_sharing_cfg_t *)calloc(1, sizeof(*cfg));
    if (!cfg)
    {
        return ESP_ERR_NO_MEM;
    }
    cfg->dns_ipv4_addr = dns_ipv4_addr;

    err_t cb_err = tcpip_callback_with_block(usbnet_enable_sharing_in_tcpip, cfg, 1);
    if (cb_err != ERR_OK)
    {
        free(cfg);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "USB RNDIS sharing enabled (NAT + DHCP DNS)");
    return ESP_OK;
}

void usbnet_rndis_apply_pending_share(void)
{
    usbnet_state_t *st = usbnet_state();
    if (!st->pending_share)
    {
        return;
    }
    st->pending_share = false;

    /* Reuse the existing helper directly – we're already on the tcpip thread. */
    usbnet_sharing_cfg_t *cfg = (usbnet_sharing_cfg_t *)calloc(1, sizeof(*cfg));
    if (!cfg)
    {
        ESP_LOGE(TAG, "apply_pending_share: alloc failed");
        return;
    }
    cfg->dns_ipv4_addr = st->pending_dns.addr;
    usbnet_enable_sharing_in_tcpip(cfg); /* frees cfg */
    ESP_LOGI(TAG, "Deferred USB RNDIS sharing applied (NAT + DHCP DNS)");
}
