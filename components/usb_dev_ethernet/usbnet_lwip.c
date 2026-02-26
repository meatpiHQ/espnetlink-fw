#include "private/usbnet_lwip.h"

#include <string.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/etharp.h"
#include "lwip/inet.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/tcpip.h"

#include "dhserver.h"

#include "usbd_core.h"
#include "usbd_rndis.h"
#include "usb_errno.h"

#include "private/usbnet_dhcp_debug.h"
#include "private/usbnet_state.h"
#include "private/usbnet_usb.h"

static const char *TAG = "usb_dev_eth";

static err_t usbnet_linkoutput(struct netif *netif, struct pbuf *p)
{
    (void)netif;

#if USBNET_DHCP_DEBUG
    usbnet_debug_dhcp_pbuf("TX", p);
#endif

    for (int attempt = 0; attempt < 20; attempt++)
    {
        int ret = usbd_rndis_eth_tx(p);
        if (ret == 0)
        {
            return ERR_OK;
        }

        if (ret == -USB_ERR_BUSY)
        {
            (void)usbnet_usb_wait_tx_done(pdMS_TO_TICKS(50));
            continue;
        }

        return ERR_IF;
    }

    return ERR_BUF;
}

static err_t usbnet_netif_init(struct netif *netif)
{
    usbnet_state_t *st = usbnet_state();

    netif->name[0] = 'u';
    netif->name[1] = '0';
    netif->output = etharp_output;
    netif->linkoutput = usbnet_linkoutput;
    netif->mtu = 1500;
    netif->hwaddr_len = 6;
    memcpy(netif->hwaddr, st->hwaddr, 6);
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET;

    return ERR_OK;
}

void usbnet_lwip_input_in_tcpip(void *arg)
{
    (void)arg;

    usbnet_state_t *st = usbnet_state();
    if (!st->lwip_ready)
    {
        return;
    }

    while (true)
    {
        struct pbuf *p = usbd_rndis_eth_rx();
        if (!p)
        {
            break;
        }

#if USBNET_DHCP_DEBUG
        usbnet_debug_dhcp_pbuf("RX", p);
#endif

        err_t err = st->usbnetif.input(p, &st->usbnetif);
        if (err != ERR_OK)
        {
            pbuf_free(p);
        }
    }
}

void usbnet_lwip_start_in_tcpip(void *arg)
{
    (void)arg;

    usbnet_state_t *st = usbnet_state();

    if (st->lwip_started)
    {
        netif_set_up(&st->usbnetif);
        netif_set_link_up(&st->usbnetif);
        st->lwip_ready = true;
        return;
    }

    IP4_ADDR(&st->ipaddr, 192, 168, 7, 1);
    IP4_ADDR(&st->netmask, 255, 255, 255, 0);
    IP4_ADDR(&st->gw, 192, 168, 7, 1);

    // Upstream DNS (over PPP) unknown until PPP comes up.
    IP4_ADDR(&st->upstream_dns, 0, 0, 0, 0);

    if (!netif_add(&st->usbnetif, &st->ipaddr, &st->netmask, &st->gw, NULL, usbnet_netif_init, tcpip_input))
    {
        ESP_LOGE(TAG, "netif_add failed");
        st->lwip_ready = false;
        return;
    }

    netif_set_up(&st->usbnetif);
    netif_set_link_up(&st->usbnetif);

    memset(st->dhcp_entries, 0, sizeof(st->dhcp_entries));
    for (int i = 0; i < (int)(sizeof(st->dhcp_entries) / sizeof(st->dhcp_entries[0])); i++)
    {
        IP4_ADDR(&st->dhcp_entries[i].addr, 192, 168, 7, 2 + i);
        st->dhcp_entries[i].lease = 24 * 60 * 60;
    }

    memset(&st->dhcp_cfg, 0, sizeof(st->dhcp_cfg));
    st->dhcp_cfg.router = st->ipaddr;
    st->dhcp_cfg.port = 67;

    // Advertise the device IP as DNS server and run a small DNS proxy on 192.168.7.1.
    st->dhcp_cfg.dns = st->ipaddr;

    st->dhcp_cfg.domain = NULL;
    st->dhcp_cfg.num_entry = (int)(sizeof(st->dhcp_entries) / sizeof(st->dhcp_entries[0]));
    st->dhcp_cfg.entries = st->dhcp_entries;

    err_t dh_err = dhserv_init(&st->dhcp_cfg);
    if (dh_err != ERR_OK)
    {
        ESP_LOGE(TAG, "DHCP server start failed: dhserv_init err=%d", (int)dh_err);
    }
    else
    {
        ESP_LOGI(TAG, "DHCP server started on UDP/%u", (unsigned)st->dhcp_cfg.port);
    }

    st->lwip_started = true;
    st->lwip_ready = true;

    /* If enable_sharing() was called before we were ready, apply it now. */
    usbnet_rndis_apply_pending_share();
}

void usbnet_lwip_stop_in_tcpip(void *arg)
{
    (void)arg;

    usbnet_state_t *st = usbnet_state();

    if (!st->lwip_started)
    {
        return;
    }

    st->lwip_ready = false;
    dhserv_free();

    netif_set_link_down(&st->usbnetif);
    netif_set_down(&st->usbnetif);
}
