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

#include "dhcpserver/dhcpserver.h"
#include "dhcpserver/dhcpserver_options.h"

#include "usbd_core.h"
#include "usb_errno.h"

#include "private/usbnet_class.h"
#include "private/usbnet_dhcp_debug.h"
#include "private/usbnet_state.h"
#include "private/usbnet_usb.h"

static const char *TAG = "usb_dev_ethernet";

static err_t usbnet_linkoutput(struct netif *netif, struct pbuf *p)
{
    (void)netif;

#if USBNET_DHCP_DEBUG
    usbnet_debug_dhcp_pbuf("TX", p);
#endif

    for (int attempt = 0; attempt < 20; attempt++)
    {
        int ret = usbnet_class_eth_tx(p);
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

/* dhcpserver.c calls this UNCONDITIONALLY on every lease grant (esp_netif
 * always installs one; the raw API crashes on NULL — bench-hit 2026-07-11:
 * InstrFetchProhibited on the first DHCP REQUEST). */
static void usbnet_dhcps_lease_cb(void *cb_arg, u8_t client_ip[4],
                                  u8_t client_mac[6])
{
    (void)cb_arg;

    ESP_LOGI(TAG, "DHCP lease %u.%u.%u.%u -> %02X:%02X:%02X:%02X:%02X:%02X",
             client_ip[0], client_ip[1], client_ip[2], client_ip[3],
             client_mac[0], client_mac[1], client_mac[2], client_mac[3],
             client_mac[4], client_mac[5]);
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
        struct pbuf *p = usbnet_class_eth_rx();
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

        /* Re-plug / bus reset: the stop path stopped the DHCP server, so a
         * host that re-enumerates would never get a lease (Windows fell to
         * 169.254.x after the wifi_modem runtime restore, bench 2026-08-23).
         * Restart it with the same address/options. */
        if (st->dhcps != NULL)
        {
            err_t dh_err = dhcps_start(st->dhcps, &st->usbnetif, st->ipaddr);

            if (dh_err != ERR_OK)
            {
                ESP_LOGE(TAG, "DHCP server restart failed: err=%d", (int)dh_err);
            }
            else
            {
                ESP_LOGI(TAG, "DHCP server restarted (usbnet netif re-up)");
            }
        }

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

    // IDF's per-netif DHCP server (udp_bind_netif) — coexists with the WiFi
    // AP's server on UDP/67. Advertise the device IP as DNS server; the small
    // DNS proxy on 192.168.7.1 forwards upstream.
    if (st->dhcps == NULL)
    {
        st->dhcps = dhcps_new();
    }

    if (st->dhcps == NULL)
    {
        ESP_LOGE(TAG, "DHCP server start failed: dhcps_new returned NULL");
    }
    else
    {
        ip_addr_t dns;

        ip_addr_copy_from_ip4(dns, st->ipaddr);
        dhcps_dns_setserver(st->dhcps, &dns);

        /* Options 3 (router) and 6 (DNS) only when the uplink is shared over
         * USB. The IDF server's default mask offers the router always, which
         * moved a PC's default route onto the LTE link even with NAT off;
         * and offering ourselves as DNS without sharing makes the host wait
         * on a proxy that has no upstream. Control-channel mode = address
         * only: the host keeps its own gateway and resolvers. */
        dhcps_offer_t offer_dns = st->offer_router ? OFFER_DNS : 0;

        dhcps_set_option_info(st->dhcps, DOMAIN_NAME_SERVER, &offer_dns,
                              sizeof(offer_dns));

        dhcps_offer_t offer_router = st->offer_router ? OFFER_ROUTER : 0;

        dhcps_set_option_info(st->dhcps, ROUTER_SOLICITATION_ADDRESS,
                              &offer_router, sizeof(offer_router));

        dhcps_set_new_lease_cb(st->dhcps, usbnet_dhcps_lease_cb, NULL);

        err_t dh_err = dhcps_start(st->dhcps, &st->usbnetif, st->ipaddr);
        if (dh_err != ERR_OK)
        {
            ESP_LOGE(TAG, "DHCP server start failed: dhcps_start err=%d", (int)dh_err);
        }
        else
        {
            ESP_LOGI(TAG, "DHCP server started on UDP/67 (usbnet netif, "
                          "router+DNS %s)",
                     st->offer_router ? "offered" : "not offered");
        }
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
    if (st->dhcps != NULL)
    {
        dhcps_stop(st->dhcps, &st->usbnetif);
    }

    netif_set_link_down(&st->usbnetif);
    netif_set_down(&st->usbnetif);
}
