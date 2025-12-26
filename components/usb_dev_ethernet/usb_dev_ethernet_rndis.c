#include "usb_dev_ethernet.h"

#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"

#include "sdkconfig.h"

#include "lwip/etharp.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/sockets.h"
#include "lwip/tcpip.h"
#include "lwip/inet.h"
#if CONFIG_LWIP_IPV4_NAPT
#include "lwip/lwip_napt.h"
#endif

#include "dhserver.h" // TinyUSB networking dhcp server (netif-pinnable)

#include "usbd_core.h"
#include "usbd_rndis.h"
#include "usb_errno.h"

static const char *TAG = "usb_dev_eth";

// Set to 1 to log decoded DHCP message types/options seen on RX/TX.
// (Kept local to this component; no Kconfig required.)
#ifndef USBNET_DHCP_DEBUG
#define USBNET_DHCP_DEBUG 1
#endif

#if USBNET_DHCP_DEBUG

#define USBNET_ETH_HDR_LEN 14

static uint16_t usbnet_be16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] << 8) | (uint16_t)p[1];
}

static uint32_t usbnet_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static const char *usbnet_dhcp_msg_str(uint8_t t)
{
    switch (t) {
        case 1: return "DISCOVER";
        case 2: return "OFFER";
        case 3: return "REQUEST";
        case 4: return "DECLINE";
        case 5: return "ACK";
        case 6: return "NAK";
        case 7: return "RELEASE";
        case 8: return "INFORM";
        default: return "?";
    }
}

static void usbnet_ipv4_to_str(uint32_t be_addr, char out[16])
{
    uint8_t a = (uint8_t)(be_addr >> 24);
    uint8_t b = (uint8_t)(be_addr >> 16);
    uint8_t c = (uint8_t)(be_addr >> 8);
    uint8_t d = (uint8_t)(be_addr);
    (void)snprintf(out, 16, "%u.%u.%u.%u", a, b, c, d);
}

static void usbnet_debug_dhcp_pbuf(const char *dir, struct pbuf *p)
{
    static uint32_t s_tx_logged = 0;
    static uint32_t s_rx_logged = 0;

    // Avoid log spam: log first few packets per direction.
    if (dir && dir[0] == 'T') {
        if (s_tx_logged++ > 25) {
            return;
        }
    } else {
        if (s_rx_logged++ > 50) {
            return;
        }
    }

    // Copy a fixed window so we can parse DHCP options (offers are ~516 bytes).
    // This is only for diagnostics; keep it bounded.
    const uint16_t want = 600;
    uint16_t cap = (p->tot_len < want) ? (uint16_t)p->tot_len : want;
    uint8_t buf[600];
    (void)memset(buf, 0, sizeof(buf));
    (void)pbuf_copy_partial(p, buf, cap, 0);

    if (cap < USBNET_ETH_HDR_LEN + 20 + 8) {
        return;
    }

    // Ethernet
    uint16_t eth_type = usbnet_be16(&buf[12]);
    if (eth_type != 0x0800) { // IPv4
        return;
    }

    // IPv4
    const uint8_t *ip = &buf[USBNET_ETH_HDR_LEN];
    uint8_t ver_ihl = ip[0];
    if ((ver_ihl >> 4) != 4) {
        return;
    }
    uint8_t ihl = (uint8_t)((ver_ihl & 0x0F) * 4);
    if (ihl < 20 || cap < (uint16_t)(USBNET_ETH_HDR_LEN + ihl + 8)) {
        return;
    }
    if (ip[9] != 17) { // UDP
        return;
    }

    char src_ip[16] = {0};
    char dst_ip[16] = {0};
    usbnet_ipv4_to_str(usbnet_be32(&ip[12]), src_ip);
    usbnet_ipv4_to_str(usbnet_be32(&ip[16]), dst_ip);

    // UDP
    const uint8_t *udp = ip + ihl;
    uint16_t sport = usbnet_be16(&udp[0]);
    uint16_t dport = usbnet_be16(&udp[2]);
    uint16_t ulen = usbnet_be16(&udp[4]);

    if (!((sport == 67 || sport == 68) && (dport == 67 || dport == 68))) {
        return;
    }

    // BOOTP/DHCP
    const uint8_t *bootp = udp + 8;
    uint16_t udp_payload_avail = (cap > (uint16_t)(USBNET_ETH_HDR_LEN + ihl + 8))
                                     ? (uint16_t)(cap - (USBNET_ETH_HDR_LEN + ihl + 8))
                                     : 0;
    if (udp_payload_avail < 240) {
        ESP_LOGW(TAG, "%s DHCP short: %s:%u -> %s:%u (udp_len=%u, cap_payload=%u)",
                 dir, src_ip, sport, dst_ip, dport, (unsigned)ulen, (unsigned)udp_payload_avail);
        return;
    }

    uint8_t op = bootp[0];
    uint8_t htype = bootp[1];
    uint8_t hlen = bootp[2];
    uint32_t xid = usbnet_be32(&bootp[4]);
    uint16_t flags = usbnet_be16(&bootp[10]);

    uint32_t ciaddr = usbnet_be32(&bootp[12]);
    uint32_t yiaddr = usbnet_be32(&bootp[16]);
    uint32_t siaddr = usbnet_be32(&bootp[20]);

    const uint8_t *chaddr = &bootp[28];
    char ch[18] = {0};
    if (hlen >= 6) {
        (void)snprintf(ch, sizeof(ch), "%02X:%02X:%02X:%02X:%02X:%02X",
                       chaddr[0], chaddr[1], chaddr[2], chaddr[3], chaddr[4], chaddr[5]);
    } else {
        (void)snprintf(ch, sizeof(ch), "-");
    }

    char ci[16] = {0};
    char yi[16] = {0};
    char si[16] = {0};
    usbnet_ipv4_to_str(ciaddr, ci);
    usbnet_ipv4_to_str(yiaddr, yi);
    usbnet_ipv4_to_str(siaddr, si);

    const uint8_t *opts = bootp + 236;
    uint32_t cookie = usbnet_be32(opts);
    if (cookie != 0x63825363u) {
        ESP_LOGW(TAG, "%s DHCP bad cookie 0x%08" PRIX32 " (op=%u xid=0x%08" PRIX32 ")",
                 dir, cookie, (unsigned)op, xid);
        return;
    }

    uint8_t msg_type = 0;
    uint32_t server_id = 0;
    uint32_t subnet_mask = 0;
    uint32_t router = 0;
    uint32_t dns = 0;
    bool prl_has_dns = false;
    bool prl_has_router = false;
    bool prl_has_subnet = false;

    // Parse options (best-effort).
    uint16_t idx = 4;
    while ((uint32_t)(236 + idx) < udp_payload_avail) {
        uint8_t code = opts[idx++];
        if (code == 0) {
            continue;
        }
        if (code == 255) {
            break;
        }
        if ((uint32_t)(236 + idx) >= udp_payload_avail) {
            break;
        }
        uint8_t len = opts[idx++];
        if ((uint32_t)(236 + idx + len) > udp_payload_avail) {
            break;
        }

        const uint8_t *val = &opts[idx];
        if (code == 53 && len >= 1) {
            msg_type = val[0];
        } else if (code == 54 && len == 4) {
            server_id = usbnet_be32(val);
        } else if (code == 1 && len == 4) {
            subnet_mask = usbnet_be32(val);
        } else if (code == 3 && len >= 4) {
            router = usbnet_be32(val);
        } else if (code == 6 && len >= 4) {
            dns = usbnet_be32(val);
        } else if (code == 55 && len > 0) {
            for (uint8_t j = 0; j < len; j++) {
                if (val[j] == 6) {
                    prl_has_dns = true;
                } else if (val[j] == 3) {
                    prl_has_router = true;
                } else if (val[j] == 1) {
                    prl_has_subnet = true;
                }
            }
        }

        idx = (uint16_t)(idx + len);
    }

    char sid[16] = {0};
    char sm[16] = {0};
    char rt[16] = {0};
    char dn[16] = {0};
    usbnet_ipv4_to_str(server_id, sid);
    usbnet_ipv4_to_str(subnet_mask, sm);
    usbnet_ipv4_to_str(router, rt);
    usbnet_ipv4_to_str(dns, dn);

    ESP_LOGI(TAG,
             "%s DHCP %s (op=%u htype=%u hlen=%u chaddr=%s xid=0x%08" PRIX32 " flags=0x%04X) %s:%u -> %s:%u ci=%s yi=%s si=%s opt54=%s opt1=%s opt3=%s opt6=%s prl(dns=%u router=%u subnet=%u)",
             dir,
             usbnet_dhcp_msg_str(msg_type),
             (unsigned)op,
             (unsigned)htype,
             (unsigned)hlen,
             ch,
             xid,
             (unsigned)flags,
             src_ip,
             (unsigned)sport,
             dst_ip,
             (unsigned)dport,
             ci,
             yi,
             si,
             (server_id != 0) ? sid : "-",
             (subnet_mask != 0) ? sm : "-",
             (router != 0) ? rt : "-",
             (dns != 0) ? dn : "-",
             prl_has_dns ? 1u : 0u,
             prl_has_router ? 1u : 0u,
             prl_has_subnet ? 1u : 0u);
}

#endif // USBNET_DHCP_DEBUG

// -------------------------
// CherryUSB RNDIS descriptors
// -------------------------

#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x02
#define CDC_INT_EP 0x83

#define USBD_VID 0x303A
#define USBD_PID 0x4007 // keep distinct from prior NCM PID to avoid Windows caching

#define USBD_MAX_POWER_MA 100
#define USBD_LANGID_EN_US 1033

#define USB_CONFIG_SIZE (9 + CDC_RNDIS_DESCRIPTOR_LEN)

#define USB_STRING_RNDIS_INTF_INDEX 0x04

#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#else
#define CDC_MAX_MPS 64
#endif

// CherryUSB on ESP-IDF enables ADVANCE_DESC by default (see usb_config.h), so register
// descriptors via callbacks.

static const uint8_t s_device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
};

static const uint8_t s_config_descriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER_MA),
    CDC_RNDIS_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MAX_MPS, USB_STRING_RNDIS_INTF_INDEX),
};

static const uint8_t s_device_quality_descriptor[] = {
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
};

static const char *s_string_descriptors[] = {
    (const char[]){ 0x09, 0x04 }, /* Langid: en-US */
    "ESPNetLink",                /* Manufacturer */
    "ESPNetLink RNDIS",          /* Product */
    "000000000000",             /* Serial */
    "RNDIS",                     /* Interface */
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return s_device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return s_config_descriptor;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return s_device_quality_descriptor;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void)speed;

    if (index >= (uint8_t)(sizeof(s_string_descriptors) / sizeof(s_string_descriptors[0]))) {
        return NULL;
    }
    return s_string_descriptors[index];
}

static const struct usb_descriptor s_usb_desc = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
};

// -------------------------
// lwIP netif + DHCP
// -------------------------

static struct netif s_usbnetif;
static bool s_lwip_ready = false;
static bool s_lwip_started = false;

static ip4_addr_t s_ipaddr;
static ip4_addr_t s_netmask;
static ip4_addr_t s_gw;
static ip4_addr_t s_upstream_dns;

static dhcp_entry_t s_dhcp_entries[4];
static dhcp_config_t s_dhcp_cfg;

static struct usbd_interface s_rndis_intf;
static TaskHandle_t s_rx_task = NULL;
static TaskHandle_t s_evt_task = NULL;
static uint8_t s_hwaddr[6] = {0};
static uint8_t s_rndis_host_mac[6] = {0};

static TaskHandle_t s_dns_task = NULL;
static int s_dns_sock = -1;

#define USBNET_DNS_PORT 53

static StaticSemaphore_t s_tx_done_sem_buf;
static SemaphoreHandle_t s_tx_done_sem = NULL;

#define USBNET_EVT_CONFIGURED  (1u << 0)
#define USBNET_EVT_DISCONNECT  (1u << 1)

static err_t usbnet_linkoutput(struct netif *netif, struct pbuf *p)
{
    (void)netif;

#if USBNET_DHCP_DEBUG
    usbnet_debug_dhcp_pbuf("TX", p);
#endif

    if (!s_tx_done_sem) {
        return ERR_IF;
    }

    for (int attempt = 0; attempt < 20; attempt++) {
        int ret = usbd_rndis_eth_tx(p);
        if (ret == 0) {
            return ERR_OK;
        }

        if (ret == -USB_ERR_BUSY) {
            // Wait for the previous IN transfer to complete then retry.
            (void)xSemaphoreTake(s_tx_done_sem, pdMS_TO_TICKS(50));
            continue;
        }

        // not connected / other failure
        return ERR_IF;
    }

    return ERR_BUF;
}

static err_t usbnet_netif_init(struct netif *netif)
{
    netif->name[0] = 'u';
    netif->name[1] = '0';
    netif->output = etharp_output;
    netif->linkoutput = usbnet_linkoutput;
    netif->mtu = 1500;
    netif->hwaddr_len = 6;
    memcpy(netif->hwaddr, s_hwaddr, 6);
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET;
    return ERR_OK;
}

static void usbnet_input_in_tcpip(void *arg)
{
    (void)arg;
    if (!s_lwip_ready) {
        return;
    }

    while (true) {
        struct pbuf *p = usbd_rndis_eth_rx();
        if (!p) {
            break;
        }

    #if USBNET_DHCP_DEBUG
        usbnet_debug_dhcp_pbuf("RX", p);
    #endif

        err_t err = s_usbnetif.input(p, &s_usbnetif);
        if (err != ERR_OK) {
            pbuf_free(p);
        }
    }
}

static void usbnet_start_lwip_in_tcpip(void *arg)
{
    (void)arg;

    if (s_lwip_started) {
        netif_set_up(&s_usbnetif);
        netif_set_link_up(&s_usbnetif);
        s_lwip_ready = true;
        return;
    }

    IP4_ADDR(&s_ipaddr, 192, 168, 7, 1);
    IP4_ADDR(&s_netmask, 255, 255, 255, 0);
    IP4_ADDR(&s_gw, 192, 168, 7, 1);

    // Upstream DNS (over PPP) unknown until PPP comes up.
    IP4_ADDR(&s_upstream_dns, 0, 0, 0, 0);

    if (!netif_add(&s_usbnetif, &s_ipaddr, &s_netmask, &s_gw, NULL, usbnet_netif_init, tcpip_input)) {
        ESP_LOGE(TAG, "netif_add failed");
        s_lwip_ready = false;
        return;
    }

    netif_set_up(&s_usbnetif);
    netif_set_link_up(&s_usbnetif);

    memset(s_dhcp_entries, 0, sizeof(s_dhcp_entries));
    for (int i = 0; i < (int)(sizeof(s_dhcp_entries) / sizeof(s_dhcp_entries[0])); i++) {
        // 192.168.7.2 ...
        IP4_ADDR(&s_dhcp_entries[i].addr, 192, 168, 7, 2 + i);
        s_dhcp_entries[i].lease = 24 * 60 * 60;
    }

    memset(&s_dhcp_cfg, 0, sizeof(s_dhcp_cfg));
    s_dhcp_cfg.netif = &s_usbnetif;
    s_dhcp_cfg.router = s_ipaddr;
    s_dhcp_cfg.port = 67;
    // Advertise the device IP as DNS server and run a small DNS proxy on 192.168.7.1.
    // This avoids needing to renew DHCP when PPP comes up.
    s_dhcp_cfg.dns = s_ipaddr;
    s_dhcp_cfg.domain = NULL;
    s_dhcp_cfg.num_entry = (int)(sizeof(s_dhcp_entries) / sizeof(s_dhcp_entries[0]));
    s_dhcp_cfg.entries = s_dhcp_entries;

    err_t dh_err = dhserv_init(&s_dhcp_cfg);
    if (dh_err != ERR_OK) {
        ESP_LOGE(TAG, "DHCP server start failed: dhserv_init err=%d", (int)dh_err);
    } else {
        ESP_LOGI(TAG, "DHCP server started on UDP/%u", (unsigned)s_dhcp_cfg.port);
    }

    s_lwip_started = true;
    s_lwip_ready = true;
}

static void usbnet_stop_lwip_in_tcpip(void *arg)
{
    (void)arg;

    if (!s_lwip_started) {
        return;
    }

    s_lwip_ready = false;
    dhserv_free();

    if (s_dns_sock >= 0) {
        (void)lwip_close(s_dns_sock);
        s_dns_sock = -1;
    }

    netif_set_link_down(&s_usbnetif);
    netif_set_down(&s_usbnetif);
}

static bool usbnet_is_usb_client_ipv4(uint32_t be_addr)
{
    // 192.168.7.0/24, be_addr in network order
    return ((lwip_ntohl(be_addr) & 0xFFFFFF00u) == 0xC0A80700u);
}

static void usbnet_dns_proxy_task(void *arg)
{
    (void)arg;

    uint8_t buf[512];

    while (true) {
        if (!s_lwip_ready) {
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        if (s_dns_sock < 0) {
            int sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (sock < 0) {
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }

            struct sockaddr_in bind_addr = {0};
            bind_addr.sin_family = AF_INET;
            bind_addr.sin_port = lwip_htons(USBNET_DNS_PORT);
            bind_addr.sin_addr.s_addr = lwip_htonl(INADDR_ANY);

            if (lwip_bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) != 0) {
                (void)lwip_close(sock);
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }

            s_dns_sock = sock;
            ESP_LOGI(TAG, "DNS proxy listening on UDP/%u", (unsigned)USBNET_DNS_PORT);
        }

        struct sockaddr_in client = {0};
        socklen_t clen = sizeof(client);
        int n = lwip_recvfrom(s_dns_sock, buf, sizeof(buf), 0, (struct sockaddr *)&client, &clen);
        if (n <= 0) {
            continue;
        }

        if (!usbnet_is_usb_client_ipv4(client.sin_addr.s_addr)) {
            continue;
        }

        // Choose upstream DNS: prefer PPP-provided DNS, else fall back.
        ip4_addr_t upstream = s_upstream_dns;
        if (ip4_addr_isany_val(upstream)) {
            IP4_ADDR(&upstream, 8, 8, 8, 8);
        }

        int up = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (up < 0) {
            continue;
        }

        struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
        (void)lwip_setsockopt(up, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        struct sockaddr_in upstream_addr = {0};
        upstream_addr.sin_family = AF_INET;
        upstream_addr.sin_port = lwip_htons(53);
        upstream_addr.sin_addr.s_addr = upstream.addr;

        if (lwip_sendto(up, buf, n, 0, (struct sockaddr *)&upstream_addr, sizeof(upstream_addr)) < 0) {
            (void)lwip_close(up);
            continue;
        }

        struct sockaddr_in from = {0};
        socklen_t flen = sizeof(from);
        int rn = lwip_recvfrom(up, buf, sizeof(buf), 0, (struct sockaddr *)&from, &flen);
        (void)lwip_close(up);

        if (rn <= 0) {
            continue;
        }

        (void)lwip_sendto(s_dns_sock, buf, rn, 0, (struct sockaddr *)&client, clen);
    }
}

// -------------------------
// CherryUSB callbacks
// -------------------------

void usbd_rndis_data_recv_done(uint32_t len)
{
    (void)len;

    if (!s_rx_task) {
        return;
    }

    // This callback can be invoked from ISR context.
    if (xPortInIsrContext()) {
        BaseType_t hp_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(s_rx_task, &hp_task_woken);
        if (hp_task_woken) {
            portYIELD_FROM_ISR();
        }
    } else {
        xTaskNotifyGive(s_rx_task);
    }
}

void usbd_rndis_data_send_done(uint32_t len)
{
    (void)len;

    if (!s_tx_done_sem) {
        return;
    }

    // This callback can be invoked from ISR context.
    if (xPortInIsrContext()) {
        BaseType_t hp_task_woken = pdFALSE;
        (void)xSemaphoreGiveFromISR(s_tx_done_sem, &hp_task_woken);
        if (hp_task_woken) {
            portYIELD_FROM_ISR();
        }
    } else {
        (void)xSemaphoreGive(s_tx_done_sem);
    }
}

static void usbnet_rx_task(void *arg)
{
    (void)arg;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!s_lwip_ready) {
            continue;
        }

        (void)tcpip_callback_with_block(usbnet_input_in_tcpip, NULL, 1);
    }
}

static void usbnet_evt_task(void *arg)
{
    (void)arg;

    while (true) {
        uint32_t bits = 0;
        (void)xTaskNotifyWait(0, UINT32_MAX, &bits, portMAX_DELAY);

        if ((bits & USBNET_EVT_DISCONNECT) != 0u) {
            ESP_LOGI(TAG, "USB disconnected/reset");
            (void)usbd_rndis_set_connect(false);
            (void)tcpip_callback_with_block(usbnet_stop_lwip_in_tcpip, NULL, 1);
        }

        if ((bits & USBNET_EVT_CONFIGURED) != 0u) {
            ESP_LOGI(TAG, "USB configured");
            (void)usbd_rndis_set_connect(true);
            (void)tcpip_callback_with_block(usbnet_start_lwip_in_tcpip, NULL, 1);
        }
    }
}

static void usbnet_usbd_event_handler(uint8_t busid, uint8_t event)
{
    (void)busid;

    if (!s_evt_task) {
        return;
    }

    uint32_t notify_bits = 0;

    switch (event) {
        case USBD_EVENT_CONFIGURED: {
            notify_bits = USBNET_EVT_CONFIGURED;
            break;
        }
        case USBD_EVENT_DISCONNECTED:
        case USBD_EVENT_RESET: {
            notify_bits = USBNET_EVT_DISCONNECT;
            break;
        }
        default:
            break;
    }

    if (notify_bits == 0) {
        return;
    }

    // CherryUSB can invoke this handler from ISR context.
    if (xPortInIsrContext()) {
        BaseType_t hp_task_woken = pdFALSE;
        (void)xTaskNotifyFromISR(s_evt_task, notify_bits, eSetBits, &hp_task_woken);
        if (hp_task_woken) {
            portYIELD_FROM_ISR();
        }
    } else {
        (void)xTaskNotify(s_evt_task, notify_bits, eSetBits);
    }
}

esp_err_t usb_dev_ethernet_start(void)
{
    if (!s_rx_task) {
        xTaskCreatePinnedToCore(usbnet_rx_task, "usbnet_rx", 4096, NULL, 10, &s_rx_task, 0);
    }

    if (!s_evt_task) {
        xTaskCreatePinnedToCore(usbnet_evt_task, "usbnet_evt", 3072, NULL, 11, &s_evt_task, 0);
    }

    if (!s_tx_done_sem) {
        s_tx_done_sem = xSemaphoreCreateBinaryStatic(&s_tx_done_sem_buf);
    }

    if (!s_dns_task) {
        xTaskCreatePinnedToCore(usbnet_dns_proxy_task, "usbnet_dns", 4096, NULL, 9, &s_dns_task, 0);
    }

    uint8_t dev_mac[6] = {0};
    esp_err_t err = esp_read_mac(dev_mac, ESP_MAC_WIFI_STA);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_read_mac failed (%d), using fallback", (int)err);
        dev_mac[0] = 0x02;
        dev_mac[1] = 0x00;
        dev_mac[2] = 0x00;
        dev_mac[3] = 0x00;
        dev_mac[4] = 0x00;
        dev_mac[5] = 0x01;
    } else {
        // Ensure locally administered + unicast
        dev_mac[0] = (uint8_t)((dev_mac[0] | 0x02) & 0xFE);
    }

    // Important Windows interop detail:
    // - The MAC reported by RNDIS (OID_802_3_CURRENT_ADDRESS) is used by Windows as the NIC's
    //   adapter address and shows up as the source MAC in DHCP DISCOVER.
    // - If we send frames with the same source MAC, Windows can treat them as loopback and drop
    //   them (seen in pktmon as "Loopback Ethernet packet"), which prevents DHCP from progressing.
    // Therefore use a distinct MAC for our lwIP netif (device MAC) and a different one for the
    // RNDIS adapter MAC used by Windows.
    memcpy(s_hwaddr, dev_mac, sizeof(s_hwaddr));
    memcpy(s_rndis_host_mac, dev_mac, sizeof(s_rndis_host_mac));
    s_rndis_host_mac[5] ^= 0x01; // make it different but stable
    s_rndis_host_mac[0] = (uint8_t)((s_rndis_host_mac[0] | 0x02) & 0xFE); // LAA + unicast

    ESP_LOGI(TAG,
             "Starting CherryUSB RNDIS (netif MAC %02X:%02X:%02X:%02X:%02X:%02X, Windows adapter MAC %02X:%02X:%02X:%02X:%02X:%02X)",
             dev_mac[0], dev_mac[1], dev_mac[2], dev_mac[3], dev_mac[4], dev_mac[5],
             s_rndis_host_mac[0], s_rndis_host_mac[1], s_rndis_host_mac[2], s_rndis_host_mac[3], s_rndis_host_mac[4], s_rndis_host_mac[5]);

#ifdef CONFIG_USBDEV_ADVANCE_DESC
    usbd_desc_register(0, &s_usb_desc);
#else
    // Fallback for non-ADVANCE_DESC builds (not expected on ESP-IDF).
    // In this mode CherryUSB expects a single blob containing device+config+strings.
    // Keep the build working by registering only device+config.
    static const uint8_t s_simple_desc[] = {
        USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
        USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER_MA),
        CDC_RNDIS_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MAX_MPS, USB_STRING_RNDIS_INTF_INDEX),
    };
    usbd_desc_register(0, s_simple_desc);
#endif

    struct usbd_interface *intf = usbd_rndis_init_intf(&s_rndis_intf, CDC_OUT_EP, CDC_IN_EP, CDC_INT_EP, s_rndis_host_mac);
    usbd_add_interface(0, intf);

    uintptr_t reg_base = (uintptr_t)ESP_USBD_BASE;
    int usb_ret = usbd_initialize(0, reg_base, usbnet_usbd_event_handler);
    if (usb_ret != 0) {
        ESP_LOGE(TAG, "usbd_initialize failed (%d)", usb_ret);
        return ESP_FAIL;
    }

    return ESP_OK;
}

typedef struct {
    uint32_t dns_ipv4_addr;
} usbnet_sharing_cfg_t;

static void usbnet_enable_sharing_in_tcpip(void *arg)
{
    usbnet_sharing_cfg_t *cfg = (usbnet_sharing_cfg_t *)arg;

    if (cfg && cfg->dns_ipv4_addr != 0) {
        s_upstream_dns.addr = cfg->dns_ipv4_addr;
        // Keep DHCP option 6 as 192.168.7.1; the DNS proxy forwards upstream.
        s_dhcp_cfg.dns = s_ipaddr;
    }

    dhserv_free();
    err_t dh_err = dhserv_init(&s_dhcp_cfg);
    if (dh_err != ERR_OK) {
        ESP_LOGE(TAG, "DHCP server restart failed: dhserv_init err=%d", (int)dh_err);
    }

#if CONFIG_LWIP_IPV4_NAPT
    ip_napt_enable(s_ipaddr.addr, 1);
#else
    ESP_LOGW(TAG, "NAT not enabled: set CONFIG_LWIP_IPV4_NAPT=y to share upstream internet");
#endif

    free(cfg);
}

esp_err_t usb_dev_ethernet_enable_sharing(uint32_t dns_ipv4_addr)
{
    if (!s_lwip_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    usbnet_sharing_cfg_t *cfg = (usbnet_sharing_cfg_t *)calloc(1, sizeof(*cfg));
    if (!cfg) {
        return ESP_ERR_NO_MEM;
    }
    cfg->dns_ipv4_addr = dns_ipv4_addr;

    err_t cb_err = tcpip_callback_with_block(usbnet_enable_sharing_in_tcpip, cfg, 1);
    if (cb_err != ERR_OK) {
        free(cfg);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "USB RNDIS sharing enabled (NAT + DHCP DNS)");
    return ESP_OK;
}
