#include "private/usbnet_dhcp_debug.h"

#if USBNET_DHCP_DEBUG

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

static const char *TAG = "usb_dev_eth";

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
    switch (t)
    {
        case 1:
            return "DISCOVER";
        case 2:
            return "OFFER";
        case 3:
            return "REQUEST";
        case 4:
            return "DECLINE";
        case 5:
            return "ACK";
        case 6:
            return "NAK";
        case 7:
            return "RELEASE";
        case 8:
            return "INFORM";
        default:
            return "?";
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

void usbnet_debug_dhcp_pbuf(const char *dir, struct pbuf *p)
{
    static uint32_t s_tx_logged = 0;
    static uint32_t s_rx_logged = 0;

    if (dir && dir[0] == 'T')
    {
        if (s_tx_logged++ > 25)
        {
            return;
        }
    }
    else
    {
        if (s_rx_logged++ > 50)
        {
            return;
        }
    }

    const uint16_t want = 600;
    uint16_t cap = (p->tot_len < want) ? (uint16_t)p->tot_len : want;
    uint8_t buf[600];
    (void)memset(buf, 0, sizeof(buf));
    (void)pbuf_copy_partial(p, buf, cap, 0);

    if (cap < USBNET_ETH_HDR_LEN + 20 + 8)
    {
        return;
    }

    uint16_t eth_type = usbnet_be16(&buf[12]);
    if (eth_type != 0x0800)
    {
        return;
    }

    const uint8_t *ip = &buf[USBNET_ETH_HDR_LEN];
    uint8_t ver_ihl = ip[0];
    if ((ver_ihl >> 4) != 4)
    {
        return;
    }

    uint8_t ihl = (uint8_t)((ver_ihl & 0x0F) * 4);
    if (ihl < 20 || cap < (uint16_t)(USBNET_ETH_HDR_LEN + ihl + 8))
    {
        return;
    }

    if (ip[9] != 17)
    {
        return;
    }

    char src_ip[16] = { 0 };
    char dst_ip[16] = { 0 };
    usbnet_ipv4_to_str(usbnet_be32(&ip[12]), src_ip);
    usbnet_ipv4_to_str(usbnet_be32(&ip[16]), dst_ip);

    const uint8_t *udp = ip + ihl;
    uint16_t sport = usbnet_be16(&udp[0]);
    uint16_t dport = usbnet_be16(&udp[2]);
    uint16_t ulen = usbnet_be16(&udp[4]);

    if (!((sport == 67 || sport == 68) && (dport == 67 || dport == 68)))
    {
        return;
    }

    const uint8_t *bootp = udp + 8;
    uint16_t udp_payload_avail = (cap > (uint16_t)(USBNET_ETH_HDR_LEN + ihl + 8))
                                     ? (uint16_t)(cap - (USBNET_ETH_HDR_LEN + ihl + 8))
                                     : 0;

    if (udp_payload_avail < 240)
    {
        ESP_LOGW(TAG,
                 "%s DHCP short: %s:%u -> %s:%u (udp_len=%u, cap_payload=%u)",
                 dir,
                 src_ip,
                 (unsigned)sport,
                 dst_ip,
                 (unsigned)dport,
                 (unsigned)ulen,
                 (unsigned)udp_payload_avail);
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
    char ch[18] = { 0 };
    if (hlen >= 6)
    {
        (void)snprintf(ch,
                       sizeof(ch),
                       "%02X:%02X:%02X:%02X:%02X:%02X",
                       chaddr[0],
                       chaddr[1],
                       chaddr[2],
                       chaddr[3],
                       chaddr[4],
                       chaddr[5]);
    }
    else
    {
        (void)snprintf(ch, sizeof(ch), "-");
    }

    char ci[16] = { 0 };
    char yi[16] = { 0 };
    char si[16] = { 0 };
    usbnet_ipv4_to_str(ciaddr, ci);
    usbnet_ipv4_to_str(yiaddr, yi);
    usbnet_ipv4_to_str(siaddr, si);

    const uint8_t *opts = bootp + 236;
    uint32_t cookie = usbnet_be32(opts);
    if (cookie != 0x63825363u)
    {
        ESP_LOGW(TAG, "%s DHCP bad cookie 0x%08" PRIX32 " (op=%u xid=0x%08" PRIX32 ")", dir, cookie, (unsigned)op, xid);
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

    uint16_t idx = 4;
    while ((uint32_t)(236 + idx) < udp_payload_avail)
    {
        uint8_t code = opts[idx++];
        if (code == 0)
        {
            continue;
        }
        if (code == 255)
        {
            break;
        }
        if ((uint32_t)(236 + idx) >= udp_payload_avail)
        {
            break;
        }
        uint8_t len = opts[idx++];
        if ((uint32_t)(236 + idx + len) > udp_payload_avail)
        {
            break;
        }

        const uint8_t *val = &opts[idx];
        if (code == 53 && len >= 1)
        {
            msg_type = val[0];
        }
        else if (code == 54 && len == 4)
        {
            server_id = usbnet_be32(val);
        }
        else if (code == 1 && len == 4)
        {
            subnet_mask = usbnet_be32(val);
        }
        else if (code == 3 && len >= 4)
        {
            router = usbnet_be32(val);
        }
        else if (code == 6 && len >= 4)
        {
            dns = usbnet_be32(val);
        }
        else if (code == 55 && len > 0)
        {
            for (uint8_t j = 0; j < len; j++)
            {
                if (val[j] == 6)
                {
                    prl_has_dns = true;
                }
                else if (val[j] == 3)
                {
                    prl_has_router = true;
                }
                else if (val[j] == 1)
                {
                    prl_has_subnet = true;
                }
            }
        }

        idx = (uint16_t)(idx + len);
    }

    char sid[16] = { 0 };
    char sm[16] = { 0 };
    char rt[16] = { 0 };
    char dn[16] = { 0 };
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

#endif
