#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

#include "dhserver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    struct netif usbnetif;

    bool lwip_ready;
    bool lwip_started;

    ip4_addr_t ipaddr;
    ip4_addr_t netmask;
    ip4_addr_t gw;
    ip4_addr_t upstream_dns;

    dhcp_entry_t dhcp_entries[4];
    dhcp_config_t dhcp_cfg;

    uint8_t hwaddr[6];
    uint8_t rndis_host_mac[6];

    /* Set by usb_dev_ethernet_enable_sharing() when called before lwip_ready.
     * usbnet_lwip_start_in_tcpip() applies it as soon as the interface is up. */
    bool     pending_share;
    ip4_addr_t pending_dns;
} usbnet_state_t;

usbnet_state_t *usbnet_state(void);

#ifdef __cplusplus
}
#endif
