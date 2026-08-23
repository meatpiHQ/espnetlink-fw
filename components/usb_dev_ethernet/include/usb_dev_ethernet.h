#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t usb_dev_ethernet_start(void);

// Optional CDC-ACM console alongside RNDIS.
// Must be set before usb_dev_ethernet_start().
void usb_dev_ethernet_set_console_enabled(bool enabled);

// Whether the USB link's DHCP server offers this device as the host's
// default ROUTER. Default false: the USB link is then a control channel
// (the host reaches 192.168.7.1 by its subnet route) and the host's own
// internet path is untouched. main sets it true only when the LTE uplink
// is configured to be shared over USB (lte_upstream_pppos.ncm_share) —
// otherwise a PC's default route would silently move onto the slow LTE
// link. Must be set before usb_dev_ethernet_start().
void usb_dev_ethernet_set_router_offer(bool enabled);

// Enables basic "internet sharing" for the USB-NCM interface:
// - Updates DHCP options (router + DNS) and restarts the tiny DHCP server
// - Enables lwIP NAPT using the USB interface IPv4 address (192.168.7.1)
// Call this after the upstream interface (e.g., PPPoS/LTE) has an IP.
// dns_ipv4_addr should be an IPv4 address in lwIP's internal format (same as esp_netif DNS info ip4.addr).
esp_err_t usb_dev_ethernet_enable_sharing(uint32_t dns_ipv4_addr);

#ifdef __cplusplus
}
#endif
