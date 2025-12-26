#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t usb_dev_ethernet_start(void);

// Enables basic "internet sharing" for the USB-NCM interface:
// - Updates DHCP options (router + DNS) and restarts the tiny DHCP server
// - Enables lwIP NAPT using the USB interface IPv4 address (192.168.7.1)
// Call this after the upstream interface (e.g., PPPoS/LTE) has an IP.
// dns_ipv4_addr should be an IPv4 address in lwIP's internal format (same as esp_netif DNS info ip4.addr).
esp_err_t usb_dev_ethernet_enable_sharing(uint32_t dns_ipv4_addr);

#ifdef __cplusplus
}
#endif
