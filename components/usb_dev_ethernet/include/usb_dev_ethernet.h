#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    USB_DEV_ETHERNET_CLASS_NCM = 0,
    USB_DEV_ETHERNET_CLASS_RNDIS,
} usb_dev_ethernet_class_t;

// Settings descriptor "usb_dev_ethernet" v1 (`class`: ncm|rndis, default
// per Kconfig USBNET_CLASS). Register with the other settings init steps;
// the boot apply decides which class usb_dev_ethernet_start() presents.
esp_err_t usb_dev_ethernet_register_settings(void);
usb_dev_ethernet_class_t usb_dev_ethernet_class(void);

esp_err_t usb_dev_ethernet_start(void);

// Optional CDC-ACM console alongside the Ethernet function.
// Must be set before usb_dev_ethernet_start().
void usb_dev_ethernet_set_console_enabled(bool enabled);

// Whether the composite presents the USB Ethernet function at all
// (default true). false = the CDC-ACM console ALONE at interfaces 0 and 1:
// no NCM/RNDIS interfaces, no USB netif, no DHCP server, no DNS proxy
// (wifi_modem usb_mode=console, 2026-10-12: the WiFi AP is the product and
// the console is the owner's way back in). The console must be enabled in
// that case. Must be set before usb_dev_ethernet_start().
void usb_dev_ethernet_set_ethernet_enabled(bool enabled);

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

/** Whether a software restart first drops the USB device off the bus
 *  (default true: a warm reboot then looks like an unplug to the host, so
 *  no transfer is in flight when the next image enumerates). false for a
 *  restart meant to land in ROM download mode, where the ROM's OTG
 *  download device needs the controller still connected. */
void usb_dev_ethernet_set_restart_detach(bool detach);

#ifdef __cplusplus
}
#endif
