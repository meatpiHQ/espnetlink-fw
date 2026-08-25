/**
 * @file usbnet_class.h
 * @brief Runtime USB Ethernet class selection: CDC-NCM (default — in-box
 *        on Windows 11/macOS/Linux) or RNDIS (legacy Windows 10). Both are
 *        CherryUSB device classes with mirror-image APIs; everything below
 *        the eth_rx/eth_tx line (netif, DHCP, NAT, DNS proxy) is shared.
 *        The class is picked ONCE, before usbnet_usb_start(), from the
 *        usb_dev_ethernet settings descriptor (`class`) — switching it is
 *        a reboot-to-apply operation like every setting (standard §4).
 */
#pragma once

#include <stdbool.h>

struct pbuf;

typedef enum
{
    USBNET_CLASS_KIND_NCM = 0,
    USBNET_CLASS_KIND_RNDIS,
} usbnet_class_kind_t;

/* Select the class BEFORE usbnet_usb_start() (descriptor tables, interface
 * registration and the event model all key off it). Default NCM. */
void usbnet_class_select(usbnet_class_kind_t kind);
usbnet_class_kind_t usbnet_class_kind(void);
const char *usbnet_class_name(void);

/* Class-dispatched data path (usbnet_lwip.c contract): eth_rx() returns one
 * frame-pbuf per call (NULL when drained); eth_tx(p) returns 0 /
 * -USB_ERR_BUSY (caller retries after tx-done) / other negative (drop). */
struct pbuf *usbnet_class_eth_rx(void);
int usbnet_class_eth_tx(struct pbuf *p);

/* RNDIS: device-side carrier notification. NCM: no-op — its link state is
 * host-driven (data-interface alt setting → usbd_cdc_ncm_link_event). */
void usbnet_class_set_connect(bool connect);

/* usbnet_ncm.c adapts the NTB-level NCM class APIs to the pbuf contract */
struct pbuf *usbnet_ncm_eth_rx(void);
int usbnet_ncm_eth_tx(struct pbuf *p);
void usbnet_ncm_start(void);
void usbnet_ncm_stop(void);
