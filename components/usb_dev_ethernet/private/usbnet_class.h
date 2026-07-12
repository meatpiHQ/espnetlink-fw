/**
 * @file usbnet_class.h
 * @brief Compile-time USB Ethernet class selection (Kconfig USBNET_CLASS):
 *        CDC-ECM (default — in-box on Windows 11/macOS/Linux) or RNDIS
 *        (legacy Windows; deprecated by Microsoft). Both are CherryUSB
 *        device classes with mirror-image APIs; everything below the
 *        eth_rx/eth_tx line (netif, DHCP, NAT, DNS proxy) is shared.
 */
#pragma once

#include <stdbool.h>

#include "sdkconfig.h"

#if CONFIG_USBNET_CLASS_NCM

#include "usbd_cdc_ncm.h"

#include "lwip/pbuf.h"

#define USBNET_CLASS_NAME "CDC-NCM"

/* usbnet_ncm.c adapts the NTB-level class APIs to the pbuf contract */
struct pbuf *usbnet_ncm_eth_rx(void);
int usbnet_ncm_eth_tx(struct pbuf *p);
void usbnet_ncm_start(void);
void usbnet_ncm_stop(void);

#define usbnet_class_eth_rx() usbnet_ncm_eth_rx()
#define usbnet_class_eth_tx(p) usbnet_ncm_eth_tx(p)

static inline void usbnet_class_set_connect(bool connect)
{
    /* NCM link state is host-driven (data-interface alt setting); the
     * class raises usbd_cdc_ncm_link_event() — nothing to assert here. */
    (void)connect;
}

#elif CONFIG_USBNET_CLASS_ECM

#include "usbd_cdc_ecm.h"

#define USBNET_CLASS_NAME "CDC-ECM"

#define usbnet_class_eth_rx() usbd_cdc_ecm_eth_rx()
#define usbnet_class_eth_tx(p) usbd_cdc_ecm_eth_tx(p)

static inline void usbnet_class_set_connect(bool connect)
{
    /* ECM reports a link speed pair with the connect notification */
    uint32_t speed[2] = { 12000000, 12000000 }; /* USB FS line rate */

    (void)usbd_cdc_ecm_set_connect(connect, speed);
}

#else /* CONFIG_USBNET_CLASS_RNDIS */

#include "usbd_rndis.h"

#define USBNET_CLASS_NAME "RNDIS"

#define usbnet_class_eth_rx() usbd_rndis_eth_rx()
#define usbnet_class_eth_tx(p) usbd_rndis_eth_tx(p)

static inline void usbnet_class_set_connect(bool connect)
{
    (void)usbd_rndis_set_connect(connect);
}

#endif
