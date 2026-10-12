#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

void usbnet_usb_set_console_enabled(bool enabled);
bool usbnet_usb_get_console_enabled(void);

/* false = the CDC console alone (no Ethernet interfaces, no netif); set
 * before usbnet_usb_start(). Default true. */
void usbnet_usb_set_ethernet_enabled(bool enabled);
bool usbnet_usb_get_ethernet_enabled(void);

void usbnet_usb_start(void);

// Blocks until the previous USB IN transfer completes (best-effort).
// Intended for lwIP linkoutput backpressure handling.
bool usbnet_usb_wait_tx_done(TickType_t ticks);

// ISR-safe notifications from the class data callbacks (any class):
// wake the rx drain task / release the tx-done semaphore / drive the
// link state machine (NCM's host-controlled alt-setting link).
void usbnet_usb_notify_rx(void);
void usbnet_usb_notify_tx_done(void);
void usbnet_usb_notify_link(bool up);

#ifdef __cplusplus
}
#endif
