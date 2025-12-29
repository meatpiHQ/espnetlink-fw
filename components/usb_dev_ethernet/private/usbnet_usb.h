#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

void usbnet_usb_set_console_enabled(bool enabled);
bool usbnet_usb_get_console_enabled(void);

void usbnet_usb_start(void);

// Blocks until the previous RNDIS IN transfer completes (best-effort).
// Intended for lwIP linkoutput backpressure handling.
bool usbnet_usb_wait_tx_done(TickType_t ticks);

// CherryUSB callbacks (implemented by this component)
void usbd_rndis_data_recv_done(uint32_t len);
void usbd_rndis_data_send_done(uint32_t len);

#ifdef __cplusplus
}
#endif
