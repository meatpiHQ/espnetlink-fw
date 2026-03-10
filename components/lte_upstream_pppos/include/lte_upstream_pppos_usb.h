#pragma once

/*
 * lte_upstream_pppos_usb.h
 *
 * C interface for creating an esp_modem DCE that communicates with the BG95
 * modem through its native USB CDC-ACM port instead of UART.
 *
 * BG95 USB identifiers:
 *   VID : 0x2C7C  (Quectel)
 *   PID : 0x0095  (BG95 series)
 *   Interface 2: AT command / modem data port  <-- use for PPPoS
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "esp_netif.h"
#include "esp_modem_config.h"
#include "esp_modem_c_api_types.h"

#define LTE_USB_VID_QUECTEL  0x2C7Cu
#define LTE_USB_PID_BG95     0x0095u
#define LTE_USB_IFACE_MODEM  2u      /* AT + data (PPP) interface */

/**
 * @brief Create an esp_modem DCE backed by the BG95 USB CDC-ACM port.
 *
 * Installs the USB Host Library and CDC-ACM driver (idempotent – safe to call
 * even if another part of the firmware already installed them).
 * Blocks until the BG95 USB device enumerates or conn_timeout_ms elapses.
 *
 * @param dte_config      DTE config.  dte_buffer_size / task_stack_size /
 *                        task_priority are used; uart_config fields are ignored.
 * @param dce_config      DCE config (APN, etc.)
 * @param netif           PPP netif
 * @param usb_sel_gpio    GPIO number to control a USB MUX/select line
 *                        (-1 = not used, no GPIO is driven)
 * @param usb_sel_level   Logic level to assert on usb_sel_gpio
 *                        (true = HIGH, false = LOW)
 * @param vid             USB Vendor ID of the modem  (LTE_USB_VID_QUECTEL)
 * @param pid             USB Product ID of the modem (LTE_USB_PID_BG95)
 * @param interface_num   CDC-ACM interface index for the modem AT/data port
 *                        (LTE_USB_IFACE_MODEM = 2)
 * @param conn_timeout_ms Timeout waiting for device enumeration in ms;
 *                        0 = use default of 15 000 ms
 *
 * @return Opaque esp_modem_dce_t * compatible with all esp_modem_* C API calls,
 *         or NULL on error.
 */
esp_modem_dce_t *lte_pppos_usb_create_dce(
    const esp_modem_dte_config_t *dte_config,
    const esp_modem_dce_config_t *dce_config,
    esp_netif_t                  *netif,
    int8_t                        usb_sel_gpio,
    bool                          usb_sel_level,
    uint16_t                      vid,
    uint16_t                      pid,
    uint8_t                       interface_num,
    uint32_t                      conn_timeout_ms);

#ifdef __cplusplus
}
#endif
