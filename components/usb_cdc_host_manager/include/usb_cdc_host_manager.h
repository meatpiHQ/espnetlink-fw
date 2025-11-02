/**
 * @file usb_cdc_host_manager.h
 * @brief USB CDC Host Manager for multi-interface CDC-ACM devices
 *
 * This file is part of the WiCAN project.
 *
 * Copyright (C) 2022  Meatpi Electronics.
 * Written by Ali Slim <ali@meatpi.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_host.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Constants and Definitions
 * ========================================================================= */

/** Common CH34x Vendor ID used by CH342 with dual interfaces */
#define USB_VID_QINHENG     0x1A86

/** CH342 Product ID (dual-port USB to serial converter) */
#define USB_PID_CH342       0x55D2

/** Interface index for first CDC interface */
#define USB_CDC_IFACE_0     0

/** Interface index for second CDC interface */
#define USB_CDC_IFACE_2     2

/* ============================================================================
 * Type Definitions
 * ========================================================================= */

/**
 * @brief USB CDC RX data callback function type
 *
 * @param data      Pointer to received data buffer
 * @param data_len  Length of received data in bytes
 * @param arg       User-provided argument passed during open
 * @return true to continue receiving, false to stop
 */
typedef bool (*usb_cdc_rx_cb_t)(const uint8_t *data, size_t data_len, void *arg);

/**
 * @brief USB CDC device event callback function type
 *
 * @param event     Pointer to event data structure
 * @param user_ctx  User-provided context passed during open
 */
typedef void (*usb_cdc_event_cb_t)(const cdc_acm_host_dev_event_data_t *event, void *user_ctx);

/* ============================================================================
 * Public API Functions
 * ========================================================================= */

/**
 * @brief Initialize USB Host stack and CDC-ACM driver
 *
 * Initializes the USB Host library, starts the host event task, and
 * installs the CDC-ACM driver (idempotent operation).
 *
 * @return
 *     - ESP_OK on success
 *     - ESP_ERR_* error code on failure
 */
esp_err_t usb_cdc_mgr_init(void);

/**
 * @brief Open a USB CDC device on a specific interface
 *
 * Opens a CDC-ACM device matching the specified VID/PID on the given interface
 * index, with configured buffers and callbacks. Maintains separate context per
 * interface to support multi-port devices.
 *
 * @param vid                   USB Vendor ID
 * @param pid                   USB Product ID
 * @param interface_idx         Interface index to open (e.g., 0 or 2)
 * @param connection_timeout_ms Maximum time to wait for device connection
 * @param out_buffer_size       Size of TX buffer in bytes
 * @param in_buffer_size        Size of RX buffer in bytes
 * @param event_cb              Event callback function (can be NULL)
 * @param rx_cb                 RX data callback function (can be NULL)
 * @param user_arg              User argument passed to callbacks
 *
 * @return
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if parameters are invalid
 *     - ESP_ERR_TIMEOUT if connection timeout occurs
 *     - ESP_ERR_* other error codes on failure
 */
esp_err_t usb_cdc_mgr_open(uint16_t vid,
                           uint16_t pid,
                           uint8_t interface_idx,
                           uint32_t connection_timeout_ms,
                           size_t out_buffer_size,
                           size_t in_buffer_size,
                           usb_cdc_event_cb_t event_cb,
                           usb_cdc_rx_cb_t rx_cb,
                           void *user_arg);

/**
 * @brief Configure serial line parameters for a CDC interface
 *
 * Sets baud rate, stop bits, parity, and data bits for the specified interface.
 *
 * @param interface_idx Interface index (e.g., 0 or 2)
 * @param baudrate      Baud rate in bits per second
 * @param stop_bits     Number of stop bits (0=1bit, 1=1.5bits, 2=2bits)
 * @param parity        Parity setting (0=None, 1=Odd, 2=Even, 3=Mark, 4=Space)
 * @param data_bits     Number of data bits (5-8)
 *
 * @return
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if interface not opened or parameters invalid
 *     - ESP_ERR_* other error codes on failure
 */
esp_err_t usb_cdc_mgr_set_line_coding(uint8_t interface_idx,
                                      uint32_t baudrate,
                                      uint8_t stop_bits,
                                      uint8_t parity,
                                      uint8_t data_bits);

/**
 * @brief Transmit data via specified CDC interface (blocking)
 *
 * Sends data through the specified interface with timeout.
 *
 * @param interface_idx Interface index (e.g., 0 or 2)
 * @param data          Pointer to data buffer to transmit
 * @param len           Length of data in bytes
 * @param timeout_ms    Maximum time to wait for transmission in milliseconds
 *
 * @return
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if interface not opened or data is NULL
 *     - ESP_ERR_TIMEOUT if transmission timeout occurs
 *     - ESP_ERR_* other error codes on failure
 */
esp_err_t usb_cdc_mgr_tx(uint8_t interface_idx,
                         const uint8_t *data,
                         size_t len,
                         uint32_t timeout_ms);

/**
 * @brief Close all opened CDC devices and release resources
 *
 * Closes all opened interfaces and frees associated resources.
 * Safe to call even if no devices are opened.
 */
void usb_cdc_mgr_close_all(void);

/**
 * @brief Close a specific CDC interface if opened
 *
 * @param interface_idx Interface index (e.g., 0 or 2)
 * @return ESP_OK if closed or not open, ESP_ERR_INVALID_ARG for bad index
 */
esp_err_t usb_cdc_mgr_close(uint8_t interface_idx);

#ifdef __cplusplus
}
#endif
