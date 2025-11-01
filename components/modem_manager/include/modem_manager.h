/*
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

/**
 * @file modem_manager.h
 * @brief Modem Manager interface for controlling the BG95 cellular modem
 *
 * This module provides initialization and control functions for the BG95 cellular modem,
 * handling modem configuration, communication setup, and operational mode management.
 * The modem manager ensures proper initialization sequence and configures high-speed
 * serial communication with hardware flow control.
 *
 * Key Features:
 * - BG95 modem initialization and power management
 * - High-speed UART configuration (3,000,000 baud)
 * - RTS/CTS hardware flow control support
 * - Command mode configuration and verification
 * - Error handling and recovery mechanisms
 */

#ifndef MODEM_MANAGER_H
#define MODEM_MANAGER_H

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Modem Manager configuration
 *
 * Pass hardware pin mapping and UART port selection instead of relying on
 * compile-time macros. This removes the dependency on hw_config.h from the
 * modem manager component.
 */
typedef struct {
	// UART selection and pins
	uart_port_t uart_port;    // e.g. UART_NUM_1
	gpio_num_t  tx_pin;       // TXD GPIO
	gpio_num_t  rx_pin;       // RXD GPIO
	gpio_num_t  rts_pin;      // RTS GPIO (required for HW flow control)
	gpio_num_t  cts_pin;      // CTS GPIO (required for HW flow control)

	// Modem control pins
	gpio_num_t  pwr_key_pin;  // Active-low power key control
	gpio_num_t  status_pin;   // Status indicator pin from modem
	gpio_num_t  dtr_pin;      // Optional DTR pin (GPIO_NUM_NC if unused)

	// Polarity of the status pin: true if level LOW means modem is ON
	bool        status_on_is_low; // Typically true for BG95
} modem_mgr_config_t;

/**
 * @brief Initialize the BG95 cellular modem
 *
 * Performs complete initialization sequence for the BG95 modem including:
 * - Power-up and reset sequence
 * - Verification that modem is in AT command mode
 * - UART configuration at 3,000,000 baud rate
 * - RTS/CTS hardware flow control enablement
 * - Basic modem health check and readiness verification
 *
 * @param cfg Pointer to configuration describing UART port and GPIO pins.
 *            Must remain valid for the duration of the call. The content is
 *            copied internally at the start of the function.
 *
 * @return ESP_OK on successful initialization
 * @return ESP_FAIL if modem fails to respond or initialize
 * @return ESP_ERR_TIMEOUT if modem initialization times out
 *
 * @note This function may take several seconds to complete as it waits for
 *       the modem to power up and respond to commands.
 * @note Should only be called once during system initialization.
 */
esp_err_t modem_mgr_init(const modem_mgr_config_t *cfg);

#ifdef __cplusplus
}
#endif

#endif // MODEM_MANAGER_H
