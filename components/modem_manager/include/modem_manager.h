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

#ifdef __cplusplus
extern "C" {
#endif

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
 * This function must be called before any other modem operations. It ensures
 * the modem is ready for communication and properly configured for high-speed
 * data transfer with reliable flow control.
 *
 * @return ESP_OK on successful initialization
 * @return ESP_FAIL if modem fails to respond or initialize
 * @return ESP_ERR_TIMEOUT if modem initialization times out
 * @return ESP_ERR_INVALID_STATE if modem is not in expected state
 *
 * @note This function may take several seconds to complete as it waits for
 *       the modem to power up and respond to commands.
 * @note Should only be called once during system initialization.
 *
 * @see https://www.quectel.com/product/lte-bg95-series/
 *
 * Example usage:
 * @code
 * esp_err_t ret = modem_mgr_init();
 * if (ret != ESP_OK) {
 *     ESP_LOGE(TAG, "Failed to initialize modem: %s", esp_err_to_name(ret));
 *     return ret;
 * }
 * ESP_LOGI(TAG, "Modem initialized successfully");
 * @endcode
 */
esp_err_t modem_mgr_init(void);

#ifdef __cplusplus
}
#endif

#endif // MODEM_MANAGER_H
