/*
 * This file is part of the ESPNetLink project.
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
 * @file modem_manager.c
 * @brief Modem Manager implementation for BG95 cellular modem control
 *
 * This module provides comprehensive initialization and configuration for the Quectel BG95
 * cellular modem. It handles the complete power-up sequence, AT command interface setup,
 * UART configuration at high speed (3 Mbaud), and hardware flow control management.
 *
 * Key Responsibilities:
 * - GPIO initialization for modem control (PWR_KEY, STATUS, DTR)
 * - Power state management with graceful power-off and power-on sequences
 * - UART driver setup with configurable baud rates and flow control
 * - AT command interface with automatic recovery mechanisms
 * - Baud rate negotiation and configuration persistence
 * - Hardware flow control (RTS/CTS) configuration
 * - Modem firmware version detection and logging
 *
 * Initialization Flow:
 * 1. Configure GPIOs (PWR_KEY, STATUS, DTR)
 * 2. Check modem status and perform power cycle if needed
 * 3. Establish AT command access at various baud rates (with fallback)
 * 4. Configure desired baud rate (3,000,000) and hardware flow control
 * 5. Save configuration to modem's non-volatile memory
 * 6. Query and log modem firmware version
 * 7. Clean up UART driver (for subsequent use by other modules)
 *
 * Recovery Mechanisms:
 * - Automatic baud rate detection (3M, 115200 with/without flow control)
 * - Safe power reset if AT commands unresponsive
 * - Multiple retry attempts for AT command synchronization
 * - Graceful degradation and error reporting
 *
 * @see hw_config.h for GPIO pin definitions
 * @see modem_manager.h for public interface
 */

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "modem_manager.h"
#include "modem_manager_private.h"

static const char *TAG = "modem_manager"; /* §10: TAG == component dir */

// Active configuration is copied from user-provided cfg in modem_mgr_init
modem_mgr_config_t s_cfg;

/*
 * =============================================================================
 * TIMING CONSTANTS
 * =============================================================================
 */

/** PWR_KEY pulse duration to power on modem (BG95 spec: 500-800ms) */
#define PWRKEY_PULSE_MS_ON       800

/** PWR_KEY pulse duration for graceful power-off (longer for clean shutdown) */
#define PWRKEY_PULSE_MS_OFF      1500

/** Maximum time to wait for STATUS pin to indicate modem is ON (low level) */
#define STATUS_WAIT_ON_MS        30000

/** Maximum time to wait for STATUS pin to indicate modem is OFF (high level) */
#define STATUS_WAIT_OFF_MS       30000

/*
 * =============================================================================
 * UART CONFIGURATION CONSTANTS
 * =============================================================================
 */

/** UART receive buffer size (must handle high-speed data bursts) */
#define UART_RX_BUF_SIZE         4096

/** UART transmit buffer size */
#define UART_TX_BUF_SIZE         2048

/*
 * =============================================================================
 * CMUX FCS (GSM 07.10 CRC-8, reversed polynomial 0xE0)
 * =============================================================================
 */

/**
 * @brief Compute GSM 07.10 FCS (Frame Check Sequence) for CMUX frames
 * @param data Pointer to bytes to checksum (address, control, and optionally length)
 * @param len  Number of bytes
 * @return FCS byte (complement of CRC)
 */
static uint8_t cmux_calc_fcs(const uint8_t *data, size_t len)
{
    uint8_t fcs = 0xFF;
    for (size_t i = 0; i < len; i++)
    {
        uint8_t byte = data[i];
        for (int bit = 0; bit < 8; bit++)
        {
            if ((fcs ^ byte) & 1)
                fcs = (fcs >> 1) ^ 0xE0;
            else
                fcs >>= 1;
            byte >>= 1;
        }
    }
    return (uint8_t)~fcs;
}

/*
 * =============================================================================
 * UTILITY FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Get current time in milliseconds
 * @return Current time in milliseconds since system start
 */
int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

/**
 * @brief Sleep for specified milliseconds
 * @param ms Number of milliseconds to sleep
 */
void sleep_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/*
 * =============================================================================
 * GPIO CONTROL FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Initialize GPIO pins for modem control
 *
 * Configures three essential GPIO pins:
 * - PWR_KEY: Output pin for power control (active high, idle low)
 * - STATUS: Input pin to monitor modem power state (low when ON)
 * - DTR: Output pin kept high to prevent modem sleep mode
 */
static void modem_gpio_init(void)
{
    // PWR_KEY (active low)
    gpio_reset_pin(s_cfg.pwr_key_pin);
    gpio_set_direction(s_cfg.pwr_key_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(s_cfg.pwr_key_pin, 0); 

    // STATUS (input) - low when modem is ON per hw_config.h comment
    gpio_reset_pin(s_cfg.status_pin);
    gpio_set_direction(s_cfg.status_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(s_cfg.status_pin, GPIO_FLOATING);

    // DTR (keep high to avoid sleep)
    if (s_cfg.dtr_pin != GPIO_NUM_NC && (int)s_cfg.dtr_pin >= 0)
    {
        gpio_reset_pin(s_cfg.dtr_pin);
        gpio_set_direction(s_cfg.dtr_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(s_cfg.dtr_pin, 1);
    }
}

/**
 * @brief Check if modem is currently powered on
 * @return true if modem is ON (STATUS pin is low), false otherwise
 * @note BG95 STATUS pin is active low: 0 = ON, 1 = OFF
 */
static bool modem_status_is_on(void)
{
    int level = gpio_get_level(s_cfg.status_pin);
    // Respect configured polarity
    return s_cfg.status_on_is_low ? (level == 0) : (level != 0);
}

/**
 * @brief Generate a PWR_KEY pulse to toggle modem power state
 * @param ms Duration of the pulse in milliseconds
 * @note PWR_KEY is active high. Different pulse durations trigger different actions:
 *       - 500-800ms: Toggle power state (ON->OFF or OFF->ON)
 *       - 1500ms+: Graceful power-off with proper shutdown sequence
 */
static void modem_pwrkey_pulse(uint32_t ms)
{
    // Active low pulse
    gpio_set_level(s_cfg.pwr_key_pin, 1);
    sleep_ms(ms);
    gpio_set_level(s_cfg.pwr_key_pin, 0);
}

/**
 * @brief Wait for modem to reach desired power state
 * @param want_on true to wait for ON state, false to wait for OFF state
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return ESP_OK if desired state reached, ESP_ERR_TIMEOUT if timeout occurs
 */
static esp_err_t modem_wait_status(bool want_on, uint32_t timeout_ms)
{
    const int64_t deadline = now_ms() + timeout_ms;
    while (now_ms() < deadline)
    {
        if (modem_status_is_on() == want_on)
        {
            return ESP_OK;
        }
        sleep_ms(50);
    }
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Perform a safe modem reset sequence with graceful shutdown
 *
 * This function ensures the modem is in a known good state by:
 * 1. If modem is ON: Request graceful power-off (ensures clean shutdown)
 * 2. Wait for modem to fully power off
 * 3. Power modem back ON
 * 4. Wait for modem to be fully operational
 *
 * This is safer than a hard reset as it allows the modem to:
 * - Save any configuration changes
 * - Properly close network connections
 * - Clean up internal state
 *
 * @return ESP_OK if modem successfully powered on, ESP_ERR_TIMEOUT if power-on fails
 * @note May take up to 60 seconds (30s off + 30s on) in worst case
 */
esp_err_t modem_safe_reset_sequence(void)
{
    if (modem_status_is_on())
    {
        ESP_LOGI(TAG, "Modem appears ON; requesting graceful power-off");
        modem_pwrkey_pulse(PWRKEY_PULSE_MS_OFF);
        esp_err_t err = modem_wait_status(false, STATUS_WAIT_OFF_MS);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "Timeout waiting modem OFF; proceeding to power on sequence anyway");
        }
    }

    ESP_LOGI(TAG, "Powering modem ON");
    modem_pwrkey_pulse(PWRKEY_PULSE_MS_ON);
    return modem_wait_status(true, STATUS_WAIT_ON_MS);
}

/*
 * =============================================================================
 * UART CONFIGURATION FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Configure and install UART driver for modem communication
 * @param baud Baud rate (e.g., 115200, 3000000)
 * @param hwfc_on true to enable RTS/CTS hardware flow control, false to disable
 * @return ESP_OK on success, error code otherwise
 * @note This function installs a new UART driver. Call modem_uart_teardown() first
 *       if driver is already installed.
 */
esp_err_t modem_uart_setup(int baud, bool hwfc_on)
{
    const uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = hwfc_on ? UART_HW_FLOWCTRL_CTS_RTS : UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 64,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Helper to map GPIO_NUM_NC to UART_PIN_NO_CHANGE
    int tx = (s_cfg.tx_pin == GPIO_NUM_NC) ? UART_PIN_NO_CHANGE : (int)s_cfg.tx_pin;
    int rx = (s_cfg.rx_pin == GPIO_NUM_NC) ? UART_PIN_NO_CHANGE : (int)s_cfg.rx_pin;
    int rts = (s_cfg.rts_pin == GPIO_NUM_NC) ? UART_PIN_NO_CHANGE : (int)s_cfg.rts_pin;
    int cts = (s_cfg.cts_pin == GPIO_NUM_NC) ? UART_PIN_NO_CHANGE : (int)s_cfg.cts_pin;

    /* No panic paths in components (standard §3): a UART setup failure
     * degrades the modem, never boot-loops the device. */
    ESP_RETURN_ON_ERROR(uart_driver_install(s_cfg.uart_port, UART_RX_BUF_SIZE,
                                            UART_TX_BUF_SIZE, 0, NULL, 0),
                        TAG, "uart_driver_install");
    ESP_RETURN_ON_ERROR(uart_param_config(s_cfg.uart_port, &cfg), TAG,
                        "uart_param_config");
    ESP_RETURN_ON_ERROR(uart_set_pin(s_cfg.uart_port, tx, rx, rts, cts), TAG,
                        "uart_set_pin");

    if (hwfc_on)
    {
        ESP_RETURN_ON_ERROR(uart_set_hw_flow_ctrl(s_cfg.uart_port,
                                                  UART_HW_FLOWCTRL_CTS_RTS, 64),
                            TAG, "uart_set_hw_flow_ctrl");
    }
    else
    {
        ESP_RETURN_ON_ERROR(uart_set_hw_flow_ctrl(s_cfg.uart_port,
                                                  UART_HW_FLOWCTRL_DISABLE, 0),
                            TAG, "uart_set_hw_flow_ctrl");
    }

    return ESP_OK;
}

/**
 * @brief Uninstall UART driver to free resources
 * @note Only uninstalls if driver is currently installed to avoid error messages
 */
void modem_uart_teardown(void)
{
    // Delete driver only if installed to avoid noisy "ALREADY NULL" logs
    if (uart_is_driver_installed(s_cfg.uart_port))
    {
        ESP_LOGI(TAG, "Tearing down UART driver");
        uart_driver_delete(s_cfg.uart_port);
    }
}

/**
 * @brief Change UART baud rate on the fly
 * @param baud New baud rate to set
 * @note Does not require driver reinstall, changes rate dynamically
 */
void modem_uart_set_baud(int baud)
{
    ESP_LOGI(TAG, "Setting UART baud rate to %d", baud);
    uart_set_baudrate(s_cfg.uart_port, baud);
}

/*
 * =============================================================================
 * PUBLIC API IMPLEMENTATION
 * =============================================================================
 */

/**
 * @brief Initialize the BG95 cellular modem (PUBLIC API)
 *
 * Complete initialization sequence:
 * 1. Initialize GPIO pins (PWR_KEY, STATUS, DTR)
 * 2. Check modem power status
 *    - If ON: Perform safe reset to ensure command mode (not data/CMUX mode)
 *    - If OFF: Power on modem
 * 3. Establish AT command access with automatic recovery
 *    - Try multiple baud rates and flow control combinations
 *    - Perform power cycle if needed
 * 4. Configure optimal settings:
 *    - Hardware flow control (RTS/CTS)
 *    - High-speed baud rate (3 Mbaud)
 * 5. Save configuration to modem NV memory
 * 6. Query and log firmware version
 * 7. Clean up UART driver (for use by other modules)
 *
 * @return ESP_OK on success
 * @return ESP_FAIL if unable to establish AT communication
 *
 * @note This function may take 30-60 seconds to complete in worst case
 *       (power cycling and multiple retry attempts)
 * @note UART driver is torn down at end, allowing other modules to use it
 * @note Should only be called once during system initialization
 *
 * @see modem_manager.h for detailed API documentation
 */
esp_err_t modem_mgr_init(const modem_mgr_config_t *cfg)
{
    // Copy configuration
    if (cfg == NULL) {
        ESP_LOGE(TAG, "modem_mgr_init: cfg is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;
    // Default for status polarity if caller forgot to fill
    // (BG95 uses LOW = ON)
    // Note: We can't detect uninitialized bool, so keep caller value.

    ESP_LOGI(TAG, "Initializing modem manager");

    modem_gpio_init();

    bool at_ready = false;

    if (modem_status_is_on())
    {
        // Modem is ON – try quick AT probes (~600 ms each instead of ~13 s)
        // to avoid a power cycle that kills existing LTE registration.
        ESP_LOGI(TAG, "Modem status: ON -> trying AT access without reset");
        if (try_at_quick(BAUD_REQ, true) || try_at_quick(BAUD_REQ, false) ||
            try_at_quick(115200, false)  || try_at_quick(115200, true))
        {
            ESP_LOGI(TAG, "AT access OK – skipping power cycle to preserve registration");
            at_ready = true;
        }
        else
        {
            // Modem may be in PPP data mode or CMUX.
            // Step 1: Try DTR toggle to exit data/CMUX mode.
            //         BG95 with AT&D1 interprets DTR drop as "return to command mode".
            //         This is instantaneous compared to +++ (which needs 2.2 s guard times)
            //         and works in CMUX mode where +++ does not.
            if (s_cfg.dtr_pin != GPIO_NUM_NC && (int)s_cfg.dtr_pin >= 0)
            {
                ESP_LOGW(TAG, "AT unresponsive – toggling DTR to exit data/CMUX mode");
                gpio_set_level(s_cfg.dtr_pin, 0);  // drop DTR
                sleep_ms(200);
                gpio_set_level(s_cfg.dtr_pin, 1);  // raise DTR
                sleep_ms(500);
                if (try_at_quick(BAUD_REQ, true) || try_at_quick(115200, false))
                {
                    ESP_LOGI(TAG, "DTR toggle succeeded – preserving registration");
                    at_write("ATH\r");
                    at_expect_ok(1000);
                    at_ready = true;
                }
            }

            // Step 2: Send CMUX CLD (Close Down) frame to exit multiplexer mode.
            //         DTR and +++ do not work in CMUX – only a proper GSM 07.10
            //         CLD command or DISC on DLCI 0 can close the session.
            if (!at_ready)
            {
                ESP_LOGW(TAG, "AT unresponsive – sending CMUX close-down frame");
                modem_uart_teardown();
                modem_uart_setup(BAUD_REQ, true);

                // CLD (Close Down) in a UIH frame on DLCI 0
                //  Flag=F9  Addr=03(DLCI0,CR=1,EA=1)  Ctrl=EF(UIH)
                //  Len=05(2 bytes,EA=1)  Type=C3(CLD,CR=1,EA=1)  CmdLen=01(0,EA=1)
                //  FCS=xx(over addr+ctrl)  Flag=F9
                uint8_t cld[] = { 0xF9, 0x03, 0xEF, 0x05, 0xC3, 0x01, 0x00, 0xF9 };
                cld[6] = cmux_calc_fcs(&cld[1], 2);  // FCS over address + control
                uart_write_bytes(s_cfg.uart_port, cld, sizeof(cld));
                uart_wait_tx_done(s_cfg.uart_port, pdMS_TO_TICKS(100));
                sleep_ms(300);

                // Also try DISC on DLCI 0 as a fallback
                //  Flag=F9  Addr=03  Ctrl=53(DISC,P=1)  Len=01(0,EA=1)  FCS=xx  Flag=F9
                uint8_t disc[] = { 0xF9, 0x03, 0x53, 0x01, 0x00, 0xF9 };
                disc[4] = cmux_calc_fcs(&disc[1], 3);  // FCS over addr+ctrl+len
                uart_write_bytes(s_cfg.uart_port, disc, sizeof(disc));
                uart_wait_tx_done(s_cfg.uart_port, pdMS_TO_TICKS(100));
                sleep_ms(500);

                // Flush any CMUX response frames and try AT
                uart_flush_input(s_cfg.uart_port);
                if (at_probe_quick())
                {
                    ESP_LOGI(TAG, "CMUX close succeeded – preserving registration");
                    at_ready = true;
                }
            }

            // Step 3: Try +++ escape sequence (works for plain PPP data mode).
            if (!at_ready)
            {
                ESP_LOGW(TAG, "AT unresponsive – trying +++ escape to exit data mode");
                static const int escape_bauds[] = { BAUD_REQ, 115200 };
                for (size_t i = 0; i < sizeof(escape_bauds) / sizeof(escape_bauds[0]) && !at_ready; i++)
                {
                    modem_uart_teardown();
                    modem_uart_setup(escape_bauds[i], (i == 0));
                    sleep_ms(1100);                               // guard time before +++
                    at_write("+++");                              // no \r – raw escape
                    sleep_ms(1100);                               // guard time after +++
                    if (at_probe_quick())
                    {
                        ESP_LOGI(TAG, "+++ escape succeeded at %d baud – preserving registration", escape_bauds[i]);
                        at_write("ATH\r");
                        at_expect_ok(1000);
                        at_ready = true;
                    }
                }
            }

            if (!at_ready)
            {
                ESP_LOGW(TAG, "Modem ON but unresponsive (CMUX or hung) – resetting");
                (void)modem_safe_reset_sequence();
            }
        }
    }
    else
    {
        // Power it on
        ESP_LOGI(TAG, "Modem status: OFF -> powering ON");
        if (modem_safe_reset_sequence() != ESP_OK)
        {
            ESP_LOGW(TAG, "Could not confirm modem ON status");
        }
    }

    esp_err_t err;
    if (!at_ready)
    {
        // Ensure AT command access (includes fallback reset if needed)
        err = ensure_at_access_with_recovery();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Unable to get AT access");
            modem_uart_teardown();
            return err;
        }
    }

    // Configure flow control and baudrate, store if changed
    err = configure_flow_and_baud();

    // Before tearing down UART, log modem version as requested
    modem_log_version();

    // Initialization complete: release UART driver as requested
    modem_uart_teardown();

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Modem ready at %d baud with HW flow control", BAUD_REQ);
    }
    return err;
}
