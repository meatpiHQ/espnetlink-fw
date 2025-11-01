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

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "modem_manager.h"

static const char *TAG = "modem_mgr";

// Active configuration is copied from user-provided cfg in modem_mgr_init
static modem_mgr_config_t s_cfg;

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

/** Timeout for reading a single line from AT command response */
#define AT_READ_LINE_TIMEOUT_MS  500

/** Delay between AT command retry attempts */
#define AT_RETRY_DELAY_MS        200

/** Maximum number of AT command retry attempts */
#define AT_MAX_TRIES             5

/** Target baud rate for modem communication (3 Mbaud for high-speed operation) */
#define BAUD_REQ                 3000000

/*
 * =============================================================================
 * UTILITY FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Get current time in milliseconds
 * @return Current time in milliseconds since system start
 */
static inline int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

/**
 * @brief Sleep for specified milliseconds
 * @param ms Number of milliseconds to sleep
 */
static void sleep_ms(uint32_t ms)
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
 * - PWR_KEY: Output pin for power control (active low, idle high)
 * - STATUS: Input pin to monitor modem power state (low when ON)
 * - DTR: Output pin kept high to prevent modem sleep mode
 */
static void modem_gpio_init(void)
{
    // PWR_KEY (active low)
    gpio_reset_pin(s_cfg.pwr_key_pin);
    gpio_set_direction(s_cfg.pwr_key_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(s_cfg.pwr_key_pin, 1); // idle high

    // STATUS (input) - low when modem is ON per hw_config.h comment
    gpio_reset_pin(s_cfg.status_pin);
    gpio_set_direction(s_cfg.status_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(s_cfg.status_pin, GPIO_FLOATING);

    // DTR (keep high to avoid sleep)
    // gpio_reset_pin(LTE_DTR_PIN);
    // gpio_set_direction(LTE_DTR_PIN, GPIO_MODE_OUTPUT);
    // gpio_set_level(LTE_DTR_PIN, 1);
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
 * @note PWR_KEY is active low. Different pulse durations trigger different actions:
 *       - 500-800ms: Toggle power state (ON->OFF or OFF->ON)
 *       - 1500ms+: Graceful power-off with proper shutdown sequence
 */
static void modem_pwrkey_pulse(uint32_t ms)
{
    // Active low pulse
    gpio_set_level(s_cfg.pwr_key_pin, 0);
    sleep_ms(ms);
    gpio_set_level(s_cfg.pwr_key_pin, 1);
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
static esp_err_t modem_safe_reset_sequence(void)
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
static esp_err_t modem_uart_setup(int baud, bool hwfc_on)
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

    ESP_ERROR_CHECK(uart_driver_install(s_cfg.uart_port, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(s_cfg.uart_port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(s_cfg.uart_port, tx, rx, rts, cts));

    if (hwfc_on)
    {
        ESP_ERROR_CHECK(uart_set_hw_flow_ctrl(s_cfg.uart_port, UART_HW_FLOWCTRL_CTS_RTS, 64));
    }
    else
    {
        ESP_ERROR_CHECK(uart_set_hw_flow_ctrl(s_cfg.uart_port, UART_HW_FLOWCTRL_DISABLE, 0));
    }

    return ESP_OK;
}

/**
 * @brief Uninstall UART driver to free resources
 * @note Only uninstalls if driver is currently installed to avoid error messages
 */
static void modem_uart_teardown(void)
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
static void modem_uart_set_baud(int baud)
{
    ESP_LOGI(TAG, "Setting UART baud rate to %d", baud);
    uart_set_baudrate(s_cfg.uart_port, baud);
}

/*
 * =============================================================================
 * AT COMMAND INTERFACE FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Write AT command string to modem via UART
 * @param s AT command string to send (should include \r terminator)
 * @return ESP_OK if all bytes written successfully, ESP_FAIL otherwise
 * @note Waits for transmission to complete before returning
 */
static esp_err_t at_write(const char *s)
{
    size_t len = strlen(s);
    int w = uart_write_bytes(s_cfg.uart_port, s, len);
    uart_wait_tx_done(s_cfg.uart_port, pdMS_TO_TICKS(50));
    return (w == (int)len) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Read one line from modem AT response (CR/LF terminated)
 * @param buf Buffer to store the line
 * @param max_len Maximum buffer size
 * @param timeout_ms Timeout in milliseconds
 * @return Line length (>=0) on success, -1 on timeout or no data
 * @note Automatically strips leading and trailing CR/LF characters
 */
static int at_read_line(char *buf, size_t max_len, uint32_t timeout_ms)
{
    size_t idx = 0;
    const int64_t deadline = now_ms() + timeout_ms;

    while (now_ms() < deadline && idx + 1 < max_len)
    {
        uint8_t ch;
    int r = uart_read_bytes(s_cfg.uart_port, &ch, 1, pdMS_TO_TICKS(20));
        if (r == 1)
        {
            // Skip leading CR/LF
            if ((idx == 0) && (ch == '\r' || ch == '\n'))
            {
                continue;
            }
            buf[idx++] = (char)ch;
            if (ch == '\n')
            {
                break;
            }
        }
    }

    if (idx == 0)
    {
        return -1; // timeout/no data
    }

    buf[idx] = '\0';
    // Trim trailing CR/LF
    while (idx > 0 && (buf[idx - 1] == '\r' || buf[idx - 1] == '\n'))
    {
        buf[--idx] = '\0';
    }
    return (int)idx;
}

/**
 * @brief Wait for "OK" or "ERROR" response from modem
 * @param timeout_ms Timeout for reading each line
 * @return true if "OK" received, false if "ERROR" or timeout
 * @note Consumes intermediate response lines until OK/ERROR found
 *       Tries up to AT_MAX_TRIES times before giving up
 */
static bool at_expect_ok(uint32_t timeout_ms)
{
    char line[256];
    int tries = 0;
    while (tries++ < AT_MAX_TRIES)
    {
        int n = at_read_line(line, sizeof(line), timeout_ms);
        if (n < 0)
        {
            continue;
        }
        if (strcmp(line, "OK") == 0)
        {
            return true;
        }
        if (strcmp(line, "ERROR") == 0)
        {
            return false;
        }
        // else keep consuming until OK/ERROR
    }
    return false;
}

/**
 * @brief Test AT command interface by sending "AT" and expecting "OK"
 * @return true if modem responds with OK, false otherwise
 * @note Retries AT_MAX_TRIES times with AT_RETRY_DELAY_MS between attempts
 *       Used to verify modem is in command mode and responsive
 */
static bool at_probe(void)
{
    for (int i = 0; i < AT_MAX_TRIES; ++i)
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(at_write("AT\r"));
        if (at_expect_ok(AT_READ_LINE_TIMEOUT_MS))
        {
            return true;
        }
        sleep_ms(AT_RETRY_DELAY_MS);
    }
    return false;
}

/**
 * @brief Attempt to establish AT command access at specific baud rate and flow control
 * @param baud Baud rate to try (e.g., 115200, 3000000)
 * @param hwfc true to enable hardware flow control, false to disable
 * @return true if AT probe successful, false otherwise
 * @note Tears down and reinstalls UART driver with new settings each time
 */
static bool try_at(int baud, bool hwfc)
{
    modem_uart_teardown();
    modem_uart_setup(baud, hwfc);
    if (at_probe())
    {
        ESP_LOGI(TAG, "AT access at %d baud (hwfc=%s) established", baud, hwfc ? "on" : "off");
        return true;
    }
    return false;
}

/**
 * @brief Query modem with AT command and parse response with expected prefix
 * @param cmd AT command without CR terminator (e.g., "AT+IFC?")
 * @param prefix Expected response prefix (e.g., "+IFC")
 * @param out Buffer to store parsed response (content after prefix)
 * @param out_len Size of output buffer
 * @return true if command successful and response matches prefix, false otherwise
 * @note Handles echo by trying to read a second line if first doesn't match prefix
 */
static bool at_query_prefix(const char *cmd, const char *prefix, char *out, size_t out_len)
{
    char line[256];
    char cmdline[64];
    snprintf(cmdline, sizeof(cmdline), "%s\r", cmd);
    if (at_write(cmdline) != ESP_OK)
    {
        return false;
    }

    // Expect response line (e.g., "+IFC: 2,2"), then OK
    int n = at_read_line(line, sizeof(line), 1000);
    if (n <= 0)
    {
        return false;
    }
    if (strncmp(line, prefix, strlen(prefix)) != 0)
    {
        // Might be echo or something else; read next line once more
        n = at_read_line(line, sizeof(line), 500);
        if (n <= 0 || strncmp(line, prefix, strlen(prefix)) != 0)
        {
            return false;
        }
    }

    // Copy tail after prefix and optional spaces/colon
    const char *p = line + strlen(prefix);
    while (*p == ' ' || *p == ':' )
    {
        ++p;
    }
    strlcpy(out, p, out_len);

    return at_expect_ok(1000);
}

/**
 * @brief Parse AT+IFC? response (flow control settings)
 * @param s Response string (e.g., "2,2")
 * @param rx Pointer to store RX flow control value
 * @param tx Pointer to store TX flow control value
 * @return true if parsed successfully, false otherwise
 * @note Flow control values: 0=none, 1=software, 2=hardware (RTS/CTS)
 */
static bool parse_ifc(const char *s, int *rx, int *tx)
{
    // formats: "2,2"
    int r = -1, t = -1;
    if (sscanf(s, "%d,%d", &r, &t) == 2)
    {
        *rx = r; *tx = t;
        return true;
    }
    return false;
}

/**
 * @brief Parse AT+IPR? response (baud rate setting)
 * @param s Response string (e.g., "115200" or "3000000")
 * @param baud Pointer to store parsed baud rate
 * @return true if parsed successfully, false otherwise
 */
static bool parse_ipr(const char *s, int *baud)
{
    int b = -1;
    if (sscanf(s, "%d", &b) == 1)
    {
        *baud = b;
        return true;
    }
    return false;
}

/**
 * @brief Execute AT command and log all response lines
 * @param cmd AT command with CR terminator (e.g., "AT+GMR\r")
 * @return true if command succeeded and logged at least one line, false otherwise
 * @note Skips command echo and stops at OK/ERROR
 *       Used for multi-line responses like firmware version
 */
static bool modem_log_cmd_lines(const char *cmd)
{
    char line[256];
    bool got_any = false;

    // Best effort flush to reduce leftover echoes
    uart_flush_input(s_cfg.uart_port);

    if (at_write(cmd) != ESP_OK)
    {
        return false;
    }

    int lines = 0;
    int64_t deadline = now_ms() + 2000; // up to 2s
    while (now_ms() < deadline && lines < 10)
    {
        int n = at_read_line(line, sizeof(line), 500);
        if (n < 0)
        {
            continue;
        }
        // Skip exact echo of the command (without CR)
        if ((strcmp(cmd, "AT+GMR\r") == 0 && strcmp(line, "AT+GMR") == 0) ||
            (strcmp(cmd, "ATI\r") == 0 && strcmp(line, "ATI") == 0))
        {
            continue;
        }
        if (strcmp(line, "OK") == 0)
        {
            return got_any; // success if we logged something
        }
        if (strcmp(line, "ERROR") == 0)
        {
            return false;
        }
        // Log just the modem-reported content
        ESP_LOGI(TAG, "%s", line);
        got_any = true;
        lines++;
    }
    return got_any;
}

/**
 * @brief Query and log modem firmware version information
 * @note Tries AT+GMR (firmware revision) first, falls back to ATI if that fails
 *       Logs version information directly to console via ESP_LOGI
 */
static void modem_log_version(void)
{
    ESP_LOGI(TAG, "Querying modem version (AT+GMR/ATI)");
    // Try AT+GMR first (firmware revision), fallback to ATI
    if (!modem_log_cmd_lines("AT+GMR\r"))
    {
        (void)modem_log_cmd_lines("ATI\r");
    }
}

/*
 * =============================================================================
 * HIGH-LEVEL CONFIGURATION FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Establish AT command access with automatic recovery and fallback
 *
 * Recovery strategy (in order):
 * 1. Try 3 Mbaud with hardware flow control (expected operating state)
 * 2. Try 3 Mbaud without hardware flow control
 * 3. Try 115200 baud (factory default) with/without flow control
 * 4. Perform safe modem reset (power cycle)
 * 5. Retry 115200 and 3 Mbaud after reset
 *
 * This multi-stage approach handles various scenarios:
 * - Normal operation (modem already configured)
 * - Factory reset modem (115200 default)
 * - Modem in unknown state (requires reset)
 * - Flow control mismatch
 *
 * @return ESP_OK if AT access established, ESP_FAIL if all attempts exhausted
 * @note UART driver will be installed at whatever baud rate succeeds
 */
static esp_err_t ensure_at_access_with_recovery(void)
{
    // Start with the normal operating state: 3,000,000 baud with HW flow control ON
    if (try_at(BAUD_REQ, true))
    {
        return ESP_OK;
    }

    // In case flow control is off at 3M
    if (try_at(BAUD_REQ, false))
    {
        return ESP_OK;
    }

    // Fall back to factory defaults typically: 115200 (try without and with HW flow)
    if (try_at(115200, false) || try_at(115200, true))
    {
        return ESP_OK;
    }

    ESP_LOGW(TAG, "AT not responding; attempting safe reset");
    if (modem_safe_reset_sequence() != ESP_OK)
    {
        ESP_LOGW(TAG, "Safe reset didn't confirm status; continuing");
    }

    // After power cycle, defaults may be 115200. Try 115200 first, then 3M again.
    if (try_at(115200, false) || try_at(115200, true))
    {
        return ESP_OK;
    }

    if (try_at(BAUD_REQ, true) || try_at(BAUD_REQ, false))
    {
        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * @brief Configure modem for optimal baud rate and hardware flow control
 *
 * Configuration steps:
 * 1. Query current flow control settings (AT+IFC?)
 * 2. If not RTS/CTS (2,2), configure hardware flow control (AT+IFC=2,2)
 * 3. Query current baud rate (AT+IPR?)
 * 4. If not 3 Mbaud, configure target baud rate (AT+IPR=3000000)
 * 5. Dynamically switch ESP32 UART to new baud rate
 * 6. Verify communication at new baud rate
 * 7. Save configuration to modem NV memory (AT&W)
 *
 * @return ESP_OK if configuration successful, ESP_FAIL on error
 * @note Modem will immediately switch baud rate after AT+IPR command
 *       ESP32 UART is adjusted on-the-fly to match
 * @note Configuration is persisted to modem's non-volatile memory
 */
static esp_err_t configure_flow_and_baud(void)
{
    bool changed = false;

    // Query current IFC
    char resp[64];
    int rx = -1, tx = -1;
    if (at_query_prefix("AT+IFC?", "+IFC", resp, sizeof(resp)))
    {
        if (parse_ifc(resp, &rx, &tx))
        {
            ESP_LOGI(TAG, "Current IFC: %d,%d", rx, tx);
        }
    }

    if (!(rx == 2 && tx == 2))
    {
        ESP_LOGI(TAG, "Enabling HW flow control (AT+IFC=2,2)");
        if (at_write("AT+IFC=2,2\r") != ESP_OK || !at_expect_ok(1000))
        {
            ESP_LOGE(TAG, "Failed to set IFC");
            return ESP_FAIL;
        }
        changed = true;
    }

    // Query current IPR
    int cur_baud = -1;
    if (at_query_prefix("AT+IPR?", "+IPR", resp, sizeof(resp)))
    {
        if (parse_ipr(resp, &cur_baud))
        {
            ESP_LOGI(TAG, "Current IPR: %d", cur_baud);
        }
    }

    if (cur_baud != BAUD_REQ)
    {
        ESP_LOGI(TAG, "Setting baud to %d", BAUD_REQ);
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "AT+IPR=%d\r", BAUD_REQ);
        if (at_write(cmd) != ESP_OK)
        {
            ESP_LOGE(TAG, "Write AT+IPR failed");
            return ESP_FAIL;
        }
        // After this command, modem switches speed immediately
        sleep_ms(50);
        modem_uart_set_baud(BAUD_REQ);
        // Re-sync
        if (!at_probe())
        {
            ESP_LOGE(TAG, "Failed to communicate at new baud");
            return ESP_FAIL;
        }
        changed = true;
    }

    if (changed)
    {
        ESP_LOGI(TAG, "Storing configuration (AT&W)");
        if (at_write("AT&W\r") != ESP_OK || !at_expect_ok(2000))
        {
            ESP_LOGW(TAG, "AT&W failed or not supported");
        }
    }

    return ESP_OK;
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

    // If modem is ON and potentially in data/cmux, perform a safe reset to command mode
    if (modem_status_is_on())
    {
        ESP_LOGI(TAG, "Modem status: ON -> performing safe reset to ensure command mode");
        (void)modem_safe_reset_sequence();
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

    // Ensure AT command access
    esp_err_t err = ensure_at_access_with_recovery();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Unable to get AT access");
        // Make sure to release UART before returning
        modem_uart_teardown();
        return err;
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
