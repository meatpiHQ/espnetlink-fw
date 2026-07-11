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
 * @file modem_manager_probe.c
 * @brief AT-command access, baud-rate probing and negotiation for the BG95 modem.
 *
 * Moved verbatim from modem_manager.c (§1 split): the AT command interface
 * helpers, the multi-baud AT access recovery ladder (3 Mbaud / 115200 with and
 * without hardware flow control) and the AT+IFC / AT+IPR negotiation layer.
 * GPIO power sequencing and the UART driver helpers stay in modem_manager.c;
 * shared pieces go through modem_manager_private.h.
 */

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"

#include "modem_manager.h"
#include "modem_manager_parse.h"
#include "modem_manager_private.h"

static const char *TAG = "modem_manager"; /* §10: TAG == component dir */

/** Timeout for reading a single line from AT command response */
#define AT_READ_LINE_TIMEOUT_MS  500

/** Delay between AT command retry attempts */
#define AT_RETRY_DELAY_MS        200

/** Maximum number of AT command retry attempts */
#define AT_MAX_TRIES             5

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
esp_err_t at_write(const char *s)
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
bool at_expect_ok(uint32_t timeout_ms)
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
 * @brief Fast AT probe with minimal retries for initial responsiveness check
 * @return true if modem responds with OK within ~600 ms, false otherwise
 */
bool at_probe_quick(void)
{
    for (int i = 0; i < 2; ++i)
    {
        uart_flush_input(s_cfg.uart_port);
        ESP_ERROR_CHECK_WITHOUT_ABORT(at_write("AT\r"));
        if (at_expect_ok(300))
        {
            return true;
        }
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
 * @brief Quick attempt: setup UART and do a fast probe (~600 ms instead of ~13 s)
 */
bool try_at_quick(int baud, bool hwfc)
{
    modem_uart_teardown();
    modem_uart_setup(baud, hwfc);
    if (at_probe_quick())
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

    // Best effort flush to reduce leftover responses interfering with parsing
    uart_flush_input(s_cfg.uart_port);

    if (at_write(cmdline) != ESP_OK)
    {
        return false;
    }

    // Read multiple lines until we find the prefixed response or hit a deadline.
    // (Some firmwares echo, insert blank lines, or print extra URCs.)
    const int64_t deadline = now_ms() + 2000;
    bool found = false;
    while (now_ms() < deadline)
    {
        int n = at_read_line(line, sizeof(line), 500);
        if (n <= 0)
        {
            continue;
        }
        if (strcmp(line, "OK") == 0)
        {
            break;
        }
        if (strcmp(line, "ERROR") == 0)
        {
            return false;
        }
        // Skip echo of the command
        if (strcmp(line, cmd) == 0)
        {
            continue;
        }
        if (strncmp(line, prefix, strlen(prefix)) == 0)
        {
            found = true;
            break;
        }
    }

    if (!found)
    {
        return false;
    }

    // Copy tail after prefix and optional spaces/colon
    const char *p = line + strlen(prefix);
    while (*p == ' ' || *p == ':' )
    {
        ++p;
    }
    strlcpy(out, p, out_len);


    // Consume remaining lines until OK/ERROR
    return at_expect_ok(1000);
}

/* parse_ifc / parse_ipr moved to modem_manager_parse.c (host-tested, §7). */

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
 * @brief Execute an AT command and wait for OK/ERROR while logging all lines
 *
 * Useful for commands that may take longer (e.g., AT&W).
 *
 * @param cmd AT command with CR terminator
 * @param total_timeout_ms Maximum time to wait in milliseconds
 * @return true if OK received, false otherwise
 */
static bool at_exec_wait_ok_log_lines(const char *cmd, uint32_t total_timeout_ms)
{
    char line[256];

    // Best effort flush to reduce leftover echoes
    uart_flush_input(s_cfg.uart_port);

    if (at_write(cmd) != ESP_OK) {
        return false;
    }

    const int64_t deadline = now_ms() + total_timeout_ms;
    // Prepare a printable command name (strip trailing CR)
    char cmd_print[64];
    size_t cmd_len = strlen(cmd);
    size_t plen = cmd_len;
    if (plen > 0 && cmd[plen - 1] == '\r') {
        plen--;
    }
    if (plen >= sizeof(cmd_print)) {
        plen = sizeof(cmd_print) - 1;
    }
    memcpy(cmd_print, cmd, plen);
    cmd_print[plen] = '\0';

    while (now_ms() < deadline) {
        int n = at_read_line(line, sizeof(line), 500);
        if (n < 0) {
            continue;
        }

        // Skip echo of the command
        if (cmd_print[0] != '\0' && strcmp(line, cmd_print) == 0) {
            continue;
        }

        if (strcmp(line, "OK") == 0) {
            ESP_LOGI(TAG, "%s -> OK", cmd_print);
            return true;
        }
        if (strcmp(line, "ERROR") == 0) {
            ESP_LOGW(TAG, "%s -> ERROR", cmd_print);
            return false;
        }

        ESP_LOGI(TAG, "%s", line);
    }

    return false;
}

/**
 * @brief Query and log modem firmware version information
 * @note Tries AT+GMR (firmware revision) first, falls back to ATI if that fails
 *       Logs version information directly to console via ESP_LOGI
 */
void modem_log_version(void)
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
esp_err_t ensure_at_access_with_recovery(void)
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
esp_err_t configure_flow_and_baud(void)
{
    bool changed = false;

    bool ifc_known = false;
    bool ipr_known = false;

    // Query current IFC
    char resp[64];
    int rx = -1, tx = -1;
    if (at_query_prefix("AT+IFC?", "+IFC", resp, sizeof(resp)))
    {
        if (mm_parse_ifc(resp, &rx, &tx))
        {
            ESP_LOGI(TAG, "Current IFC: %d,%d", rx, tx);
            ifc_known = true;
        }
    }
    else
    {
        ESP_LOGW(TAG, "AT+IFC? query failed; will not change flow control");
    }

    if (ifc_known && !(rx == 2 && tx == 2))
    {
        ESP_LOGI(TAG, "Enabling HW flow control (AT+IFC=2,2)");

        // Important: Many modems apply IFC immediately, and if the host UART isn't
        // already configured for RTS/CTS, the modem may stop transmitting (including the OK).
        // So enable UART HW flow control BEFORE sending AT+IFC.
        ESP_RETURN_ON_ERROR(uart_set_hw_flow_ctrl(s_cfg.uart_port,
                                                  UART_HW_FLOWCTRL_CTS_RTS, 64),
                            TAG, "uart_set_hw_flow_ctrl");
        sleep_ms(20);

        if (at_write("AT+IFC=2,2\r") != ESP_OK || !at_expect_ok(1000))
        {
            ESP_LOGE(TAG, "Failed to set IFC");
            // Revert UART flow control so caller can still communicate
            ESP_ERROR_CHECK_WITHOUT_ABORT(
                uart_set_hw_flow_ctrl(s_cfg.uart_port,
                                      UART_HW_FLOWCTRL_DISABLE, 0));
            return ESP_FAIL;
        }
        changed = true;
    }

    // Query current IPR
    int cur_baud = -1;
    if (at_query_prefix("AT+IPR?", "+IPR", resp, sizeof(resp)))
    {
        if (mm_parse_ipr(resp, &cur_baud))
        {
            ESP_LOGI(TAG, "Current IPR: %d", cur_baud);
            ipr_known = true;
        }
    }
    else
    {
        ESP_LOGW(TAG, "AT+IPR? query failed; will not change baud");
    }

    if (ipr_known && cur_baud != BAUD_REQ)
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
        // AT&W may take several seconds on some firmware. Log all response lines.
        if (!at_exec_wait_ok_log_lines("AT&W\r", 10000))
        {
            ESP_LOGW(TAG, "AT&W failed or timed out; trying AT&W0");
            if (!at_exec_wait_ok_log_lines("AT&W0\r", 10000))
            {
                ESP_LOGW(TAG, "AT&W0 also failed or timed out");
            }
        }
    }

    return ESP_OK;
}
