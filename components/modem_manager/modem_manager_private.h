/**
 * @file modem_manager_private.h
 * @brief Internals shared between modem_manager.c (GPIO power sequencing +
 *        UART driver helpers) and modem_manager_probe.c (AT access, baud
 *        probing and negotiation). Not part of the public API.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "modem_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Target baud rate for modem communication (3 Mbaud for high-speed operation) */
#define BAUD_REQ                 3000000

// Active configuration copied from user-provided cfg in modem_mgr_init
// (defined in modem_manager.c)
extern modem_mgr_config_t s_cfg;

/* modem_manager.c: timing, UART driver and power-sequencing helpers */
int64_t now_ms(void);
void sleep_ms(uint32_t ms);
esp_err_t modem_uart_setup(int baud, bool hwfc_on);
void modem_uart_teardown(void);
void modem_uart_set_baud(int baud);
esp_err_t modem_safe_reset_sequence(void);

/* modem_manager_probe.c: AT command access, probing and negotiation */
esp_err_t at_write(const char *s);
bool at_expect_ok(uint32_t timeout_ms);
bool at_probe_quick(void);
bool try_at_quick(int baud, bool hwfc);
esp_err_t ensure_at_access_with_recovery(void);
esp_err_t configure_flow_and_baud(void);
void modem_log_version(void);

#ifdef __cplusplus
}
#endif
