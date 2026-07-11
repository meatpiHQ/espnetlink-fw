/**
 * @file modem_manager_parse.h
 * @brief Pure AT-response parsers shared by modem_manager_probe.c and the
 *        component's host tests (standard §7). No IDF dependencies, so the
 *        unit compiles unchanged on the linux target.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Parse an AT+IFC? response payload (flow control settings)
 * @param s Payload after the "+IFC: " prefix (e.g., "2,2")
 * @param rx Parsed RX flow control value
 * @param tx Parsed TX flow control value
 * @return true if parsed successfully, false otherwise
 * @note Flow control values: 0=none, 1=software, 2=hardware (RTS/CTS)
 */
bool mm_parse_ifc(const char *s, int *rx, int *tx);

/**
 * @brief Parse an AT+IPR? response payload (baud rate setting)
 * @param s Payload after the "+IPR: " prefix (e.g., "115200" or "3000000")
 * @param baud Parsed baud rate
 * @return true if parsed successfully, false otherwise
 */
bool mm_parse_ipr(const char *s, int *baud);

#ifdef __cplusplus
}
#endif
