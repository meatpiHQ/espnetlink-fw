// Component-internal API between the CDC console core and its command
// modules. Not part of the public header (include/usb_cli_console.h).
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Size of one CDC OUT endpoint read. The raw-RX pause threshold in consumers
// must leave at least this much space before allowing a re-arm.
#ifndef USB_CLI_RX_BUF_SIZE
#define USB_CLI_RX_BUF_SIZE 256
#endif

/**
 * Raw RX diversion: while a sink is installed, incoming CDC bytes bypass the
 * line editor / echo / GPS-stream handling entirely and go to @p fn.
 *
 * @p fn runs in USB driver context (possibly ISR) — it must not block.
 * Return value: true  = re-arm the OUT endpoint (keep receiving),
 *               false = leave the endpoint un-armed (host sees NAK-based
 *                       flow control) until usb_cli_console_raw_rx_resume().
 * Passing NULL uninstalls the sink and resumes normal console input.
 */
typedef bool (*usb_cli_raw_rx_fn_t)(const uint8_t *data, size_t len);
void usb_cli_console_set_raw_rx(usb_cli_raw_rx_fn_t fn);

/** Re-arm the OUT endpoint after the raw sink drained (safe to call often;
 *  also retries a failed re-arm). Call from task context. */
void usb_cli_console_raw_rx_resume(void);

/** Register the `ota` console command (usb_cli_ota.c). */
void usb_cli_ota_register(void);

#ifdef __cplusplus
}
#endif
