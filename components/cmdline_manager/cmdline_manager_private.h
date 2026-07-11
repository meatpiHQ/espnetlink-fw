/**
 * @file cmdline_manager_private.h
 * @brief Internal contracts: the PURE line assembler (host-testable)
 *        and the command-table registration hook.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/* ---- pure line assembler (cmdline_manager_line.c) --------------------------
 * Bytes in (any chunking) -> complete lines out. \r and \n both
 * terminate; empty lines are skipped; CRLF never yields a bogus empty
 * line; an overlong line is discarded (flagged) up to its terminator. */

typedef struct
{
    char   buf[256];
    size_t len;
    bool   overflow;
} cm_line_asm_t;

void cm_line_reset(cm_line_asm_t *a);

/**
 * Consume input until a complete line or the end of input.
 * @return bytes consumed from @p data. *out_line is set to the
 * NUL-terminated line when one completed (else NULL). Call repeatedly
 * while it consumes less than @p len.
 */
size_t cm_line_feed(cm_line_asm_t *a, const uint8_t *data, size_t len,
                    const char **out_line);

/* ---- shared internals -------------------------------------------------------- */

#define CM_PROMPT "wican> " /* the legacy prompt */

/** Start the UART0 linenoise console task (cmdline_manager_console.c).
 *  Called from cmdline_manager_start() when the `uart` setting is on. */
void cm_console_start(void);

/* ---- settings (cmdline_manager_settings.c) ----------------------------------- */

/** Register the "cmdline_manager" descriptor with settings_manager. */
esp_err_t cm_settings_register(void);

bool cm_settings_enabled(void);       /* the dispatcher/console run at all  */
bool cm_settings_uart(void);          /* the UART0 linenoise console        */
bool cm_settings_is_configured(void); /* boot apply ran (standard §4.3)     */
