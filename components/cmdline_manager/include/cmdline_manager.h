/**
 * @file cmdline_manager.h
 * @brief WiCAN command line owner (service component; rewrite of the
 *        legacy `cmdline`).
 *
 * ONE command REGISTRY + dispatcher (esp_console underneath). The
 * manager owns NO commands except `help` — every component registers
 * its own via cmdline_manager_register() (the same ownership inversion
 * as settings/log/http registration; shape follows IDF's
 * examples/system/console per-domain register_*() convention).
 *
 * Reachable over every transport WITHOUT transport code here:
 *
 *  - The component exposes the firmware's standard chunk-endpoint face
 *    (`subscribe`/`send` — the OBD/BLE/WS convention), so TCP, UDP and
 *    WebSocket consoles are CONFIGURED BRIDGES: `br_x = cli <-> ws_cli`
 *    (or a socket_manager server) in settings, zero code.
 *  - BLE: main glues ble_manager's CLI characteristics to
 *    `cmdline_manager_exec_line_async()` (the legacy BLE console) —
 *    async because the lines arrive on the NimBLE host task, which
 *    must never run a command inline (a long command wedges the BLE
 *    stack: GATT "Unlikely Error" on the link).
 *  - UART0: a linenoise console on the log UART (settings-gated) —
 *    line editing, history, tab completion, dumb-terminal fallback
 *    (the IDF advanced console example pattern; NOT the REPL component,
 *    whose internal loop would bypass the one-command-at-a-time lock).
 *
 * Commands print through `cmdline_printf()`/`cmdline_write()` (routed
 * to the invoking transport); responses end with the legacy `wican> `
 * prompt.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_console.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CMDLINE_MANAGER_CHUNK_SIZE 128
#define CMDLINE_MANAGER_LINE_MAX   256
#define CMDLINE_MANAGER_MAX_CMDS   32

typedef struct
{
    uint16_t len;
    uint8_t  data[CMDLINE_MANAGER_CHUNK_SIZE];
} cmdline_chunk_t;

/** Register settings + log descriptors + the registry (+ `help`). */
esp_err_t cmdline_manager_init(void);

/** Start the dispatcher task (+ the UART console when enabled). */
esp_err_t cmdline_manager_start(void);
esp_err_t cmdline_manager_stop(void);

/* ---- the command registry ---------------------------------------------------- */

/**
 * Register one command (esp_console semantics; `help`/`hint` strings
 * must be static — they feed the legacy-format `help` output). Called
 * by a component's `<comp>_register_cli()` — which the component itself
 * invokes on its settings boot apply when its `cli` setting is true
 * (ownership lives in the component; main wires nothing). Requires
 * cmdline_manager_init() first — main's init order guarantees it.
 * Handlers run in the dispatcher/console/BLE task and print via
 * cmdline_printf(). ESP_ERR_NO_MEM when the table
 * (CMDLINE_MANAGER_MAX_CMDS) is full.
 */
esp_err_t cmdline_manager_register(const esp_console_cmd_t *cmd);

/* ---- the endpoint trio (bridges wire transports to these) ------------------ */

/** Attach the ONE subscriber queue receiving RESPONSE chunks
 *  (cmdline_chunk_t items). ESP_ERR_INVALID_STATE if taken. */
esp_err_t cmdline_manager_subscribe(QueueHandle_t q);
esp_err_t cmdline_manager_unsubscribe(QueueHandle_t q);

/** Feed INPUT bytes (any chunking; lines end at \n or \r). */
esp_err_t cmdline_manager_send(const uint8_t *data, size_t len);

/* ---- direct line execution (BLE glue, the UART console, tests) -------------- */

/**
 * Execute one command line; the response goes to @p out (called from
 * the invoking task, possibly in several pieces; NULL = stdout).
 * Serialized with every other transport — one command at a time
 * device-wide.
 *
 * CALLER CONTRACT: the handler runs on YOUR task — use an
 * INTERNAL-RAM stack with a few KB of headroom (commands may touch
 * internal flash, which a PSRAM-stack task may not — §2 corollary).
 */
typedef void (*cmdline_output_fn_t)(const char *data, size_t len,
                                    void *arg);
esp_err_t cmdline_manager_exec_line(const char *line,
                                    cmdline_output_fn_t out, void *arg);

/**
 * Queue one command line for the DISPATCHER task; returns immediately.
 * The response streams to @p out from the dispatcher's context (an
 * internal-RAM stack sized for arbitrary handlers) — use this from
 * tasks that must never block or run handlers inline (the NimBLE host
 * task). @p out/@p arg must stay valid until the line completes.
 * On a full queue the caller gets "busy" + the prompt through @p out
 * (so interactive clients see feedback) and ESP_ERR_NO_MEM back.
 */
esp_err_t cmdline_manager_exec_line_async(const char *line,
                                          cmdline_output_fn_t out,
                                          void *arg);

/** Response print for COMMAND HANDLERS (routed to the active
 *  transport). Outside a command context it falls through to stdout. */
void cmdline_printf(const char *fmt, ...)
    __attribute__((format(printf, 1, 2)));

/** Raw response bytes for COMMAND HANDLERS — for payloads larger than
 *  cmdline_printf's formatting buffer (scan JSON, task lists). */
void cmdline_write(const char *data, size_t len);

#ifdef __cplusplus
}
#endif
