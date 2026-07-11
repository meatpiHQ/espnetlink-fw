/**
 * @file cmdline_manager_console.c
 * @brief The UART0 console — the IDF advanced console example pattern
 *        (examples/system/console/advanced), NOT the REPL component:
 *        running linenoise + esp_console_run in our own loop keeps every
 *        command inside cmdline_manager's one-at-a-time lock.
 *
 * UART driver + blocking VFS reads, then linenoise: line editing,
 * arrow-key history (RAM, 32 entries), tab completion and hints from
 * the esp_console registry, and a dumb-mode fallback when the terminal
 * doesn't answer the escape probe (pipes, capture tools).
 */
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_attr.h"
#include "esp_console.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "linenoise/linenoise.h"
#include "sdkconfig.h"

#include "cmdline_manager.h"
#include "cmdline_manager_private.h"

static const char *TAG = "cmdline_manager";

static TaskHandle_t s_console_task;
static StaticTask_t s_console_tcb;    /* internal: FreeRTOS object */
/* INTERNAL stack: runs command handlers (same §2-corollary contract as
   the dispatcher — flash-touching commands) + linenoise editing.
   6 KB: measured 1.2 KB peak; headroom for editing + the fs walk. */
static StackType_t s_console_stack[6144];

/** UART driver + VFS so stdin blocks properly (the raw console VFS is
 *  non-blocking, which linenoise can't work with). */
static esp_err_t console_peripheral_init(void)
{
    const uart_config_t uart_config =
    {
        .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* drain whatever printf buffered before the driver takes over */
    fflush(stdout);

    esp_err_t err = uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM,
                                        256, 0, 0, NULL, 0);

    if (err != ESP_OK)
    {
        return err;
    }

    err = uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config);

    if (err != ESP_OK)
    {
        return err;
    }

    uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM,
                                          ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM,
                                          ESP_LINE_ENDINGS_CRLF);
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    return ESP_OK;
}

static void console_library_init(void)
{
    linenoiseSetMultiLine(1);
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback(
        (linenoiseHintsCallback *)&esp_console_get_hint);
    linenoiseHistorySetMaxLen(32); /* RAM only; no /data writes from
                                      this task (§2 corollary) */
    linenoiseSetMaxLineLen(CMDLINE_MANAGER_LINE_MAX);
    linenoiseAllowEmpty(false);

    if (linenoiseProbe() != 0)
    {
        /* no escape-sequence answers: a pipe or a capture tool */
        linenoiseSetDumbMode(1);
    }
}

static void console_task(void *arg)
{
    (void)arg;

    if (console_peripheral_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "uart console unavailable");
        vTaskDelete(NULL);
        return;
    }

    console_library_init();
    printf("\nType 'help' for available commands\n");

    while (true)
    {
        char *line = linenoise(CM_PROMPT);

        if (line == NULL)
        {
            continue; /* EOF / ^C — just re-prompt */
        }

        if (line[0] != '\0')
        {
            linenoiseHistoryAdd(line);
            /* NULL sink = responses go to stdout; linenoise redraws the
               prompt on the next iteration */
            cmdline_manager_exec_line(line, NULL, NULL);
            fflush(stdout);
        }

        linenoiseFree(line);
    }
}

void cm_console_start(void)
{
    if (s_console_task != NULL)
    {
        return;
    }

    s_console_task = xTaskCreateStatic(console_task, "cli_console",
                                       sizeof(s_console_stack) /
                                           sizeof(s_console_stack[0]),
                                       NULL, 3, s_console_stack,
                                       &s_console_tcb);
}
