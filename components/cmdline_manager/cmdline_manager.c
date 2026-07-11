/**
 * @file cmdline_manager.c
 * @brief Lifecycle, the command registry (+ the one built-in: `help`),
 *        the dispatcher (one command at a time), sink routing, and the
 *        endpoint trio. The UART console lives in
 *        cmdline_manager_console.c; settings live in
 *        cmdline_manager_settings.c (standard §4.1).
 */
#include "cmdline_manager.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "log_manager.h"

#include "cmdline_manager_private.h"

static const char *TAG = "cmdline_manager";

static bool s_started;

/* the ONE dispatcher lock: one command at a time device-wide */
static SemaphoreHandle_t s_lock;
static StaticSemaphore_t s_lock_buf; /* internal: FreeRTOS object */

/* the active output sink (valid only under s_lock) */
static cmdline_output_fn_t s_sink;
static void *s_sink_arg;

/* the registry mirror: {command, help, hint} for the legacy-format
 * `help` (esp_console keeps its own table but exposes no iteration) */
typedef struct
{
    const char *command;
    const char *help;
    const char *hint;
} cm_cmd_info_t;

static cm_cmd_info_t s_cmds[CMDLINE_MANAGER_MAX_CMDS] EXT_RAM_BSS_ATTR;
static size_t s_cmd_count;

/* endpoint face: input queue -> dispatcher task -> subscriber queue */
static QueueHandle_t s_in_q;
static StaticQueue_t s_in_q_buf;   /* internal: FreeRTOS object */
static uint8_t s_in_q_store[8 * sizeof(cmdline_chunk_t)] EXT_RAM_BSS_ATTR;
static QueueHandle_t s_sub_q;      /* the bridge's queue            */
static uint32_t s_drops;
static cm_line_asm_t s_ep_asm;

/* async exec face: whole lines + their sink, run by the dispatcher
 * task (BLE — the NimBLE host task must never exec inline) */
typedef struct
{
    cmdline_output_fn_t out;
    void               *arg;
    char                line[CMDLINE_MANAGER_LINE_MAX];
} cm_async_line_t;

static QueueHandle_t s_line_q;
static StaticQueue_t s_line_q_buf; /* internal: FreeRTOS object */
static uint8_t s_line_q_store[4 * sizeof(cm_async_line_t)]
    EXT_RAM_BSS_ATTR;

/* the dispatcher blocks on both faces at once */
static QueueSetHandle_t s_q_set;

static TaskHandle_t s_task;
static StaticTask_t s_tcb;         /* internal: FreeRTOS object */
/* INTERNAL stack (§2 corollary): handlers call arbitrary component
   code — filesystem_info() walks LittleFS on internal FLASH, which a
   PSRAM-stack task may not do (cache-off assert; hit live via `fs`).
   5 KB: measured 2.7 KB peak incl. the LittleFS walk. */
static StackType_t s_stack[5120];

/* ---- output routing --------------------------------------------------------- */

void cmdline_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;

    va_start(args, fmt);

    int n = vsnprintf(buf, sizeof(buf), fmt, args);

    va_end(args);

    if (n <= 0)
    {
        return;
    }

    if (n > (int)sizeof(buf) - 1)
    {
        n = (int)sizeof(buf) - 1;
    }

    cmdline_write(buf, (size_t)n);
}

void cmdline_write(const char *data, size_t len)
{
    if (data == NULL || len == 0)
    {
        return;
    }

    if (s_sink != NULL)
    {
        s_sink(data, len, s_sink_arg);
    }
    else
    {
        fwrite(data, 1, len, stdout); /* UART console context */
    }
}

esp_err_t cmdline_manager_exec_line(const char *line,
                                    cmdline_output_fn_t out, void *arg)
{
    if (line == NULL || !s_started)
    {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_sink = out;
    s_sink_arg = arg;

    int ret = 0;
    esp_err_t err = esp_console_run(line, &ret);

    if (err == ESP_ERR_NOT_FOUND)
    {
        cmdline_printf("Unknown command: %s\n", line);
        cmdline_printf("Try: help\n");
    }
    else if (err == ESP_ERR_INVALID_ARG)
    {
        /* empty/whitespace line — nothing ran */
    }
    else if (err == ESP_OK && ret != 0)
    {
        cmdline_printf("command failed (%d)\n", ret);
    }

    if (out != NULL)
    {
        cmdline_printf(CM_PROMPT); /* transports get the legacy prompt;
                                      the UART console's comes from
                                      linenoise itself */
    }

    s_sink = NULL;
    s_sink_arg = NULL;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

/* ---- the registry + help ------------------------------------------------------ */

esp_err_t cmdline_manager_register(const esp_console_cmd_t *cmd)
{
    if (cmd == NULL || cmd->command == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_cmd_count >= CMDLINE_MANAGER_MAX_CMDS)
    {
        ESP_LOGE(TAG, "command table full; '%s' not registered",
                 cmd->command);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_console_cmd_register(cmd);

    if (err != ESP_OK)
    {
        return err;
    }

    s_cmds[s_cmd_count].command = cmd->command;
    s_cmds[s_cmd_count].help = cmd->help;
    s_cmds[s_cmd_count].hint = cmd->hint;
    s_cmd_count++;
    return ESP_OK;
}

/** Legacy help format: the list with indented hints, or `help <cmd>`. */
static int cmd_help(int argc, char **argv)
{
    if (argc == 2)
    {
        const cm_cmd_info_t *entry = NULL;

        for (size_t i = 0; i < s_cmd_count; i++)
        {
            if (strcmp(s_cmds[i].command, argv[1]) == 0)
            {
                entry = &s_cmds[i];
                break;
            }
        }

        if (entry == NULL)
        {
            cmdline_printf("Unknown command: %s\n", argv[1]);
            cmdline_printf("Try: help\n");
            return 0;
        }

        cmdline_printf("%s\n", entry->command);

        if (entry->help != NULL)
        {
            cmdline_printf("  %s\n", entry->help);
        }

        if (entry->hint != NULL)
        {
            cmdline_printf("  %s\n", entry->hint);
        }

        cmdline_printf("OK\n");
        return 0;
    }

    if (argc > 2)
    {
        cmdline_printf("Usage: help [cmd]\n");
        return 0;
    }

    cmdline_printf("Available commands:\n");

    for (size_t i = 0; i < s_cmd_count; i++)
    {
        if (s_cmds[i].help != NULL)
        {
            cmdline_printf("  %s - %s\n", s_cmds[i].command,
                           s_cmds[i].help);
        }
        else
        {
            cmdline_printf("  %s\n", s_cmds[i].command);
        }

        if (s_cmds[i].hint != NULL)
        {
            cmdline_printf("    %s\n", s_cmds[i].hint);
        }
    }

    cmdline_printf("\nTip: help <cmd> for details\n");
    cmdline_printf("OK\n");
    return 0;
}

/* ---- the endpoint face --------------------------------------------------------- */

static void ep_sink(const char *data, size_t len, void *arg)
{
    (void)arg;

    while (len > 0 && s_sub_q != NULL)
    {
        cmdline_chunk_t chunk;
        size_t n = (len > CMDLINE_MANAGER_CHUNK_SIZE)
                       ? CMDLINE_MANAGER_CHUNK_SIZE : len;

        chunk.len = (uint16_t)n;
        memcpy(chunk.data, data, n);

        if (xQueueSend(s_sub_q, &chunk, pdMS_TO_TICKS(50)) != pdTRUE)
        {
            s_drops++;
            return;
        }

        data += n;
        len -= n;
    }
}

static void dispatch_chunk(const cmdline_chunk_t *chunk)
{
    size_t off = 0;

    while (off < chunk->len)
    {
        const char *line = NULL;

        off += cm_line_feed(&s_ep_asm, chunk->data + off,
                            chunk->len - off, &line);

        if (line != NULL)
        {
            char line_copy[CMDLINE_MANAGER_LINE_MAX];

            snprintf(line_copy, sizeof(line_copy), "%s", line);
            cmdline_manager_exec_line(line_copy, ep_sink, NULL);
        }
    }
}

static void dispatcher_task(void *arg)
{
    (void)arg;
    cm_line_reset(&s_ep_asm);

    while (true)
    {
        QueueSetMemberHandle_t ready =
            xQueueSelectFromSet(s_q_set, portMAX_DELAY);

        if (ready == s_in_q)
        {
            cmdline_chunk_t chunk;

            if (xQueueReceive(s_in_q, &chunk, 0) == pdTRUE)
            {
                dispatch_chunk(&chunk);
            }
        }
        else if (ready == s_line_q)
        {
            /* PSRAM slot, not the stack: the item is 264 B and the
             * exec below is the deep path */
            static cm_async_line_t s_async EXT_RAM_BSS_ATTR;

            if (xQueueReceive(s_line_q, &s_async, 0) == pdTRUE)
            {
                cmdline_manager_exec_line(s_async.line, s_async.out,
                                          s_async.arg);
            }
        }
    }
}

esp_err_t cmdline_manager_exec_line_async(const char *line,
                                          cmdline_output_fn_t out,
                                          void *arg)
{
    if (line == NULL || s_line_q == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_started)
    {
        return ESP_ERR_INVALID_STATE;
    }

    cm_async_line_t item;

    item.out = out;
    item.arg = arg;
    snprintf(item.line, sizeof(item.line), "%s", line);

    if (xQueueSend(s_line_q, &item, 0) != pdTRUE)
    {
        /* interactive clients need SOME response; tiny write, safe
         * from the caller's context */
        if (out != NULL)
        {
            static const char busy[] = "busy\n" CM_PROMPT;

            out(busy, sizeof(busy) - 1, arg);
        }

        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t cmdline_manager_subscribe(QueueHandle_t q)
{
    if (q == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_sub_q != NULL)
    {
        return ESP_ERR_INVALID_STATE; /* one subscriber = one bridge */
    }

    s_sub_q = q;
    return ESP_OK;
}

esp_err_t cmdline_manager_unsubscribe(QueueHandle_t q)
{
    if (s_sub_q != q)
    {
        return ESP_ERR_NOT_FOUND;
    }

    s_sub_q = NULL;
    return ESP_OK;
}

esp_err_t cmdline_manager_send(const uint8_t *data, size_t len)
{
    if (data == NULL || s_in_q == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    while (len > 0)
    {
        cmdline_chunk_t chunk;
        size_t n = (len > CMDLINE_MANAGER_CHUNK_SIZE)
                       ? CMDLINE_MANAGER_CHUNK_SIZE : len;

        chunk.len = (uint16_t)n;
        memcpy(chunk.data, data, n);

        if (xQueueSend(s_in_q, &chunk, 0) != pdTRUE)
        {
            s_drops++;
            return ESP_ERR_NO_MEM; /* never block the transport */
        }

        data += n;
        len -= n;
    }

    return ESP_OK;
}

/* ---- lifecycle ----------------------------------------------------------------- */

esp_err_t cmdline_manager_init(void)
{
    static const log_descriptor_t LOG_DESC =
        { "cmdline_manager", ESP_LOG_INFO };
    static const esp_console_cmd_t HELP_CMD =
    {
        .command = "help",
        .help = "List available commands",
        .hint = "Usage: help [cmd]",
        .func = cmd_help,
    };

    log_manager_register(&LOG_DESC);

    if (s_lock == NULL)
    {
        s_lock = xSemaphoreCreateMutexStatic(&s_lock_buf);
        s_in_q = xQueueCreateStatic(8, sizeof(cmdline_chunk_t),
                                    s_in_q_store, &s_in_q_buf);
        s_line_q = xQueueCreateStatic(4, sizeof(cm_async_line_t),
                                      s_line_q_store, &s_line_q_buf);
        /* both queues are empty here (adding non-empty queues to a
         * set is illegal); ~100 B internal (no static set API) */
        s_q_set = xQueueCreateSet(8 + 4);
        xQueueAddToSet(s_in_q, s_q_set);
        xQueueAddToSet(s_line_q, s_q_set);
    }

    esp_console_config_t console_cfg =
    {
        .max_cmdline_length = CMDLINE_MANAGER_LINE_MAX,
        .max_cmdline_args = 12,
    };
    esp_err_t err = esp_console_init(&console_cfg);

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }

    err = cmdline_manager_register(&HELP_CMD);

    if (err != ESP_OK)
    {
        return err;
    }

    return cm_settings_register();
}

esp_err_t cmdline_manager_start(void)
{
    if (!cm_settings_is_configured())
    {
        ESP_LOGE(TAG, "unconfigured; not starting");
        return ESP_ERR_INVALID_STATE; /* standard §4.3 step 5 */
    }

    if (s_started)
    {
        return ESP_OK;
    }

    if (!cm_settings_enabled())
    {
        ESP_LOGI(TAG, "disabled in settings");
        return ESP_OK;
    }

    s_started = true; /* before the tasks — they exec immediately */
    s_task = xTaskCreateStatic(dispatcher_task, "cli_dispatch",
                               sizeof(s_stack) / sizeof(s_stack[0]),
                               NULL, 4, s_stack, &s_tcb);

    if (s_task == NULL)
    {
        s_started = false;
        return ESP_FAIL;
    }

    if (cm_settings_uart())
    {
        cm_console_start(); /* the linenoise UART console */
    }

    ESP_LOGI(TAG, "started (%u commands; uart console %s)",
             (unsigned)s_cmd_count, cm_settings_uart() ? "on" : "off");
    return ESP_OK;
}

esp_err_t cmdline_manager_stop(void)
{
    s_started = false; /* exec_line refuses; tasks idle out */
    return ESP_OK;
}
