/**
 * @file log_manager.c
 * @brief The log pipeline: vprintf hook -> PSRAM message queue -> log task ->
 *        sinks. Producer path is wait-free (drop-oldest, §9.5); sink writes
 *        run in the log task only. Console + PSRAM crash ring are built in;
 *        everything else (TCP/UDP/WebSocket/file) registers via
 *        log_manager_add_sink().
 */
#include "log_manager.h"

#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_cache.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "log_manager_private.h"

static const char *TAG = "log_manager";

#define LM_LINE_MAX    CONFIG_LOG_MANAGER_LINE_MAX
#define LM_QUEUE_DEPTH CONFIG_LOG_MANAGER_QUEUE_DEPTH
#define LM_RING_SIZE   CONFIG_LOG_MANAGER_RING_SIZE
#define LM_MAX_SINKS   8
#define LM_MAX_LEVELS  32

typedef struct
{
    uint16_t len;
    char     text[LM_LINE_MAX];
} lm_msg_t;

typedef struct
{
    const log_sink_t *sink;
    bool              enabled;
} lm_sink_slot_t;

typedef struct
{
    char            name[32];
    esp_log_level_t level;
} lm_level_slot_t;

/* message queue + registries: PSRAM .bss (§2) */
static lm_msg_t s_queue[LM_QUEUE_DEPTH] EXT_RAM_BSS_ATTR;
static lm_sink_slot_t s_sinks[LM_MAX_SINKS] EXT_RAM_BSS_ATTR;
static lm_level_slot_t s_levels[LM_MAX_LEVELS] EXT_RAM_BSS_ATTR;
static size_t s_sink_count;
static size_t s_level_count;

/* crash ring: PSRAM .noinit — survives warm resets (restart_tracker pattern).
 * ONE object so the buffer can never link at PSRAM addr 0 on its own: the
 * header's mspi_tuning_guard leads and absorbs the boot-time tuning writes. */
static EXT_RAM_NOINIT_ATTR struct
{
    lm_ring_hdr_t hdr;
    uint8_t       buf[LM_RING_SIZE];
} s_ring;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED; /* internal: spinlock */
static uint32_t s_q_head; /* next write slot   */
static uint32_t s_q_count;
static uint32_t s_dropped;
static uint32_t s_dropped_reported;

static vprintf_like_t s_prev_vprintf;
static TaskHandle_t s_task;
static StaticTask_t s_task_tcb;            /* internal: FreeRTOS object */
static StackType_t s_task_stack[4096] EXT_RAM_BSS_ATTR; /* no flash writes here */
static volatile bool s_running;
static bool s_inited;
static bool s_console_enabled = true;
static bool s_ring_enabled = true;

/* ---- built-in sinks ---------------------------------------------------------- */

/* ESPNetLink port: global stdout is redirected to the USB CDC console
 * (usb_cli_console), so writing stdout would interleave log lines into CLI
 * responses (§6b byte-parity). The console sink therefore writes the REAL
 * console device (UART0) directly; stdout is only the fallback. */
static FILE *s_console_dev;

static esp_err_t console_write(const char *line, size_t len)
{
    if (s_console_dev == NULL)
    {
        s_console_dev = fopen("/dev/console", "w");
        if (s_console_dev != NULL)
        {
            setvbuf(s_console_dev, NULL, _IONBF, 0);
        }
    }

    fwrite(line, 1, len,
           (s_console_dev != NULL) ? s_console_dev : stdout);
    return ESP_OK;
}

static void ring_commit(void)
{
    esp_cache_msync(&s_ring.hdr, sizeof(s_ring.hdr),
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                    ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    esp_cache_msync(s_ring.buf, sizeof(s_ring.buf),
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                    ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}

static esp_err_t ring_write(const char *line, size_t len)
{
    portENTER_CRITICAL(&s_lock);
    lm_ring_append(&s_ring.hdr, s_ring.buf, line, len);
    portEXIT_CRITICAL(&s_lock);
    ring_commit();
    return ESP_OK;
}

static const log_sink_t S_CONSOLE_SINK = { "console", console_write };
static const log_sink_t S_RING_SINK = { "ring", ring_write };

/* ---- routing ------------------------------------------------------------------ */

static void route_to_sinks(const char *line, size_t len)
{
    for (size_t i = 0; i < s_sink_count; i++)
    {
        if (s_sinks[i].enabled)
        {
            s_sinks[i].sink->write(line, len);
        }
    }
}

/** Producer side: wait-free enqueue, drop-oldest under pressure (§9.5). */
static void enqueue(const char *line, size_t len)
{
    if (len >= LM_LINE_MAX)
    {
        len = LM_LINE_MAX - 1;
    }

    portENTER_CRITICAL(&s_lock);

    if (s_q_count == LM_QUEUE_DEPTH)
    {
        s_q_count--; /* drop the oldest: tail advances implicitly */
        s_dropped++;
    }

    lm_msg_t *slot = &s_queue[s_q_head];

    memcpy(slot->text, line, len);
    slot->text[len] = '\0';
    slot->len = (uint16_t)len;
    s_q_head = (s_q_head + 1) % LM_QUEUE_DEPTH;
    s_q_count++;
    portEXIT_CRITICAL(&s_lock);

    if (s_task != NULL)
    {
        xTaskNotifyGive(s_task);
    }
}

static bool dequeue(lm_msg_t *out)
{
    bool got = false;

    portENTER_CRITICAL(&s_lock);

    if (s_q_count > 0)
    {
        uint32_t tail = (s_q_head + LM_QUEUE_DEPTH - s_q_count) %
                        LM_QUEUE_DEPTH;

        *out = s_queue[tail];
        s_q_count--;
        got = true;
    }

    portEXIT_CRITICAL(&s_lock);
    return got;
}

static void log_task(void *arg)
{
    static lm_msg_t msg; /* single consumer; keep it off the stack */

    (void)arg;

    while (s_running)
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));

        while (dequeue(&msg))
        {
            route_to_sinks(msg.text, msg.len);
        }

        uint32_t dropped = s_dropped;

        if (dropped != s_dropped_reported)
        {
            char note[64];
            int n = snprintf(note, sizeof(note),
                             "W log_manager: dropped %lu messages\n",
                             (unsigned long)(dropped - s_dropped_reported));

            s_dropped_reported = dropped;

            if (n > 0)
            {
                route_to_sinks(note, (size_t)n);
            }
        }
    }

    s_task = NULL;
    vTaskDelete(NULL);
}

/** The esp_log vprintf hook: format once, then queue (synchronous pre-start). */
static int lm_vprintf(const char *fmt, va_list args)
{
    char line[LM_LINE_MAX]; /* producer stack; 256 B */
    int len = vsnprintf(line, sizeof(line), fmt, args);

    if (len <= 0)
    {
        return len;
    }

    if ((size_t)len >= sizeof(line))
    {
        len = sizeof(line) - 1; /* truncated */
    }

    if (xPortInIsrContext())
    {
        return len; /* §9.5: no logging from ISRs — drop, never lock */
    }

    if (s_running)
    {
        enqueue(line, (size_t)len);
    }
    else
    {
        route_to_sinks(line, (size_t)len); /* pre-start: synchronous */
    }

    return len;
}

/* ---- lifecycle ------------------------------------------------------------------ */

esp_err_t log_manager_init(void)
{
    if (s_inited)
    {
        return ESP_OK;
    }

    /* adopt the ring across a warm reset; power-on garbage resets it */
    if (!lm_ring_valid(&s_ring.hdr, LM_RING_SIZE))
    {
        lm_ring_reset(&s_ring.hdr, LM_RING_SIZE);
    }

    static const char BOOT_MARK[] = "\n---- boot ----\n";

    lm_ring_append(&s_ring.hdr, s_ring.buf, BOOT_MARK, sizeof(BOOT_MARK) - 1);
    ring_commit();

    s_sinks[0].sink = &S_CONSOLE_SINK;
    s_sinks[0].enabled = s_console_enabled;
    s_sinks[1].sink = &S_RING_SINK;
    s_sinks[1].enabled = s_ring_enabled;
    s_sink_count = 2;

    s_prev_vprintf = esp_log_set_vprintf(lm_vprintf);
    s_inited = true;

    ESP_LOGI(TAG, "pipeline up (queue %dx%d, ring %d KiB, %s)",
             LM_QUEUE_DEPTH, LM_LINE_MAX, LM_RING_SIZE / 1024,
             "sync until start");
    return ESP_OK;
}

esp_err_t log_manager_start(void)
{
    if (!s_inited)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_running)
    {
        return ESP_OK;
    }

    s_running = true;
    s_task = xTaskCreateStatic(log_task, "log_manager",
                               sizeof(s_task_stack) / sizeof(s_task_stack[0]),
                               NULL, 3, s_task_stack, &s_task_tcb);
    ESP_LOGI(TAG, "started");
    return ESP_OK;
}

esp_err_t log_manager_stop(void)
{
    if (!s_running)
    {
        return ESP_OK;
    }

    s_running = false; /* task drains, then exits; hook falls back to sync */

    for (int i = 0; i < 20 && s_task != NULL; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return ESP_OK;
}

/* ---- levels ----------------------------------------------------------------------- */

esp_err_t log_manager_register(const log_descriptor_t *desc)
{
    if (desc == NULL || desc->name == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_level_count >= LM_MAX_LEVELS)
    {
        return ESP_ERR_NO_MEM;
    }

    lm_level_slot_t *slot = &s_levels[s_level_count++];

    strncpy(slot->name, desc->name, sizeof(slot->name) - 1);
    slot->name[sizeof(slot->name) - 1] = '\0';
    slot->level = desc->default_level;
    esp_log_level_set(slot->name, desc->default_level);
    return ESP_OK;
}

esp_err_t log_manager_set_level(const char *name, esp_log_level_t level)
{
    if (name == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_log_level_set(name, level); /* ephemeral: not persisted (§9.4) */
    return ESP_OK;
}

/* ---- sinks ------------------------------------------------------------------------- */

esp_err_t log_manager_add_sink(const log_sink_t *sink)
{
    if (sink == NULL || sink->name == NULL || sink->write == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_sink_count >= LM_MAX_SINKS)
    {
        return ESP_ERR_NO_MEM;
    }

    s_sinks[s_sink_count].sink = sink;
    s_sinks[s_sink_count].enabled = true;
    s_sink_count++;
    ESP_LOGI(TAG, "sink '%s' registered", sink->name);
    return ESP_OK;
}

esp_err_t log_manager_sink_set_enabled(const char *name, bool enabled)
{
    for (size_t i = 0; i < s_sink_count; i++)
    {
        if (strcmp(s_sinks[i].sink->name, name) == 0)
        {
            s_sinks[i].enabled = enabled;
            return ESP_OK;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

esp_err_t log_manager_sink_get(size_t index, const char **name, bool *enabled)
{
    if (index >= s_sink_count)
    {
        return ESP_ERR_NOT_FOUND;
    }

    if (name != NULL)
    {
        *name = s_sinks[index].sink->name;
    }

    if (enabled != NULL)
    {
        *enabled = s_sinks[index].enabled;
    }

    return ESP_OK;
}

uint32_t log_manager_dropped_count(void)
{
    return s_dropped;
}

/* ---- ring access ----------------------------------------------------------------------- */

esp_err_t log_manager_ring_read(char *out, size_t out_len, size_t *out_written)
{
    if (out == NULL || out_len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_lock);

    if (!lm_ring_valid(&s_ring.hdr, LM_RING_SIZE))
    {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }

    size_t n = lm_ring_read(&s_ring.hdr, s_ring.buf, out, out_len);

    portEXIT_CRITICAL(&s_lock);

    if (out_written != NULL)
    {
        *out_written = n;
    }

    return ESP_OK;
}

esp_err_t log_manager_ring_clear(void)
{
    portENTER_CRITICAL(&s_lock);
    lm_ring_reset(&s_ring.hdr, LM_RING_SIZE);
    portEXIT_CRITICAL(&s_lock);
    ring_commit();
    return ESP_OK;
}

/* used by log_manager_settings.c (component-private) */
void lm_apply_sink_config(bool console_enabled, bool ring_enabled)
{
    s_console_enabled = console_enabled;
    s_ring_enabled = ring_enabled;
    log_manager_sink_set_enabled("console", console_enabled);
    log_manager_sink_set_enabled("ring", ring_enabled);
}
