#include "usb_cli_console.h"

#include <string.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "esp_console.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_vfs.h"

#include "gps.h"
#include "imu.h"
#include "lte_upstream_pppos.h"
#include "config_manager.h"

#include "usb_cli_commands.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

#include "usbd_core.h"
#include "usbd_cdc_acm.h"

// Endpoints chosen to not collide with RNDIS endpoints:
// RNDIS uses 0x81 IN, 0x02 OUT, 0x83 INT.
#define USB_CLI_CONSOLE_OUT_EP 0x04
#define USB_CLI_CONSOLE_IN_EP  0x82
#define USB_CLI_CONSOLE_INT_EP 0x84

#ifndef USB_CLI_RX_BUF_SIZE
#define USB_CLI_RX_BUF_SIZE 256
#endif

#ifndef USB_CLI_LINE_BUF_SIZE
#define USB_CLI_LINE_BUF_SIZE 128
#endif

#define USB_CLI_PROMPT "esp> "

static bool s_enabled = false;
static TaskHandle_t s_task = NULL;
static volatile bool s_dtr = false;
static volatile bool s_tx_busy = false;
static volatile bool s_configured = false;
static volatile bool s_gps_stream_mode = false;
static volatile bool s_gps_stream_exit_requested = false;

static portMUX_TYPE s_line_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_tx_mux = portMUX_INITIALIZER_UNLOCKED;

static uint8_t s_rx_ep_buf[USB_CLI_RX_BUF_SIZE];
static char s_line_buf[USB_CLI_LINE_BUF_SIZE];
static size_t s_line_len = 0;
static uint8_t s_busid = 0;

// Escape detection state (used in GPS stream mode)
static uint8_t s_escape_len = 0;
static TickType_t s_escape_last_tick = 0;

// TX ring buffer so printf/help output doesn't get dropped.
#ifndef USB_CLI_TX_BUF_SIZE
#define USB_CLI_TX_BUF_SIZE 2048
#endif

static uint8_t s_tx_buf[USB_CLI_TX_BUF_SIZE];
static size_t s_tx_head = 0;
static size_t s_tx_tail = 0;

static struct usbd_interface s_intf0;
static struct usbd_interface s_intf1;

static bool s_esp_console_inited = false;
static bool s_stdio_redirect_inited = false;
static bool s_vfs_registered = false;

static void usb_cli_console_init_esp_console_once(void)
{
    if (s_esp_console_inited)
    {
        return;
    }

    const esp_console_config_t console_cfg = {
        .max_cmdline_length = 256,
        .max_cmdline_args = 8,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .hint_color = 0,
#endif
    };

    (void)esp_console_init(&console_cfg);
    esp_console_register_help_command();
    usb_cli_register_console_commands();
    gps_console_register();
    imu_console_register();
    lte_upstream_pppos_console_register();
    config_manager_console_register();

    s_esp_console_inited = true;
}

void usb_cli_console_set_gps_stream_mode(bool enabled)
{
    s_gps_stream_mode = enabled;
    s_escape_len = 0;
    s_escape_last_tick = 0;
    s_gps_stream_exit_requested = false;
}

static bool usb_cli_tx_ring_is_empty(void)
{
    return s_tx_head == s_tx_tail;
}

static size_t usb_cli_tx_ring_count_unsafe(void)
{
    if (s_tx_head >= s_tx_tail)
    {
        return s_tx_head - s_tx_tail;
    }
    return USB_CLI_TX_BUF_SIZE - (s_tx_tail - s_tx_head);
}

static size_t usb_cli_tx_ring_free_unsafe(void)
{
    // Leave one byte empty so head==tail means empty.
    return (USB_CLI_TX_BUF_SIZE - 1) - usb_cli_tx_ring_count_unsafe();
}

static size_t usb_cli_tx_ring_push_unsafe(const uint8_t *data, size_t len)
{
    size_t pushed = 0;

    while (pushed < len)
    {
        if (usb_cli_tx_ring_free_unsafe() == 0)
        {
            break;
        }

        s_tx_buf[s_tx_head] = data[pushed];
        s_tx_head = (s_tx_head + 1) % USB_CLI_TX_BUF_SIZE;
        pushed++;
    }

    return pushed;
}

static size_t usb_cli_tx_ring_pop_chunk_unsafe(uint8_t *out, size_t max_len)
{
    size_t n = 0;
    while (n < max_len && !usb_cli_tx_ring_is_empty())
    {
        out[n++] = s_tx_buf[s_tx_tail];
        s_tx_tail = (s_tx_tail + 1) % USB_CLI_TX_BUF_SIZE;
    }
    return n;
}

static void usb_cli_tx_kick_from_any_context(uint8_t busid)
{
    if (!s_enabled || !s_configured || !s_dtr)
    {
        return;
    }

    if (s_tx_busy)
    {
        return;
    }

    uint8_t chunk[64];
    size_t chunk_len = 0;

    portENTER_CRITICAL(&s_tx_mux);
    if (!usb_cli_tx_ring_is_empty())
    {
        chunk_len = usb_cli_tx_ring_pop_chunk_unsafe(chunk, sizeof(chunk));
        if (chunk_len > 0)
        {
            s_tx_busy = true;
        }
    }
    portEXIT_CRITICAL(&s_tx_mux);

    if (chunk_len > 0)
    {
        (void)usbd_ep_start_write(busid, USB_CLI_CONSOLE_IN_EP, chunk, (uint32_t)chunk_len);
    }
}

static void usb_cli_tx_enqueue_from_task(const void *data, size_t len)
{
    if (!s_enabled)
    {
        return;
    }

    if (!s_dtr)
    {
        return;
    }

    if (!data || len == 0)
    {
        return;
    }

    // Best-effort enqueue; drop overflow to avoid blocking.
    // Convert \n to \r\n for Windows terminals (but keep existing \r\n intact).
    const uint8_t *bytes = (const uint8_t *)data;

    portENTER_CRITICAL(&s_tx_mux);

    uint8_t prev = 0;
    if (!usb_cli_tx_ring_is_empty())
    {
        // Best-effort: infer prev from last enqueued byte.
        size_t last = (s_tx_head == 0) ? (USB_CLI_TX_BUF_SIZE - 1) : (s_tx_head - 1);
        prev = s_tx_buf[last];
    }

    for (size_t i = 0; i < len; i++)
    {
        const uint8_t c = bytes[i];

        if (c == '\n' && prev != '\r')
        {
            (void)usb_cli_tx_ring_push_unsafe((const uint8_t *)"\r", 1);
        }

        (void)usb_cli_tx_ring_push_unsafe(&c, 1);
        prev = c;
    }

    portEXIT_CRITICAL(&s_tx_mux);

    usb_cli_tx_kick_from_any_context(s_busid);
}

// Minimal VFS device so esp_console/help/printf output can go to CDC-ACM.
// NOTE: This redirects *global* stdout/stderr once enabled.
static int usb_cli_vfs_open(const char *path, int flags, int mode)
{
    (void)path;
    (void)flags;
    (void)mode;
    return 0;
}

static int usb_cli_vfs_close(int fd)
{
    (void)fd;
    return 0;
}

static ssize_t usb_cli_vfs_write(int fd, const void *data, size_t size)
{
    (void)fd;

    if (!data || size == 0)
    {
        return 0;
    }

    usb_cli_tx_enqueue_from_task(data, size);
    return (ssize_t)size;
}

static int usb_cli_vfs_fstat(int fd, struct stat *st)
{
    (void)fd;
    if (!st)
    {
        errno = EINVAL;
        return -1;
    }
    memset(st, 0, sizeof(*st));
    st->st_mode = S_IFCHR;
    return 0;
}

static int usb_cli_vfs_fcntl(int fd, int cmd, int arg)
{
    (void)fd;
    (void)cmd;
    (void)arg;
    return 0;
}

static void usb_cli_console_init_stdio_redirect_once(void)
{
    if (s_stdio_redirect_inited)
    {
        return;
    }

    if (!s_vfs_registered)
    {
        static const esp_vfs_t vfs = {
            .open = &usb_cli_vfs_open,
            .close = &usb_cli_vfs_close,
            .write = &usb_cli_vfs_write,
            .fstat = &usb_cli_vfs_fstat,
            .fcntl = &usb_cli_vfs_fcntl,
        };

        // If already registered (e.g., after a soft restart), ignore error.
        (void)esp_vfs_register("/dev/usb_cli", &vfs, NULL);
        s_vfs_registered = true;
    }

    (void)freopen("/dev/usb_cli", "w", stdout);
    (void)freopen("/dev/usb_cli", "w", stderr);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    s_stdio_redirect_inited = true;
}

static void usb_cli_console_task(void *arg)
{
    (void)arg;

    // Wait until the host opens the COM port (DTR) and we are configured,
    // otherwise early output is likely to be dropped.
    while (!s_configured)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    while (!s_dtr)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    usb_cli_console_init_stdio_redirect_once();
    usb_cli_console_init_esp_console_once();

    printf("ESPNetLink console. Type 'help'.\n");
    printf("\r\n" USB_CLI_PROMPT);

    while (true)
    {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (s_gps_stream_exit_requested)
        {
            s_gps_stream_exit_requested = false;
            s_gps_stream_mode = false;
            gps_set_streaming(false);
            printf("\n[console mode]\n");
            printf("\r\n" USB_CLI_PROMPT);
            continue;
        }

        if (s_gps_stream_mode)
        {
            // In stream mode we ignore normal line execution.
            continue;
        }

        // Process a complete line if we have one.
        // (We notify on every RX, but only act when line contains '\n' or '\r').
        // The RX callback updates s_line_buf/s_line_len.

        // Snapshot and clear line (single-writer in RX cb + single-reader here is OK for this small use)
        // We keep it simple; if you want strict concurrency later, we can add a mutex.
        char line[USB_CLI_LINE_BUF_SIZE];
        size_t len = 0;

        portENTER_CRITICAL(&s_line_mux);
        if (s_line_len > 0 && s_line_len < sizeof(line))
        {
            memcpy(line, s_line_buf, s_line_len);
            len = s_line_len;
            s_line_len = 0;
        }
        portEXIT_CRITICAL(&s_line_mux);

        if (len == 0)
        {
            continue;
        }

        line[len] = '\0';

        // Trim whitespace
        while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n' || line[len - 1] == ' ' || line[len - 1] == '\t'))
        {
            line[len - 1] = '\0';
            len--;
        }

        if (len == 0)
        {
            printf("\r\n" USB_CLI_PROMPT);
            continue;
        }

        // Always start the command output on a fresh line.
        // When echo is disabled the host doesn't see a newline after typing;
        // when echo is enabled the typed characters are already on the line.
        // Either way, move to the next line before printing any response.
        printf("\r\n");

        int ret = 0;
        esp_err_t err = esp_console_run(line, &ret);
        bool cmd_success = false;
        if (err == ESP_ERR_NOT_FOUND)
        {
            printf("Unrecognized command\n");
        }
        else if (err == ESP_ERR_INVALID_ARG)
        {
            printf("Invalid arguments\n");
        }
        else if (err != ESP_OK)
        {
            printf("Command error: %s\n", esp_err_to_name(err));
        }
        else
        {
            cmd_success = (ret == 0);
        }

        if (!s_gps_stream_mode)
        {
            printf("%s\r\n" USB_CLI_PROMPT, cmd_success ? "OK" : "ERROR");
        }
    }
}

static void usb_cli_start_task_if_needed(void)
{
    if (s_task)
    {
        return;
    }

    xTaskCreatePinnedToCore(usb_cli_console_task, "usb_cli", 4096, NULL, 8, &s_task, 0);
}

static void usb_cli_kick_task(void)
{
    if (!s_task)
    {
        return;
    }

    xTaskNotifyGive(s_task);
}

static void usb_cli_arm_out_read(uint8_t busid)
{
    (void)usbd_ep_start_read(busid, USB_CLI_CONSOLE_OUT_EP, s_rx_ep_buf, sizeof(s_rx_ep_buf));
}

static void usb_cli_on_rx_bytes(const uint8_t *data, size_t len)
{
    if (s_gps_stream_mode)
    {
        // Same escape behavior as usb_dev_bridge:
        // send 7+ '+' (or '=') then Enter to exit stream mode.
        // While streaming we ignore other input.
        for (size_t i = 0; i < len; i++)
        {
            const char c = (char)data[i];

            if (c == '+' || c == '=')
            {
                if (s_escape_len < 32)
                {
                    s_escape_len++;
                    s_escape_last_tick = xTaskGetTickCount();
                }
                continue;
            }

            if (c == '\r' || c == '\n')
            {
                if (s_escape_len >= 7)
                {
                    s_escape_len = 0;
                    s_escape_last_tick = 0;
                    s_gps_stream_exit_requested = true;
                    usb_cli_kick_task();
                    continue;
                }

                s_escape_len = 0;
                s_escape_last_tick = 0;
                continue;
            }

            // Timeout similar to usb_dev_bridge: if user started an escape but paused, reset.
            if (s_escape_len > 0 && s_escape_last_tick != 0)
            {
                TickType_t now = xTaskGetTickCount();
                if ((now - s_escape_last_tick) > pdMS_TO_TICKS(500))
                {
                    s_escape_len = 0;
                    s_escape_last_tick = 0;
                }
            }

            // Any other character resets escape tracking.
            s_escape_len = 0;
            s_escape_last_tick = 0;
        }
        return;
    }

    for (size_t i = 0; i < len; i++)
    {
        char c = (char)data[i];

        if (c == '\r' || c == '\n')
        {
            // finalize line
            if (s_line_len < (sizeof(s_line_buf) - 1))
            {
                s_line_buf[s_line_len++] = '\n';
            }
            usb_cli_kick_task();
            continue;
        }

        if (c == '\b' || c == 0x7F)
        {
            if (s_line_len > 0)
            {
                s_line_len--;
                if (usb_cli_echo_is_enabled())
                {
                    // Echo backspace sequence: move left, erase, move left.
                    usb_cli_tx_enqueue_from_task("\b \b", 3);
                }
            }
            continue;
        }

        if (s_line_len < (sizeof(s_line_buf) - 1))
        {
            s_line_buf[s_line_len++] = c;
            if (usb_cli_echo_is_enabled())
            {
                // Echo typed character.
                usb_cli_tx_enqueue_from_task(&c, 1);
            }
        }
    }
}

void usb_cli_console_set_enabled(bool enabled)
{
    s_enabled = enabled;
}

bool usb_cli_console_is_enabled(void)
{
    return s_enabled;
}

// Endpoint callbacks (run in USB context; keep them minimal)
static void usb_cli_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)ep;

    if (!s_enabled)
    {
        usb_cli_arm_out_read(busid);
        return;
    }

    if (nbytes > sizeof(s_rx_ep_buf))
    {
        nbytes = sizeof(s_rx_ep_buf);
    }

    usb_cli_on_rx_bytes(s_rx_ep_buf, (size_t)nbytes);

    usb_cli_arm_out_read(busid);
}

static void usb_cli_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;

    // Mark idle and send next queued chunk (if any).
    (void)ep;
    (void)nbytes;
    s_tx_busy = false;
    usb_cli_tx_kick_from_any_context(busid);
}

static struct usbd_endpoint s_out_ep = {
    .ep_addr = USB_CLI_CONSOLE_OUT_EP,
    .ep_cb = usb_cli_bulk_out,
};

static struct usbd_endpoint s_in_ep = {
    .ep_addr = USB_CLI_CONSOLE_IN_EP,
    .ep_cb = usb_cli_bulk_in,
};

static void usb_cli_int_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;
    (void)nbytes;
}

static struct usbd_endpoint s_int_ep = {
    .ep_addr = USB_CLI_CONSOLE_INT_EP,
    .ep_cb = usb_cli_int_in,
};

void usb_cli_console_usb_add(uint8_t busid)
{
    if (!s_enabled)
    {
        return;
    }

    s_busid = busid;

    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &s_intf0));
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &s_intf1));
    usbd_add_endpoint(busid, &s_int_ep);
    usbd_add_endpoint(busid, &s_out_ep);
    usbd_add_endpoint(busid, &s_in_ep);

    usb_cli_start_task_if_needed();
}

void usb_cli_console_usb_on_configured(uint8_t busid)
{
    if (!s_enabled)
    {
        return;
    }

    s_busid = busid;
    s_tx_busy = false;
    s_configured = true;
    s_line_len = 0;

    portENTER_CRITICAL(&s_tx_mux);
    s_tx_head = 0;
    s_tx_tail = 0;
    portEXIT_CRITICAL(&s_tx_mux);

    usb_cli_arm_out_read(busid);
}

// Override weak hooks so we can gate output on DTR.
void usbd_cdc_acm_set_dtr(uint8_t busid, uint8_t intf, bool dtr)
{
    (void)busid;
    (void)intf;

    s_dtr = dtr;
}
