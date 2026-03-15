#include "usb_cli_commands.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "esp_chip_info.h"
#include "esp_console.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "hw_config.h"

#include "gps.h"
#include "lte_upstream_pppos.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "lwip/inet.h"
#include "lwip/ip_addr.h"
#include "ping/ping_sock.h"

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static bool s_echo_enabled  = false;
static bool s_debug_enabled = false;

// ---------------------------------------------------------------------------
// Public getter used by usb_cli_console.c
// ---------------------------------------------------------------------------

bool usb_cli_echo_is_enabled(void)
{
    return s_echo_enabled;
}

// ---------------------------------------------------------------------------
// Command handlers
// ---------------------------------------------------------------------------

static int usb_cli_cmd_ver(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("ESPNetLink USB CLI (CDC-ACM)\n");
    printf("FW %s\n", GIT_SHA);
    printf("ESP-IDF %s\n", esp_get_idf_version());
    return 0;
}

static int usb_cli_cmd_echo(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("echo %d\n", s_echo_enabled ? 1 : 0);
        return 0;
    }

    const int v = atoi(argv[1]);
    if (v == 0)
    {
        s_echo_enabled = false;
        printf("echo 0\n");
        return 0;
    }
    if (v == 1)
    {
        s_echo_enabled = true;
        printf("echo 1\n");
        return 0;
    }

    printf("Usage: echo 0|1\n");
    return 1;
}

static int usb_cli_cmd_debug(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("debug %d\n", s_debug_enabled ? 1 : 0);
        return 0;
    }

    const int v = atoi(argv[1]);
    if (v == 0)
    {
        s_debug_enabled = false;
        esp_log_level_set("*", ESP_LOG_NONE);
        printf("debug 0\n");
        return 0;
    }
    if (v == 1)
    {
        s_debug_enabled = true;
        esp_log_level_set("*", ESP_LOG_DEBUG);
        printf("debug 1\n");
        return 0;
    }

    printf("Usage: debug 0|1\n");
    return 1;
}

static int usb_cli_cmd_system(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("Usage: system -v | -i | -s <unix_timestamp>\n");
        return 1;
    }

    if (strcmp(argv[1], "-s") == 0)
    {
        if (argc < 3)
        {
            printf("Usage: system -s <unix_timestamp>\n");
            return 1;
        }
        long ts = atol(argv[2]);
        if (ts <= 0)
        {
            printf("Invalid timestamp\n");
            return 1;
        }
        struct timeval tv = { .tv_sec = ts, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        lte_upstream_pppos_mark_time_synced();

        struct tm utc;
        time_t now = ts;
        gmtime_r(&now, &utc);
        printf("System time set to %04d-%02d-%02d %02d:%02d:%02d UTC\n",
               utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
               utc.tm_hour, utc.tm_min, utc.tm_sec);

        agnss_try_inject_time();
        return 0;
    }

    if (strcmp(argv[1], "-v") == 0)
    {
        printf("espnetlink-fw_" GIT_SHA "\n");
        return 0;
    }

    if (strcmp(argv[1], "-i") == 0)
    {
        esp_chip_info_t chip;
        esp_chip_info(&chip);

        const char *model_str;
        switch (chip.model)
        {
            case CHIP_ESP32S3: model_str = "ESP32-S3"; break;
            case CHIP_ESP32S2: model_str = "ESP32-S2"; break;
            case CHIP_ESP32C3: model_str = "ESP32-C3"; break;
            case CHIP_ESP32:   model_str = "ESP32";    break;
            default:           model_str = "Unknown";  break;
        }

        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);

        printf("FW version   : espnetlink-fw_" GIT_SHA "\n");
        printf("ESP-IDF      : %s\n", esp_get_idf_version());
        printf("Chip model   : %s rev %d\n", model_str, chip.revision);
        printf("CPU cores    : %d\n", chip.cores);
        printf("Free heap    : %lu bytes\n", (unsigned long)esp_get_free_heap_size());
        printf("Min free heap: %lu bytes\n", (unsigned long)esp_get_minimum_free_heap_size());
        printf("MAC (STA)    : %02X:%02X:%02X:%02X:%02X:%02X\n",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        return 0;
    }

    printf("Usage: system -v | -i | -s <unix_timestamp>\n");
    return 1;
}

static int usb_cli_cmd_reboot(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("Rebooting...\n");
    /* Cleanly exit CMUX/PPP so the modem returns to AT command mode.
     * Without this, the modem stays in CMUX after ESP restart and
     * modem_mgr_init has to power-cycle it (killing LTE registration). */
    (void)lte_upstream_pppos_stop();
    printf("\n\n\n");
    vTaskDelay(pdMS_TO_TICKS(1000));  // allow message to flush before reboot
    esp_restart();
    return 0;
}

// ---------------------------------------------------------------------------
// Ping
// ---------------------------------------------------------------------------

typedef struct {
    SemaphoreHandle_t done;
    uint32_t          recv;
    uint32_t          timeout_cnt;
    uint32_t          sum_ms;
    uint32_t          min_ms;
    uint32_t          max_ms;
} ping_ctx_t;

static void ping_on_success(esp_ping_handle_t hdl, void *args)
{
    ping_ctx_t *ctx = (ping_ctx_t *)args;
    uint8_t  ttl;
    uint16_t seq;
    uint32_t elapsed, size;
    ip_addr_t addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO,   &seq,     sizeof(seq));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TTL,     &ttl,     sizeof(ttl));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR,  &addr,    sizeof(addr));
    esp_ping_get_profile(hdl, ESP_PING_PROF_SIZE,    &size,    sizeof(size));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed, sizeof(elapsed));
    ctx->recv++;
    ctx->sum_ms += elapsed;
    if (elapsed < ctx->min_ms) ctx->min_ms = elapsed;
    if (elapsed > ctx->max_ms) ctx->max_ms = elapsed;
    printf("Reply from %s: bytes=%lu ttl=%d time=%lu ms\n",
           inet_ntoa(addr.u_addr.ip4), (unsigned long)size, ttl, (unsigned long)elapsed);
}

static void ping_on_timeout(esp_ping_handle_t hdl, void *args)
{
    ping_ctx_t *ctx = (ping_ctx_t *)args;
    ctx->timeout_cnt++;
    printf("Request timed out\n");
}

static void ping_on_end(esp_ping_handle_t hdl, void *args)
{
    ping_ctx_t *ctx = (ping_ctx_t *)args;
    xSemaphoreGive(ctx->done);
}

static int usb_cli_cmd_ping(int argc, char **argv)
{
    const char *host     = "8.8.8.8";
    uint32_t    count    = 4;
    uint32_t    interval = 1000;

    for (int i = 1; i < argc; i++)
    {
        if      (strcmp(argv[i], "-c") == 0 && i + 1 < argc) { count    = (uint32_t)atoi(argv[++i]); }
        else if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) { interval = (uint32_t)atoi(argv[++i]); }
        else if (argv[i][0] != '-')                           { host = argv[i]; }
    }

    ip_addr_t target;
    if (!ipaddr_aton(host, &target))
    {
        printf("ping: cannot parse address '%s' -- only IP addresses supported\n", host);
        return 1;
    }

    ping_ctx_t ctx = {
        .done        = xSemaphoreCreateBinary(),
        .recv        = 0,
        .timeout_cnt = 0,
        .sum_ms      = 0,
        .min_ms      = UINT32_MAX,
        .max_ms      = 0,
    };
    if (!ctx.done) { printf("ping: out of memory\n"); return 1; }

    esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
    cfg.target_addr    = target;
    cfg.count          = count;
    cfg.interval_ms    = interval;
    cfg.timeout_ms     = 5000;
    cfg.data_size      = 32;
    cfg.task_stack_size = 4096;
    cfg.task_prio      = 3;

    esp_ping_callbacks_t cbs = {
        .cb_args        = &ctx,
        .on_ping_success = ping_on_success,
        .on_ping_timeout = ping_on_timeout,
        .on_ping_end     = ping_on_end,
    };

    esp_ping_handle_t hdl;
    esp_err_t err = esp_ping_new_session(&cfg, &cbs, &hdl);
    if (err != ESP_OK)
    {
        vSemaphoreDelete(ctx.done);
        printf("ping: %s\n", esp_err_to_name(err));
        return 1;
    }

    printf("Pinging %s with 32 bytes (%lu packets):\n", host, (unsigned long)count);
    esp_ping_start(hdl);

    /* Wait long enough: (interval + 5s timeout) per packet + 2s margin */
    uint32_t wait_ms = count * (interval + 5000) + 2000;
    bool finished = (xSemaphoreTake(ctx.done, pdMS_TO_TICKS(wait_ms)) == pdTRUE);

    esp_ping_stop(hdl);
    esp_ping_delete_session(hdl);
    vSemaphoreDelete(ctx.done);

    uint32_t sent = ctx.recv + ctx.timeout_cnt;
    printf("\n--- %s ping statistics ---\n", host);
    printf("  Packets: sent=%lu  received=%lu  lost=%lu (%.0f%% loss)\n",
           (unsigned long)sent, (unsigned long)ctx.recv, (unsigned long)ctx.timeout_cnt,
           sent > 0 ? (ctx.timeout_cnt * 100.0f / sent) : 0.0f);
    if (ctx.recv > 0 && ctx.min_ms != UINT32_MAX)
    {
        printf("  RTT    : min=%lu ms  avg=%lu ms  max=%lu ms\n",
               (unsigned long)ctx.min_ms,
               (unsigned long)(ctx.sum_ms / ctx.recv),
               (unsigned long)ctx.max_ms);
    }
    if (!finished)
    {
        printf("  (WARNING: wait timed out before ping session ended)\n");
    }
    return (ctx.recv > 0) ? 0 : 1;
}

// ---------------------------------------------------------------------------
// Speed test
// ---------------------------------------------------------------------------

#define SPEEDTEST_DOWN_URL  "https://speed.cloudflare.com/__down?bytes=%d"
#define SPEEDTEST_UP_URL    "https://speed.cloudflare.com/__up"
#define SPEEDTEST_DOWN_DEF  102400   /* 100 KB */
#define SPEEDTEST_UP_DEF    51200    /*  50 KB */

static esp_err_t speedtest_dl_event_cb(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_DATA && evt->user_data)
    {
        *(size_t *)evt->user_data += (size_t)evt->data_len;
    }
    return ESP_OK;
}

static int speedtest_run_download(int bytes)
{
    char url[128];
    snprintf(url, sizeof(url), SPEEDTEST_DOWN_URL, bytes);

    size_t rx = 0;
    esp_http_client_config_t cfg = {
        .url                  = url,
        .event_handler        = speedtest_dl_event_cb,
        .user_data            = &rx,
        .crt_bundle_attach    = esp_crt_bundle_attach,
        .buffer_size          = 4096,
        .timeout_ms           = 40000,
        .disable_auto_redirect = false,
    };

    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) { printf("Download: HTTP client init failed\n"); return 1; }

    int64_t t0 = esp_timer_get_time();
    esp_err_t err = esp_http_client_perform(c);
    int64_t elapsed_us = esp_timer_get_time() - t0;
    int status = esp_http_client_get_status_code(c);
    esp_http_client_cleanup(c);

    if (err != ESP_OK)
    {
        printf("Download: %s\n", esp_err_to_name(err));
        return 1;
    }
    if (status < 200 || status >= 300)
    {
        printf("Download: HTTP %d\n", status);
        return 1;
    }

    double s = elapsed_us / 1e6;
    printf("Download : %zu B in %.2f s  ->  %.1f kbps  (%.1f kB/s)\n",
           rx, s, rx * 8.0 / (s * 1000.0), rx / (s * 1024.0));
    return 0;
}

static int speedtest_run_upload(int bytes)
{
    uint8_t *buf = malloc(bytes);
    if (!buf) { printf("Upload: out of memory\n"); return 1; }
    memset(buf, 0xAA, bytes);

    esp_http_client_config_t cfg = {
        .url               = SPEEDTEST_UP_URL,
        .method            = HTTP_METHOD_POST,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size       = 4096,
        .timeout_ms        = 40000,
    };

    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) { free(buf); printf("Upload: HTTP client init failed\n"); return 1; }

    esp_http_client_set_post_field(c, (const char *)buf, bytes);

    int64_t t0 = esp_timer_get_time();
    esp_err_t err = esp_http_client_perform(c);
    int64_t elapsed_us = esp_timer_get_time() - t0;
    int status = esp_http_client_get_status_code(c);
    esp_http_client_cleanup(c);
    free(buf);

    if (err != ESP_OK)
    {
        printf("Upload: %s\n", esp_err_to_name(err));
        return 1;
    }
    if (status < 200 || status >= 300)
    {
        printf("Upload: HTTP %d\n", status);
        return 1;
    }

    double s = elapsed_us / 1e6;
    printf("Upload   : %d B in %.2f s  ->  %.1f kbps  (%.1f kB/s)\n",
           bytes, s, bytes * 8.0 / (s * 1000.0), bytes / (s * 1024.0));
    return 0;
}

typedef struct {
    bool             do_down;
    bool             do_up;
    int              down_bytes;
    int              up_bytes;
    int              result;
    SemaphoreHandle_t done;
} speedtest_args_t;

static void speedtest_task(void *arg)
{
    speedtest_args_t *a = (speedtest_args_t *)arg;
    a->result = 0;
    if (a->do_down && speedtest_run_download(a->down_bytes) != 0) { a->result = 1; }
    if (a->do_up   && speedtest_run_upload(a->up_bytes)     != 0) { a->result = 1; }
    xSemaphoreGive(a->done);
    vTaskDelete(NULL);
}

static int usb_cli_cmd_speedtest(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "-h") == 0)
    {
        printf("Usage: speedtest [-d] [-u] [-b <bytes>]\n");
        printf("  (no flags)  download + upload\n");
        printf("  -d          download only\n");
        printf("  -u          upload only\n");
        printf("  -b <n>      bytes to download (default %d); upload uses half\n",
               SPEEDTEST_DOWN_DEF);
        return 0;
    }

    speedtest_args_t *a = calloc(1, sizeof(*a));
    if (!a) { printf("Out of memory\n"); return 1; }

    a->do_down    = true;
    a->do_up      = true;
    a->down_bytes = SPEEDTEST_DOWN_DEF;
    a->up_bytes   = SPEEDTEST_UP_DEF;
    a->done       = xSemaphoreCreateBinary();
    if (!a->done) { free(a); printf("Out of memory\n"); return 1; }

    for (int i = 1; i < argc; i++)
    {
        if      (strcmp(argv[i], "-d") == 0) { a->do_up   = false; }
        else if (strcmp(argv[i], "-u") == 0) { a->do_down = false; }
        else if (strcmp(argv[i], "-b") == 0 && i + 1 < argc)
        {
            int b = atoi(argv[++i]);
            if (b > 0) { a->down_bytes = b; a->up_bytes = b / 2; }
        }
    }

    printf("--- Speed Test ---\n");
    if (a->do_down) { printf("Download : %d bytes from Cloudflare\n", a->down_bytes); }
    if (a->do_up)   { printf("Upload   : %d bytes to Cloudflare\n",   a->up_bytes); }
    printf("(running, please wait...)\n\n");

    if (xTaskCreate(speedtest_task, "speedtest", 8192, a, 5, NULL) != pdPASS)
    {
        vSemaphoreDelete(a->done);
        free(a);
        printf("Failed to create speedtest task\n");
        return 1;
    }

    bool ok = (xSemaphoreTake(a->done, pdMS_TO_TICKS(90000)) == pdTRUE);
    int  ret = ok ? a->result : 1;
    if (!ok) { printf("Speed test timed out\n"); }

    vSemaphoreDelete(a->done);
    free(a);
    return ret;
}

// ---------------------------------------------------------------------------
// Sleep / Wake
// ---------------------------------------------------------------------------

static volatile bool s_sleep_mode_active = false;

static void sleep_power_off_peripherals(void)
{
    // Stop LTE PPPoS stack and power off modem
    printf("Stopping LTE stack...\n");
    (void)lte_upstream_pppos_stop();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Ensure STATUS pin is configured as input before reading
    gpio_reset_pin(LTE_STATUS_PIN);
    gpio_set_direction(LTE_STATUS_PIN, GPIO_MODE_INPUT);

    // Ensure PWR_KEY is configured as output
    gpio_reset_pin(LTE_PWR_KEY_PIN);
    gpio_set_direction(LTE_PWR_KEY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LTE_PWR_KEY_PIN, 0);

    // Check if modem is already off
    if (gpio_get_level(LTE_STATUS_PIN) != 0)
    {
        printf("LTE modem already OFF\n");
    }
    else
    {
        // Graceful modem power-off: 1500ms PWR_KEY pulse
        printf("Powering off LTE modem...\n");
        gpio_set_level(LTE_PWR_KEY_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1500));
        gpio_set_level(LTE_PWR_KEY_PIN, 0);

        // Wait for STATUS pin to go HIGH (modem OFF), up to 30s
        for (int i = 0; i < 60; i++)
        {
            if (gpio_get_level(LTE_STATUS_PIN) != 0)
            {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        printf("LTE modem %s\n",
               gpio_get_level(LTE_STATUS_PIN) != 0 ? "OFF" : "still on (timeout)");
    }

    // Power off GPS module
    printf("Powering off GPS...\n");
    gpio_set_level(GPS_PWR_EN_PIN, 0);

    // Turn off LEDs
    gpio_set_level(LED_RED_PIN, 1);  // active-low: HIGH = OFF

    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);
}

static int usb_cli_cmd_sleep(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("Usage: sleep <mode> [seconds]\n");
        printf("  sleep 1          - Light sleep: power off GPS/LTE, keep USB active\n");
        printf("                     Wake with 'wake' command\n");
        printf("  sleep 2 [secs]   - Deep sleep: power off everything including USB\n");
        printf("                     Wake on timer (if secs given) or reset button\n");
        printf("  sleep 3          - Deep sleep: USB mux off, wake on WAKE_GPIO\n");
        return 1;
    }

    const int mode = atoi(argv[1]);

    if (mode == 1)
    {
        printf("Entering low-power mode (USB active)...\n");
        sleep_power_off_peripherals();
        s_sleep_mode_active = true;
        printf("Low-power mode active. Type 'wake' to restore.\n");
        return 0;
    }

    if (mode == 2)
    {
        uint64_t sleep_us = 0;
        if (argc >= 3)
        {
            long secs = atol(argv[2]);
            if (secs <= 0)
            {
                printf("Invalid duration\n");
                return 1;
            }
            sleep_us = (uint64_t)secs * 1000000ULL;
            printf("Entering deep sleep for %ld seconds...\n", secs);
            esp_sleep_enable_timer_wakeup(sleep_us);
        }
        else
        {
            printf("Entering deep sleep (wake on reset only)...\n");
        }

        sleep_power_off_peripherals();
        vTaskDelay(pdMS_TO_TICKS(500));  // flush output

        printf("Sleeping now.\n\n\n");
        vTaskDelay(pdMS_TO_TICKS(200));  // let last message flush

        esp_deep_sleep_start();
        // Does not return
        return 0;
    }

    if (mode == 3)
    {
        printf("Entering deep sleep (USB mux off, wake on WAKE_GPIO)...\n");

        sleep_power_off_peripherals();

        // Disconnect USB mux and hold LOW during deep sleep
        gpio_set_direction(USB_SEL_1, GPIO_MODE_OUTPUT);
        gpio_set_level(USB_SEL_1, 0);
        gpio_sleep_set_pull_mode(USB_SEL_1, GPIO_PULLDOWN_ONLY);
        gpio_pulldown_en(USB_SEL_1);
        rtc_gpio_pulldown_en(USB_SEL_1);
        gpio_hold_en(USB_SEL_1);
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Configure WAKE_GPIO as deep sleep wakeup source (wake on LOW)
        rtc_gpio_init(WAKE_GPIO);
        rtc_gpio_set_direction(WAKE_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pulldown_dis(WAKE_GPIO);
        rtc_gpio_pullup_en(WAKE_GPIO);
        esp_sleep_enable_ext1_wakeup_io(1ULL << WAKE_GPIO,
                                        ESP_EXT1_WAKEUP_ALL_LOW);

        printf("Sleeping now.\n\n\n");
        vTaskDelay(pdMS_TO_TICKS(200));

        esp_deep_sleep_start();
        return 0;
    }

    printf("Unknown mode %d. Use 1, 2, or 3.\n", mode);
    return 1;
}

static int usb_cli_cmd_wake(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (!s_sleep_mode_active)
    {
        printf("Device is not in sleep mode\n");
        return 1;
    }

    printf("Waking up — rebooting to re-initialize peripherals...\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return 0;
}

// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------

void usb_cli_register_console_commands(void)
{
    const esp_console_cmd_t cmd_ver = {
        .command = "ver",
        .help    = "Print firmware/version info",
        .hint    = NULL,
        .func    = &usb_cli_cmd_ver,
    };
    (void)esp_console_cmd_register(&cmd_ver);

    const esp_console_cmd_t cmd_echo = {
        .command = "echo",
        .help    = "Enable/disable device-side input echo (0|1)",
        .hint    = NULL,
        .func    = &usb_cli_cmd_echo,
    };
    (void)esp_console_cmd_register(&cmd_echo);

    const esp_console_cmd_t cmd_debug = {
        .command = "debug",
        .help    = "Enable/disable global log output (0=off, 1=on)",
        .hint    = NULL,
        .func    = &usb_cli_cmd_debug,
    };
    (void)esp_console_cmd_register(&cmd_debug);

    const esp_console_cmd_t cmd_system = {
        .command = "system",
        .help    = "System info/control: -v (version), -i (device info), -s <ts> (set time)",
        .hint    = "-v | -i | -s <unix_ts>",
        .func    = &usb_cli_cmd_system,
    };
    (void)esp_console_cmd_register(&cmd_system);

    const esp_console_cmd_t cmd_reboot = {
        .command = "reboot",
        .help    = "Reboot the device",
        .hint    = NULL,
        .func    = &usb_cli_cmd_reboot,
    };
    (void)esp_console_cmd_register(&cmd_reboot);

    const esp_console_cmd_t cmd_speedtest = {
        .command = "speedtest",
        .help    = "LTE throughput test via Cloudflare. Flags: -d download only, -u upload only, -b <bytes>",
        .hint    = "[-d|-u] [-b bytes]",
        .func    = &usb_cli_cmd_speedtest,
    };
    (void)esp_console_cmd_register(&cmd_speedtest);

    const esp_console_cmd_t cmd_ping = {
        .command = "ping",
        .help    = "ICMP ping. Usage: ping [ip] [-c count] [-i interval_ms]. Default: 8.8.8.8 x4",
        .hint    = "[ip] [-c n] [-i ms]",
        .func    = &usb_cli_cmd_ping,
    };
    (void)esp_console_cmd_register(&cmd_ping);

    const esp_console_cmd_t cmd_sleep = {
        .command = "sleep",
        .help    = "Enter low-power mode. 1=USB active, 2=deep sleep (timer/reset), 3=deep sleep (GPIO wake)",
        .hint    = "<1|2|3> [seconds]",
        .func    = &usb_cli_cmd_sleep,
    };
    (void)esp_console_cmd_register(&cmd_sleep);

    const esp_console_cmd_t cmd_wake = {
        .command = "wake",
        .help    = "Wake from sleep mode 1 (reboots to re-init peripherals)",
        .hint    = NULL,
        .func    = &usb_cli_cmd_wake,
    };
    (void)esp_console_cmd_register(&cmd_wake);
}
