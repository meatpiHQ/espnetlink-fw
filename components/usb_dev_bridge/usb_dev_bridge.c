#include "usb_dev_bridge.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_console.h"

#include "hw_config.h"
#include "modem_manager.h"

#include <ctype.h>
#include <stdio.h>
#include <stdbool.h>

#include "esp_console.h"
#include "esp_idf_version.h"

#include <nmea.h>
#include <gpgga.h>
#include <gprmc.h>

static const char *TAG = "usb_dev_bridge";

// CDC port mapping
#define CDC_PORT_LTE TINYUSB_CDC_ACM_0
#define CDC_PORT_CONSOLE TINYUSB_CDC_ACM_1

typedef enum {
    CDC1_MODE_GPS_STREAM = 0,
    CDC1_MODE_CONSOLE = 1,
} cdc1_mode_t;

static volatile cdc1_mode_t s_cdc1_mode = CDC1_MODE_GPS_STREAM;
static volatile TickType_t s_cdc1_pause_until = 0;

typedef struct {
    bool has_fix;
    bool valid;
    double lat;
    double lon;
    int satellites;
    int fix_quality;
    double altitude_m;
    double speed_knots;
    double course_deg;
    uint32_t last_update_ms;
} gps_fix_t;

static gps_fix_t s_gps_fix;
static SemaphoreHandle_t s_gps_fix_lock;

typedef struct {
    char sentence[128];
    size_t len;
} gps_sentence_t;

static QueueHandle_t s_gps_sentence_q;

static double nmea_coord_to_decimal(int degrees, double minutes, char cardinal)
{
    double v = (double)degrees + (minutes / 60.0);
    if (cardinal == 'S' || cardinal == 'W') {
        v = -v;
    }
    return v;
}

static void gps_fix_update_from_gpgga(const nmea_gpgga_s *gpgga)
{
    if (!gpgga) {
        return;
    }
    if (xSemaphoreTake(s_gps_fix_lock, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }
    s_gps_fix.has_fix = true;
    s_gps_fix.fix_quality = (int)gpgga->position_fix;
    s_gps_fix.satellites = gpgga->n_satellites;
    s_gps_fix.altitude_m = (double)gpgga->altitude;
    s_gps_fix.lat = nmea_coord_to_decimal(gpgga->latitude.degrees, gpgga->latitude.minutes, (char)gpgga->latitude.cardinal);
    s_gps_fix.lon = nmea_coord_to_decimal(gpgga->longitude.degrees, gpgga->longitude.minutes, (char)gpgga->longitude.cardinal);
    s_gps_fix.last_update_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    xSemaphoreGive(s_gps_fix_lock);
}

static void gps_fix_update_from_gprmc(const nmea_gprmc_s *gprmc)
{
    if (!gprmc) {
        return;
    }
    if (xSemaphoreTake(s_gps_fix_lock, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }
    s_gps_fix.has_fix = true;
    s_gps_fix.valid = gprmc->valid;
    s_gps_fix.lat = nmea_coord_to_decimal(gprmc->latitude.degrees, gprmc->latitude.minutes, (char)gprmc->latitude.cardinal);
    s_gps_fix.lon = nmea_coord_to_decimal(gprmc->longitude.degrees, gprmc->longitude.minutes, (char)gprmc->longitude.cardinal);
    s_gps_fix.speed_knots = gprmc->gndspd_knots;
    s_gps_fix.course_deg = gprmc->track_deg;
    s_gps_fix.last_update_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    xSemaphoreGive(s_gps_fix_lock);
}

static void gps_parser_task(void *arg)
{
    (void)arg;

    gps_sentence_t msg;
    while (1) {
        if (xQueueReceive(s_gps_sentence_q, &msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (msg.len == 0) {
            continue;
        }

        nmea_s *data = nmea_parse(msg.sentence, msg.len, 0);
        if (!data) {
            continue;
        }

        if (data->type == NMEA_GPGGA) {
            gps_fix_update_from_gpgga((const nmea_gpgga_s *)data);
        } else if (data->type == NMEA_GPRMC) {
            gps_fix_update_from_gprmc((const nmea_gprmc_s *)data);
        }

        nmea_free(data);
    }
}

// Escape detection state for GPS passthrough mode
static uint8_t s_escape_buf[32];
static size_t s_escape_len = 0;
static TickType_t s_escape_last_tick = 0;

static void gps_escape_flush_to_uart(void)
{
    if (s_escape_len == 0) {
        return;
    }
    (void)uart_write_bytes(GPS_UART_PORT, (const char *)s_escape_buf, (int)s_escape_len);
    s_escape_len = 0;
}

static void gps_escape_reset(void)
{
    s_escape_len = 0;
    s_escape_last_tick = 0;
}

static void set_cdc1_mode(cdc1_mode_t mode)
{
    s_cdc1_mode = mode;
}

static void pause_cdc1_output_ms(uint32_t ms)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t until = now + pdMS_TO_TICKS(ms);
    if (until > s_cdc1_pause_until) {
        s_cdc1_pause_until = until;
    }
}

static int cmd_gps(int argc, char **argv)
{
    bool opt_stream = false;
    bool opt_print = false;
    bool opt_json = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-s") == 0) {
            opt_stream = true;
        } else if (strcmp(argv[i], "-p") == 0) {
            opt_print = true;
        } else if (strcmp(argv[i], "-j") == 0) {
            opt_json = true;
        } else {
            printf("Usage: gps -p [-j] | gps -s\n");
            return 1;
        }
    }

    if (opt_stream) {
        set_cdc1_mode(CDC1_MODE_GPS_STREAM);
        printf("Returning to GPS stream mode. Send +++++++ (or =======) + Enter to enter console.\n");
        fflush(stdout);
        return 0;
    }

    if (!opt_print) {
        printf("Usage: gps -p [-j] | gps -s\n");
        return 1;
    }

    gps_fix_t snap = {0};
    if (xSemaphoreTake(s_gps_fix_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        snap = s_gps_fix;
        xSemaphoreGive(s_gps_fix_lock);
    }

    if (!snap.has_fix) {
        printf("No GPS fix parsed yet.\n");
        return 0;
    }

    if (opt_json) {
        printf(
            "{\"valid\":%s,\"lat\":%.7f,\"lon\":%.7f,\"satellites\":%d,\"fix_quality\":%d,\"altitude_m\":%.1f,\"speed_knots\":%.2f,\"course_deg\":%.2f,\"last_update_ms\":%u}\n",
            snap.valid ? "true" : "false",
            snap.lat,
            snap.lon,
            snap.satellites,
            snap.fix_quality,
            snap.altitude_m,
            snap.speed_knots,
            snap.course_deg,
            (unsigned)snap.last_update_ms);
    } else {
        printf("valid=%d lat=%.7f lon=%.7f sat=%d fixq=%d alt=%.1fm spd=%.2fkn crs=%.2fdeg age_ms=%u\n",
               snap.valid,
               snap.lat,
               snap.lon,
               snap.satellites,
               snap.fix_quality,
               snap.altitude_m,
               snap.speed_knots,
               snap.course_deg,
               (unsigned)((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - snap.last_update_ms));
    }

    return 0;
}

static void register_console_commands(void)
{
    const esp_console_cmd_t cmd = {
        .command = "gps",
        .help = "GPS helpers: gps -p [-j] prints last parsed fix (optional JSON), gps -s returns to GPS stream mode",
        .hint = NULL,
        .func = &cmd_gps,
    };
    (void)esp_console_cmd_register(&cmd);
}

static esp_err_t setup_uart_gps(void)
{
    // Power on GPS module (best-effort)
    if (GPS_PWR_EN_PIN >= 0) {
        gpio_reset_pin(GPS_PWR_EN_PIN);
        gpio_set_direction(GPS_PWR_EN_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(GPS_PWR_EN_PIN, 1);
    }

    uart_config_t cfg = {
        .baud_rate = GPS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(GPS_UART_PORT, 8192, 0, 0, NULL, 0), TAG, "GPS uart_driver_install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(GPS_UART_PORT, &cfg), TAG, "GPS uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(GPS_UART_PORT, GPS_UART_TX_PIN, GPS_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "GPS uart_set_pin failed");
    return ESP_OK;
}

static void gps_to_usb_task(void *arg)
{
    (void)arg;

    uint8_t buf[256];
    gps_sentence_t sentence = {0};
    size_t sidx = 0;

    while (1) {
        int n = uart_read_bytes(GPS_UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(50));
        if (n <= 0) {
            continue;
        }

        // Stream to CDC1 only in GPS stream mode (and not during the "typing pause")
        if (s_cdc1_mode == CDC1_MODE_GPS_STREAM && xTaskGetTickCount() >= s_cdc1_pause_until) {
            tinyusb_cdcacm_write_queue(CDC_PORT_CONSOLE, buf, (size_t)n);
            tinyusb_cdcacm_write_flush(CDC_PORT_CONSOLE, 0);
        }

        // In console mode, parse GPS sentences in a separate task.
        if (s_cdc1_mode == CDC1_MODE_CONSOLE) {
            for (int i = 0; i < n; i++) {
                char c = (char)buf[i];
                if (sidx + 1 < sizeof(sentence.sentence)) {
                    sentence.sentence[sidx++] = c;
                }
                if (c == '\n') {
                    sentence.sentence[sidx] = '\0';
                    sentence.len = sidx;
                    (void)xQueueSend(s_gps_sentence_q, &sentence, 0);
                    sidx = 0;
                }
            }
        }
    }
}

static void usb_console_task(void *arg)
{
    (void)arg;

    // NOTE:
    // tinyusb_console_init() registers a VFS with O_NONBLOCK reads.
    // This breaks linenoise/stdio-style blocking input and causes prompt spam.
    // So we read input directly from the CDC-ACM driver (CDC1) and only use
    // tinyusb_console for output redirection (stdout/stderr).

    const esp_console_config_t console_cfg = {
        .max_cmdline_length = 256,
        .max_cmdline_args = 8,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .hint_color = 0,
#endif
    };

    ESP_ERROR_CHECK(esp_console_init(&console_cfg));
    esp_console_register_help_command();
    register_console_commands();

    ESP_LOGI(TAG, "CDC1: GPS passthrough by default; send +++++++ (or =======) + Enter to enter console");

    char line[256];
    size_t idx = 0;

    while (1) {
        uint8_t buf[64];
        size_t rx_size = 0;

        esp_err_t r = tinyusb_cdcacm_read(CDC_PORT_CONSOLE, buf, sizeof(buf), &rx_size);
        if (r != ESP_OK || rx_size == 0) {
            // If user typed some '+' but never finished the escape, don't hold them forever.
            if (s_cdc1_mode == CDC1_MODE_GPS_STREAM && s_escape_len > 0 && s_escape_last_tick != 0) {
                TickType_t now = xTaskGetTickCount();
                if ((now - s_escape_last_tick) > pdMS_TO_TICKS(500)) {
                    gps_escape_flush_to_uart();
                    gps_escape_reset();
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        for (size_t i = 0; i < rx_size; i++) {
            const char c = (char)buf[i];

            // Mode 0: GPS stream mode. Only watch for escape sequence from host.
            if (s_cdc1_mode == CDC1_MODE_GPS_STREAM) {
                // In GPS passthrough mode we forward CDC1->GPS UART,
                // but reserve the escape sequence for entering console.
                // Escape: 7+ '+' (or '=') then Enter (CR/LF).

                // Pause GPS output when host is typing to avoid terminal flood.
                pause_cdc1_output_ms(1500);

                // Collect potential escape bytes
                if (c == '+' || c == '=') {
                    if (s_escape_len < sizeof(s_escape_buf)) {
                        s_escape_buf[s_escape_len++] = (uint8_t)c;
                        s_escape_last_tick = xTaskGetTickCount();
                        continue;
                    }
                    // Buffer overflow: flush what we have and continue passthrough
                    gps_escape_flush_to_uart();
                    gps_escape_reset();
                    (void)uart_write_bytes(GPS_UART_PORT, &c, 1);
                    continue;
                }

                if (c == '\r' || c == '\n') {
                    if (s_escape_len >= 7) {
                        // Consume escape + Enter and enter console
                        gps_escape_reset();
                        pause_cdc1_output_ms(0);
                        set_cdc1_mode(CDC1_MODE_CONSOLE);
                        idx = 0;
                        printf("\n[console mode] type 'help' (or 'gps' to return)\n");
                        printf("espnetlink> ");
                        fflush(stdout);
                        continue;
                    }
                    // Not an escape: flush buffered '+'/'=' then forward newline to GPS
                    gps_escape_flush_to_uart();
                    gps_escape_reset();
                    (void)uart_write_bytes(GPS_UART_PORT, &c, 1);
                    continue;
                }

                // Any other byte: flush buffered '+'/'=' then forward byte to GPS
                gps_escape_flush_to_uart();
                gps_escape_reset();
                (void)uart_write_bytes(GPS_UART_PORT, &c, 1);
                continue;
            }

            // Mode 1: Console mode

            // Handle backspace (BS or DEL)
            if (c == '\b' || c == 0x7F) {
                if (idx > 0) {
                    idx--;
                    // erase char on terminal: \b \b
                    tinyusb_cdcacm_write_queue(CDC_PORT_CONSOLE, (uint8_t *)"\b \b", 3);
                    tinyusb_cdcacm_write_flush(CDC_PORT_CONSOLE, 0);
                }
                continue;
            }

            // Ignore NUL
            if (c == '\0') {
                continue;
            }

            // CR or LF ends the line (treat CRLF as single end)
            if (c == '\r' || c == '\n') {
                // If we got CR, swallow a following LF in this chunk
                if (c == '\r' && (i + 1) < rx_size && buf[i + 1] == '\n') {
                    i++;
                }

                // Echo newline
                tinyusb_cdcacm_write_queue(CDC_PORT_CONSOLE, (uint8_t *)"\r\n", 2);
                tinyusb_cdcacm_write_flush(CDC_PORT_CONSOLE, 0);

                line[idx] = '\0';

                // Trim whitespace
                char *start = line;
                while (*start && isspace((unsigned char)*start)) {
                    start++;
                }
                char *end = start + strlen(start);
                while (end > start && isspace((unsigned char)end[-1])) {
                    end[-1] = '\0';
                    end--;
                }

                if (*start) {
                    int ret = 0;
                    esp_err_t err = esp_console_run(start, &ret);
                    if (err == ESP_ERR_NOT_FOUND) {
                        printf("Unrecognized command\n");
                    } else if (err == ESP_ERR_INVALID_ARG) {
                        printf("Invalid arguments\n");
                    } else if (err != ESP_OK) {
                        printf("Command error: %s\n", esp_err_to_name(err));
                    }
                }

                idx = 0;
                if (s_cdc1_mode == CDC1_MODE_CONSOLE) {
                    printf("espnetlink> ");
                    fflush(stdout);
                }
                continue;
            }

            // Regular character
            if (idx + 1 < sizeof(line)) {
                line[idx++] = c;
                // Echo character
                tinyusb_cdcacm_write_queue(CDC_PORT_CONSOLE, (const uint8_t *)&c, 1);
                tinyusb_cdcacm_write_flush(CDC_PORT_CONSOLE, 0);
            }
        }
    }
}

static void lte_keep_awake_init(void)
{
    // Keep BG95 awake / prevent sleep (best-effort)
    if (LTE_DTR_PIN >= 0) {
        gpio_reset_pin(LTE_DTR_PIN);
        gpio_set_direction(LTE_DTR_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(LTE_DTR_PIN, 1);
    }
}

static esp_err_t lte_modem_init_if_needed(void)
{
    // If you want pure passthrough without any modem init, you can bypass this call.
    const modem_mgr_config_t modem_cfg = {
        .uart_port = LTE_UART_PORT,
        .tx_pin = LTE_UART_TX_PIN,
        .rx_pin = LTE_UART_RX_PIN,
        .rts_pin = LTE_RTS_PIN,
        .cts_pin = LTE_CTS_PIN,
        .pwr_key_pin = LTE_PWR_KEY_PIN,
        .status_pin = LTE_STATUS_PIN,
        .dtr_pin = LTE_DTR_PIN,
        .status_on_is_low = true, // BG95: STATUS low when ON
    };

    return modem_mgr_init(&modem_cfg);
}

static esp_err_t setup_uart_lte(void)
{
    uart_config_t cfg = {
        .baud_rate = LTE_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(LTE_UART_PORT, 8192, 8192, 0, NULL, 0), TAG, "LTE uart_driver_install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(LTE_UART_PORT, &cfg), TAG, "LTE uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(LTE_UART_PORT, LTE_UART_TX_PIN, LTE_UART_RX_PIN, LTE_RTS_PIN, LTE_CTS_PIN), TAG, "LTE uart_set_pin failed");
    ESP_RETURN_ON_ERROR(uart_set_hw_flow_ctrl(LTE_UART_PORT, UART_HW_FLOWCTRL_CTS_RTS, 64), TAG, "LTE uart_set_hw_flow_ctrl failed");

    return ESP_OK;
}

static void cdc_rx_to_uart_cb(int itf, cdcacm_event_t *event)
{
    (void)event;

    if (itf != CDC_PORT_LTE) {
        // CDC1 is used for console, not passthrough
        return;
    }

    uint8_t buf[512];
    size_t rx_size = 0;

    if (tinyusb_cdcacm_read(itf, buf, sizeof(buf), &rx_size) != ESP_OK || rx_size == 0) {
        return;
    }

    (void)uart_write_bytes(LTE_UART_PORT, (const char *)buf, (int)rx_size);
}

static void uart_to_usb_task(void *arg)
{
    (void)arg;

    uint8_t buf[512];
    ESP_LOGI(TAG, "UART->USB task started");

    while (1) {
        // LTE UART -> CDC0
        {
            size_t avail = 0;
            (void)uart_get_buffered_data_len(LTE_UART_PORT, &avail);
            if (avail) {
                size_t to_read = (avail > sizeof(buf)) ? sizeof(buf) : avail;
                int n = uart_read_bytes(LTE_UART_PORT, buf, to_read, 0);
                if (n > 0) {
                    tinyusb_cdcacm_write_queue(CDC_PORT_LTE, buf, (size_t)n);
                    tinyusb_cdcacm_write_flush(CDC_PORT_LTE, 0);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

esp_err_t usb_dev_bridge_start(void)
{
    ESP_LOGI(TAG, "Starting USB_DEV_MODE: CDC0=LTE passthrough, CDC1=GPS stream (+ escape to console)");

    // UARTs
    lte_keep_awake_init();

    // Ensure LTE modem is powered ON and configured (baud + HW flow control)
    esp_err_t lte_err = lte_modem_init_if_needed();
    if (lte_err != ESP_OK) {
        ESP_LOGW(TAG, "LTE modem init failed (%s). Continuing with passthrough.", esp_err_to_name(lte_err));
    }

    ESP_RETURN_ON_ERROR(setup_uart_lte(), TAG, "LTE UART setup failed");
    ESP_RETURN_ON_ERROR(setup_uart_gps(), TAG, "GPS UART setup failed");

    // TinyUSB device stack (esp_tinyusb manages PHY + tud_task internally)
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    ESP_RETURN_ON_ERROR(tinyusb_driver_install(&tusb_cfg), TAG, "tinyusb_driver_install failed");

    tinyusb_config_cdcacm_t cdc_cfg = {
        .cdc_port = CDC_PORT_LTE,
        .callback_rx = &cdc_rx_to_uart_cb,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL,
    };
    ESP_RETURN_ON_ERROR(tinyusb_cdcacm_init(&cdc_cfg), TAG, "CDC0 init failed");

    // CDC1: console (no passthrough callbacks)
    cdc_cfg.cdc_port = CDC_PORT_CONSOLE;
    cdc_cfg.callback_rx = NULL;
    ESP_RETURN_ON_ERROR(tinyusb_cdcacm_init(&cdc_cfg), TAG, "CDC1 init failed");

    // Redirect stdin/stdout/stderr (and therefore printf/logs) to CDC1
    ESP_RETURN_ON_ERROR(tinyusb_console_init(CDC_PORT_CONSOLE), TAG, "tinyusb_console_init failed");

    if (!s_gps_fix_lock) {
        s_gps_fix_lock = xSemaphoreCreateMutex();
    }
    if (!s_gps_sentence_q) {
        s_gps_sentence_q = xQueueCreate(10, sizeof(gps_sentence_t));
    }
    if (!s_gps_fix_lock || !s_gps_sentence_q) {
        return ESP_ERR_NO_MEM;
    }

    xTaskCreate(gps_parser_task, "gps_parser", 4096, NULL, 4, NULL);

    // Start GPS streaming task (only active in GPS stream mode)
    xTaskCreate(gps_to_usb_task, "gps_to_usb", 4096, NULL, 4, NULL);

    // Start interactive console on CDC1
    xTaskCreate(usb_console_task, "usb_console", 4096, NULL, 4, NULL);

    xTaskCreate(uart_to_usb_task, "uart_to_usb", 4096, NULL, 5, NULL);

    return ESP_OK;
}
