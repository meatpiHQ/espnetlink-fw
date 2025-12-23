#include "usb_dev_bridge.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
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

#include "esp_console.h"

static const char *TAG = "usb_dev_bridge";

// CDC port mapping
#define CDC_PORT_LTE TINYUSB_CDC_ACM_0
#define CDC_PORT_CONSOLE TINYUSB_CDC_ACM_1

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

    ESP_LOGI(TAG, "USB console ready on CDC1 (type 'help')");

    char line[256];
    size_t idx = 0;

    printf("espnetlink> ");
    fflush(stdout);

    while (1) {
        uint8_t buf[64];
        size_t rx_size = 0;

        esp_err_t r = tinyusb_cdcacm_read(CDC_PORT_CONSOLE, buf, sizeof(buf), &rx_size);
        if (r != ESP_OK || rx_size == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        for (size_t i = 0; i < rx_size; i++) {
            const char c = (char)buf[i];

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
                printf("espnetlink> ");
                fflush(stdout);
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
    ESP_LOGI(TAG, "Starting USB_DEV_MODE: CDC0=LTE passthrough, CDC1=console");

    // UARTs
    lte_keep_awake_init();

    // Ensure LTE modem is powered ON and configured (baud + HW flow control)
    esp_err_t lte_err = lte_modem_init_if_needed();
    if (lte_err != ESP_OK) {
        ESP_LOGW(TAG, "LTE modem init failed (%s). Continuing with passthrough.", esp_err_to_name(lte_err));
    }

    ESP_RETURN_ON_ERROR(setup_uart_lte(), TAG, "LTE UART setup failed");

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

    // Start interactive console on CDC1
    xTaskCreate(usb_console_task, "usb_console", 4096, NULL, 4, NULL);

    xTaskCreate(uart_to_usb_task, "uart_to_usb", 4096, NULL, 5, NULL);

    return ESP_OK;
}
