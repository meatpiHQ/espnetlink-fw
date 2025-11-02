#include "usb_cdc_uart_bridge.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "usb_cdc_host_manager.h"
#include "esp_check.h"
#include "esp_heap_caps.h"

static const char *TAG_BR = "usb_bridge";

// Context for a single UART<->USB bridge
typedef struct {
    uart_port_t uart_port;
    int uart_tx_pin;
    int uart_rx_pin;
    int uart_rts_pin;
    int uart_cts_pin;
    bool use_hw_flow;
    int uart_baud;
    int uart_data_bits;
    int uart_parity;   // 0=None,1=Odd,2=Even
    int uart_stop_bits;// 1,2

    uint16_t usb_vid;
    uint16_t usb_pid;
    uint8_t usb_iface; // 0 or 2
    int usb_baud;
    int usb_data_bits;
    int usb_parity;    // 0..4
    int usb_stop_bits; // 0..2
    int usb_conn_timeout_ms;
    size_t usb_out_buf;
    size_t usb_in_buf;
    uint32_t usb_tx_timeout_ms;

    usb_cdc_event_cb_t user_event_cb;
    usb_cdc_rx_cb_t user_rx_tap_cb;
    void *user_ctx;

    TaskHandle_t uart_rx_task;
    TaskHandle_t usb_to_uart_task;
    StreamBufferHandle_t usb_to_uart_sb;
    volatile bool running;
} bridge_ctx_t;

static bridge_ctx_t s_ctx = {};
//
static bool usb_rx_to_uart_cb(const uint8_t *data, size_t data_len, void *arg)
{
    bridge_ctx_t *ctx = (bridge_ctx_t *)arg;
    if (!ctx->running) return true;

    bool cont = true;
    if (ctx->user_rx_tap_cb) {
        cont = ctx->user_rx_tap_cb(data, data_len, ctx->user_ctx);
    }
    if (!cont || data_len == 0) return cont;

    // Enqueue USB RX data into a stream buffer for a dedicated UART TX task
    if (ctx->usb_to_uart_sb) {
        size_t sent = xStreamBufferSend(ctx->usb_to_uart_sb, data, data_len, 0);
        if (sent < data_len) {
            ESP_LOGW(TAG_BR, "USB->UART stream buffer full: dropped %u/%u bytes", (unsigned)(data_len - sent), (unsigned)data_len);
        }
        ESP_LOGD(TAG_BR, "USB->UART: received %u bytes, queued %u bytes", (unsigned)data_len, (unsigned)sent);
        ESP_LOG_BUFFER_HEXDUMP(TAG_BR, data, data_len, ESP_LOG_DEBUG);
    } else {
        // Fallback: write directly (may block USB task)
        int written = uart_write_bytes(ctx->uart_port, (const char *)data, data_len);
        if (written < 0) {
            ESP_LOGW(TAG_BR, "UART write failed: %d", written);
        }
    }

    return true; // continue receiving
}

static void usb_event_cb(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    bridge_ctx_t *ctx = (bridge_ctx_t *)user_ctx;
    if (ctx->user_event_cb) {
        ctx->user_event_cb(event, ctx->user_ctx);
    }
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG_BR, "CDC error %d", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG_BR, "USB CDC device disconnected");
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        {
            uint16_t v = event->data.serial_state.val;
            // CDC ACM spec defines the following low 7 bits:
            // bit0 RX_CARRIER (DCD), bit1 TX_CARRIER (DSR), bit2 BREAK,
            // bit3 RING_SIGNAL, bit4 FRAMING, bit5 PARITY, bit6 OVERRUN
            bool rx_carrier = (v & (1u << 0)) != 0;
            bool tx_carrier = (v & (1u << 1)) != 0;
            bool brk        = (v & (1u << 2)) != 0;
            bool ring       = (v & (1u << 3)) != 0;
            bool framing    = (v & (1u << 4)) != 0;
            bool parity     = (v & (1u << 5)) != 0;
            bool overrun    = (v & (1u << 6)) != 0;
            ESP_LOGI(TAG_BR,
                     "CDC serial state 0x%04X [DCD=%d, DSR=%d, BREAK=%d, RING=%d, FRAMING_ERR=%d, PARITY_ERR=%d, OVERRUN=%d]",
                     v, rx_carrier, tx_carrier, brk, ring, framing, parity, overrun);
        }
        break;
    default:
        break;
    }
}

static void uart_to_usb_task(void *arg)
{
    bridge_ctx_t *ctx = (bridge_ctx_t *)arg;
    // Coalesce UART reads to reduce USB traffic and CPU usage
    const size_t BUF_SZ = (ctx->usb_out_buf && ctx->usb_out_buf <= 8192) ? ctx->usb_out_buf : (size_t)4096;
    uint8_t *buf = (uint8_t *)heap_caps_malloc(BUF_SZ, MALLOC_CAP_DEFAULT);
    if (!buf) {
        ESP_LOGE(TAG_BR, "Failed to allocate UART RX buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG_BR, "UART->USB task started");
    while (ctx->running) {
        // Try to read as much as is currently buffered, with a short timeout to allow coalescing
        size_t avail = 0;
        (void)uart_get_buffered_data_len(ctx->uart_port, &avail);
        size_t to_read = avail > 0 ? (avail > BUF_SZ ? BUF_SZ : avail) : BUF_SZ;
        int n = uart_read_bytes(ctx->uart_port, buf, to_read, pdMS_TO_TICKS(10));
        if (n > 0) {
            esp_err_t err = usb_cdc_mgr_tx(ctx->usb_iface, buf, n, ctx->usb_tx_timeout_ms);
            ESP_LOGD(TAG_BR, "UART->USB: read %d bytes, sent %d bytes", n, (err == ESP_OK) ? n : 0);
            ESP_LOG_BUFFER_HEXDUMP(TAG_BR, buf, n, ESP_LOG_DEBUG);
            if (err != ESP_OK) {
                // On failure (e.g., disconnect), backoff a bit
                ESP_LOGW(TAG_BR, "USB TX failed: %d", (int)err);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        } else {
            // Nothing to read now; yield briefly
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }

    heap_caps_free(buf);
    vTaskDelete(NULL);
}

static void usb_to_uart_task(void *arg)
{
    bridge_ctx_t *ctx = (bridge_ctx_t *)arg;
    const size_t BUF_SZ = (ctx->usb_in_buf && ctx->usb_in_buf <= 8192) ? ctx->usb_in_buf : (size_t)4096;
    uint8_t *buf = (uint8_t *)heap_caps_malloc(BUF_SZ, MALLOC_CAP_DEFAULT);
    if (!buf) {
        ESP_LOGE(TAG_BR, "Failed to allocate USB->UART buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG_BR, "USB->UART task started");
    while (ctx->running) {
        size_t n = xStreamBufferReceive(ctx->usb_to_uart_sb, buf, BUF_SZ, pdMS_TO_TICKS(100));
        if (n > 0) {
            size_t offset = 0;
            while (offset < n && ctx->running) {
                int written = uart_write_bytes(ctx->uart_port, (const char *)(buf + offset), n - offset);
                if (written > 0) {
                    offset += (size_t)written;
                } else if (written == 0) {
                    // Give UART ISR a chance to drain
                    vTaskDelay(pdMS_TO_TICKS(1));
                } else {
                    ESP_LOGW(TAG_BR, "UART write failed: %d", written);
                    break;
                }
            }
            ESP_LOGD(TAG_BR, "USB->UART: wrote %u bytes", (unsigned)offset);
            ESP_LOG_BUFFER_HEXDUMP(TAG_BR, buf, offset, ESP_LOG_DEBUG);
        }
    }

    heap_caps_free(buf);
    vTaskDelete(NULL);
}

static esp_err_t setup_uart(bridge_ctx_t *ctx)
{
    uart_config_t cfg = {};
    cfg.baud_rate = ctx->uart_baud;
    switch (ctx->uart_data_bits) {
    case 5: cfg.data_bits = UART_DATA_5_BITS; break;
    case 6: cfg.data_bits = UART_DATA_6_BITS; break;
    case 7: cfg.data_bits = UART_DATA_7_BITS; break;
    default: cfg.data_bits = UART_DATA_8_BITS; break;
    }
    switch (ctx->uart_parity) {
    case 1: cfg.parity = UART_PARITY_ODD; break;
    case 2: cfg.parity = UART_PARITY_EVEN; break;
    default: cfg.parity = UART_PARITY_DISABLE; break;
    }
    cfg.stop_bits = (ctx->uart_stop_bits == 2) ? UART_STOP_BITS_2 : UART_STOP_BITS_1;
    cfg.flow_ctrl = ctx->use_hw_flow ? UART_HW_FLOWCTRL_CTS_RTS : UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    ESP_RETURN_ON_ERROR(uart_driver_install(ctx->uart_port, 4096, 4096, 0, NULL, 0), TAG_BR, "uart_driver_install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(ctx->uart_port, &cfg), TAG_BR, "uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(ctx->uart_port,
                                     ctx->uart_tx_pin,
                                     ctx->uart_rx_pin,
                                     ctx->use_hw_flow ? ctx->uart_rts_pin : UART_PIN_NO_CHANGE,
                                     ctx->use_hw_flow ? ctx->uart_cts_pin : UART_PIN_NO_CHANGE),
                        TAG_BR, "uart_set_pin failed");
    // Ensure HW flow control threshold is set when enabled
    if (ctx->use_hw_flow) {
        (void)uart_set_hw_flow_ctrl(ctx->uart_port, UART_HW_FLOWCTRL_CTS_RTS, 64);
    } else {
        (void)uart_set_hw_flow_ctrl(ctx->uart_port, UART_HW_FLOWCTRL_DISABLE, 0);
    }
    // Best-effort flush to start with a clean RX buffer
    uart_flush_input(ctx->uart_port);
    // Enable RX pull-up to avoid floating input when line is idle
    (void)gpio_pullup_en((gpio_num_t)ctx->uart_rx_pin);
    return ESP_OK;
}

esp_err_t usb_cdc_uart_bridge_start(const usb_cdc_uart_bridge_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (s_ctx.running) {
        ESP_LOGW(TAG_BR, "Bridge already running");
        return ESP_OK;
    }

    // Copy config into context
    s_ctx.uart_port = (uart_port_t)cfg->uart_port;
    s_ctx.uart_tx_pin = cfg->uart_tx_pin;
    s_ctx.uart_rx_pin = cfg->uart_rx_pin;
    s_ctx.use_hw_flow = cfg->use_hw_flow;
    s_ctx.uart_rts_pin = cfg->use_hw_flow ? cfg->uart_rts_pin : UART_PIN_NO_CHANGE;
    s_ctx.uart_cts_pin = cfg->use_hw_flow ? cfg->uart_cts_pin : UART_PIN_NO_CHANGE;
    s_ctx.uart_baud = cfg->uart_baud;
    s_ctx.uart_data_bits = cfg->uart_data_bits;
    s_ctx.uart_parity = cfg->uart_parity;
    s_ctx.uart_stop_bits = cfg->uart_stop_bits;

    s_ctx.usb_vid = cfg->usb_vid;
    s_ctx.usb_pid = cfg->usb_pid;
    s_ctx.usb_iface = cfg->usb_interface;
    s_ctx.usb_baud = cfg->usb_baud;
    s_ctx.usb_data_bits = cfg->usb_data_bits;
    s_ctx.usb_parity = cfg->usb_parity;
    s_ctx.usb_stop_bits = cfg->usb_stop_bits;
    s_ctx.usb_conn_timeout_ms = cfg->usb_conn_timeout_ms;
    s_ctx.usb_out_buf = cfg->usb_out_buffer;
    s_ctx.usb_in_buf = cfg->usb_in_buffer;
    s_ctx.usb_tx_timeout_ms = cfg->usb_tx_timeout_ms;

    s_ctx.user_event_cb = cfg->event_cb;
    s_ctx.user_rx_tap_cb = cfg->rx_tap_cb;
    s_ctx.user_ctx = cfg->user_ctx;

    ESP_RETURN_ON_ERROR(usb_cdc_mgr_init(), TAG_BR, "usb_cdc_mgr_init failed");

    cdc_acm_host_device_config_t dev_cfg = {};
    dev_cfg.connection_timeout_ms = s_ctx.usb_conn_timeout_ms;
    dev_cfg.out_buffer_size = s_ctx.usb_out_buf;
    dev_cfg.in_buffer_size = s_ctx.usb_in_buf;
    dev_cfg.event_cb = usb_event_cb;
    dev_cfg.data_cb = usb_rx_to_uart_cb;
    dev_cfg.user_arg = &s_ctx;

    ESP_LOGI(TAG_BR, "Opening USB CDC VID=0x%04X PID=0x%04X iface=%u", s_ctx.usb_vid, s_ctx.usb_pid, s_ctx.usb_iface);
    ESP_RETURN_ON_ERROR(usb_cdc_mgr_open(s_ctx.usb_vid, s_ctx.usb_pid, s_ctx.usb_iface,
                                         s_ctx.usb_conn_timeout_ms,
                                         s_ctx.usb_out_buf,
                                         s_ctx.usb_in_buf,
                                         usb_event_cb,
                                         usb_rx_to_uart_cb,
                                         &s_ctx), TAG_BR, "usb_cdc_mgr_open failed");

    ESP_RETURN_ON_ERROR(usb_cdc_mgr_set_line_coding(s_ctx.usb_iface,
                                                    s_ctx.usb_baud,
                                                    s_ctx.usb_stop_bits,
                                                    s_ctx.usb_parity,
                                                    s_ctx.usb_data_bits),
                        TAG_BR, "set_line_coding failed");

    ESP_RETURN_ON_ERROR(setup_uart(&s_ctx), TAG_BR, "UART setup failed");

    s_ctx.running = true;
    // Create stream buffer for decoupling USB RX from UART TX path
    s_ctx.usb_to_uart_sb = xStreamBufferCreate(s_ctx.usb_in_buf ? s_ctx.usb_in_buf : 4096, 0);
    if (s_ctx.usb_to_uart_sb == NULL) {
        s_ctx.running = false;
        ESP_LOGE(TAG_BR, "Failed to create stream buffer");
        return ESP_FAIL;
    }

    if (xTaskCreate(uart_to_usb_task, "uart_to_usb", 4096, &s_ctx, 8, &s_ctx.uart_rx_task) != pdTRUE) {
        s_ctx.running = false;
        ESP_LOGE(TAG_BR, "Failed to create UART->USB task");
        return ESP_FAIL;
    }

    if (xTaskCreate(usb_to_uart_task, "usb_to_uart", 4096, &s_ctx, 8, &s_ctx.usb_to_uart_task) != pdTRUE) {
        s_ctx.running = false;
        ESP_LOGE(TAG_BR, "Failed to create USB->UART task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_BR, "UART<->USB bridge started (UART%d <-> iface %u)", (int)s_ctx.uart_port, (unsigned)s_ctx.usb_iface);
    return ESP_OK;
}

void usb_cdc_uart_bridge_stop(void)
{
    if (!s_ctx.running) return;
    s_ctx.running = false;
    if (s_ctx.uart_rx_task) {
        // Give task a moment to exit
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (s_ctx.usb_to_uart_task) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    uart_driver_delete(s_ctx.uart_port);
    // Close only the interface opened by this bridge
    (void)usb_cdc_mgr_close(s_ctx.usb_iface);
    if (s_ctx.usb_to_uart_sb) {
        vStreamBufferDelete(s_ctx.usb_to_uart_sb);
        s_ctx.usb_to_uart_sb = NULL;
    }
    ESP_LOGI(TAG_BR, "UART<->USB bridge stopped");
}
