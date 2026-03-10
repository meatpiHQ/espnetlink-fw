/*
 * lte_upstream_pppos_usb.cpp
 *
 * USB CDC-ACM DTE implementation for esp_modem.
 *
 * Creates an esp_modem DCE that communicates with the BG95 modem via its native
 * USB CDC-ACM port instead of UART.  The resulting DCE pointer is fully
 * compatible with every esp_modem_* C API function (esp_modem_at,
 * esp_modem_set_mode, etc.) because we build the same internal
 * esp_modem_dce_wrap structure that esp_modem_new() normally creates.
 *
 * Design
 * ------
 *  UsbTerminal  –  custom esp_modem::Terminal subclass
 *    - CDC-ACM RX callback: buffers received bytes in a FreeRTOS ByteBuf ring.
 *    - Dedicated reader task: drains the ring and calls the DTE on_read
 *      callback with real data (uart-terminal style, avoids VFS / select()).
 *    - write(): calls cdc_acm_host_data_tx_blocking() synchronously.
 *    - start() / stop() manage the reader task lifetime.
 *    - set_read_cb(): thread-safe (port mux protected).
 *
 * Threading
 * ---------
 *  Producer: CDC-ACM host task  →  xRingbufferSend()
 *  Consumer: reader task (ours) →  xRingbufferReceive() → on_read(data, len)
 *
 * The ring buffer decouples the two tasks which may run on different cores.
 */

#include <new>
#include <memory>
#include <functional>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"

/* esp_modem C pubic headers */
#include "esp_modem_config.h"
#include "esp_modem_c_api_types.h"

/* esp_modem C++ public headers */
#include "cxx_include/esp_modem_terminal.hpp"
#include "cxx_include/esp_modem_dte.hpp"
#include "cxx_include/esp_modem_api.hpp"
#include "cxx_include/esp_modem_dce_factory.hpp"

/* Private esp_modem header: esp_modem_dce_wrap struct (intended for extension
 * developers – see the comment at the top of c_api_wrapper.hpp). */
#include "esp_private/c_api_wrapper.hpp"

#include "lte_upstream_pppos_usb.h"

static const char *TAG = "lte_pppos_usb";

/* =========================================================================
 * USB host daemon task
 * Processes USB library events.  One instance is started the first time
 * the USB host library is installed from this module.
 * ========================================================================= */
static TaskHandle_t s_host_daemon_task = nullptr;

static void usb_host_daemon_task_fn(void * /*arg*/)
{
    ESP_LOGI(TAG, "USB host daemon started");
    while (true) {
        uint32_t event_flags = 0;
        esp_err_t ret = usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "usb_host_lib_handle_events: %s", esp_err_to_name(ret));
            break;
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB host: all devices freed");
        }
    }
    s_host_daemon_task = nullptr;
    vTaskDelete(nullptr);
}

/* =========================================================================
 * UsbTerminal – custom esp_modem::Terminal for CDC-ACM
 * ========================================================================= */
namespace {

class UsbTerminal final : public esp_modem::Terminal {
public:
    explicit UsbTerminal(size_t ring_size = 32768)
        : dev_hdl_(nullptr), ring_buf_(nullptr),
          reader_task_(nullptr), running_(false),
          cb_mux_(portMUX_INITIALIZER_UNLOCKED)
    {
        ring_buf_ = xRingbufferCreate(ring_size, RINGBUF_TYPE_BYTEBUF);
        if (!ring_buf_) {
            ESP_LOGE(TAG, "UsbTerminal: ring buffer alloc failed (%u bytes)",
                     static_cast<unsigned>(ring_size));
        }
    }

    ~UsbTerminal() override
    {
        stop();
        if (ring_buf_) {
            vRingbufferDelete(ring_buf_);
            ring_buf_ = nullptr;
        }
    }

    /* Terminal interface */
    void start() override
    {
        running_ = true;
        xTaskCreate(reader_task_fn, "usb_dce_rx", 4096, this,
                    /* priority */ 5, &reader_task_);
    }

    void stop() override
    {
        running_ = false;
        /* Wake the reader task quickly (send zero-length item – ring buf wakes
         * the blocked xRingbufferReceive immediately). */
        if (ring_buf_) {
            xRingbufferSend(ring_buf_, nullptr, 0, pdMS_TO_TICKS(10));
        }
        if (reader_task_) {
            vTaskDelay(pdMS_TO_TICKS(600)); /* wait up to 500 ms select timeout + margin */
            reader_task_ = nullptr;
        }
        if (dev_hdl_) {
            cdc_acm_host_close(dev_hdl_);
            dev_hdl_ = nullptr;
        }
    }

    int write(uint8_t *data, size_t len) override
    {
        if (!dev_hdl_) {
            return 0;
        }
        esp_err_t ret = cdc_acm_host_data_tx_blocking(dev_hdl_, data, len, 2000);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "TX failed: %s", esp_err_to_name(ret));
            return 0;
        }
        return static_cast<int>(len);
    }

    /* Not used in push mode (reader task delivers data directly via on_read). */
    int read(uint8_t *data, size_t len) override
    {
        (void)data;
        (void)len;
        return 0;
    }

    void set_read_cb(std::function<bool(uint8_t *data, size_t len)> f) override
    {
        portENTER_CRITICAL(&cb_mux_);
        on_read = std::move(f);
        portEXIT_CRITICAL(&cb_mux_);
    }

    /* Called from the CDC-ACM RX callback (USB host task context). */
    void push_rx(const uint8_t *data, size_t len)
    {
        if (!ring_buf_ || len == 0) {
            return;
        }
        if (xRingbufferSend(ring_buf_, data, len, pdMS_TO_TICKS(20)) != pdTRUE) {
            ESP_LOGW(TAG, "USB RX ring full – dropping %u bytes",
                     static_cast<unsigned>(len));
        }
    }

    /* Public so the factory function can assign the handle after open. */
    cdc_acm_dev_hdl_t dev_hdl_;

    /* Public wrapper for the protected Terminal::on_error so the static
     * C-style CDC-ACM callback can invoke it without friendship. */
    void notify_error(esp_modem::terminal_error err)
    {
        if (on_error) {
            on_error(err);
        }
    }

private:
    static void reader_task_fn(void *arg)
    {
        auto *self = static_cast<UsbTerminal *>(arg);
        while (self->running_) {
            size_t item_size = 0;
            void *item = xRingbufferReceive(self->ring_buf_, &item_size,
                                            pdMS_TO_TICKS(500));
            if (item && item_size > 0) {
                /* Capture the callback under the mux (brief critical section). */
                portENTER_CRITICAL(&self->cb_mux_);
                auto cb = self->on_read;
                portEXIT_CRITICAL(&self->cb_mux_);

                if (cb) {
                    cb(static_cast<uint8_t *>(item), item_size);
                }
                vRingbufferReturnItem(self->ring_buf_, item);
            }
        }
        vTaskDelete(nullptr);
    }

    RingbufHandle_t   ring_buf_;
    TaskHandle_t      reader_task_;
    volatile bool     running_;
    portMUX_TYPE      cb_mux_;
};

} /* anonymous namespace */

/* =========================================================================
 * CDC-ACM callbacks (static C-style, forward to UsbTerminal)
 * ========================================================================= */

/** CDC-ACM data received callback. */
static bool cdc_rx_cb(const uint8_t *data, size_t data_len, void *user_ctx)
{
    static_cast<UsbTerminal *>(user_ctx)->push_rx(data, data_len);
    return true; /* consumed – driver may re-use the buffer */
}

/** CDC-ACM device event callback. */
static void cdc_event_cb(const cdc_acm_host_dev_event_data_t *event,
                         void *user_ctx)
{
    auto *term = static_cast<UsbTerminal *>(user_ctx);
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC-ACM host error %d", event->data.error);
        term->notify_error(esp_modem::terminal_error::DEVICE_GONE);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "BG95 USB disconnected");
        term->notify_error(esp_modem::terminal_error::DEVICE_GONE);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        ESP_LOGD(TAG, "CDC-ACM serial state: 0x%04X",
                 event->data.serial_state.val);
        break;
    default:
        break;
    }
}

/* =========================================================================
 * Public factory (C linkage)
 * ========================================================================= */

extern "C" esp_modem_dce_t *lte_pppos_usb_create_dce(
    const esp_modem_dte_config_t *dte_config,
    const esp_modem_dce_config_t *dce_config,
    esp_netif_t                  *netif,
    int8_t                        usb_sel_gpio,
    bool                          usb_sel_level,
    uint16_t                      vid,
    uint16_t                      pid,
    uint8_t                       interface_num,
    uint32_t                      conn_timeout_ms)
{
    /* ------------------------------------------------------------------
     * 1.  Optionally drive a USB-MUX / path-select GPIO
     * ------------------------------------------------------------------ */
    if (usb_sel_gpio >= 0) {
        gpio_config_t io_cfg = {};
        io_cfg.pin_bit_mask = (1ULL << static_cast<uint8_t>(usb_sel_gpio));
        io_cfg.mode         = GPIO_MODE_OUTPUT;
        io_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
        io_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_cfg.intr_type    = GPIO_INTR_DISABLE;
        gpio_config(&io_cfg);
        // gpio_set_level(static_cast<gpio_num_t>(usb_sel_gpio),
        //                usb_sel_level ? 1 : 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "USB SEL GPIO%d -> %s",
                 usb_sel_gpio, usb_sel_level ? "HIGH" : "LOW");
    }

    /* ------------------------------------------------------------------
     * 2.  USB Host Library
     * ------------------------------------------------------------------ */
    usb_host_config_t host_cfg = {};
    host_cfg.skip_phy_setup = false;
    host_cfg.intr_flags     = ESP_INTR_FLAG_LEVEL1;
    esp_err_t ret = usb_host_install(&host_cfg);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "USB host library already installed – reusing");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_install: %s", esp_err_to_name(ret));
        return nullptr;
    } else {
        /* Fresh install – launch the host event daemon. */
        xTaskCreate(usb_host_daemon_task_fn, "usb_host_dmn", 4096,
                    nullptr, 10, &s_host_daemon_task);
        vTaskDelay(pdMS_TO_TICKS(200)); /* let daemon settle */
    }

    /* ------------------------------------------------------------------
     * 3.  CDC-ACM class driver
     * ------------------------------------------------------------------ */
    const cdc_acm_host_driver_config_t cdc_drv_cfg = {
        .driver_task_stack_size = 4096,
        .driver_task_priority   = 11,
        .xCoreID                = 0,
        .new_dev_cb             = nullptr,
    };
    ret = cdc_acm_host_install(&cdc_drv_cfg);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "CDC-ACM driver already installed – reusing");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "cdc_acm_host_install: %s", esp_err_to_name(ret));
        return nullptr;
    }

    /* ------------------------------------------------------------------
     * 4.  Create the terminal (must exist before open so callbacks are valid)
     * ------------------------------------------------------------------ */
    auto usb_term  = std::make_unique<UsbTerminal>(/*ring_size*/ 32768);
    UsbTerminal *term_raw = usb_term.get();

    /* ------------------------------------------------------------------
     * 5.  Open the BG95 CDC-ACM modem interface
     * ------------------------------------------------------------------ */
    const cdc_acm_host_device_config_t dev_cfg = {
        .connection_timeout_ms = (conn_timeout_ms > 0) ? conn_timeout_ms : 15000u,
        .out_buffer_size       = 16384,
        .in_buffer_size        = 0,   /* 0 = use endpoint MPS (driver chunks automatically) */
        .event_cb              = cdc_event_cb,
        .data_cb               = cdc_rx_cb,
        .user_arg              = term_raw,
    };

    cdc_acm_dev_hdl_t cdc_dev = nullptr;
    ESP_LOGI(TAG, "Waiting for BG95 USB (VID=0x%04X PID=0x%04X iface=%u)...",
             vid, pid, interface_num);
    ret = cdc_acm_host_open(vid, pid, interface_num, &dev_cfg, &cdc_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "cdc_acm_host_open: %s", esp_err_to_name(ret));
        return nullptr;
    }
    term_raw->dev_hdl_ = cdc_dev;
    ESP_LOGI(TAG, "BG95 USB CDC-ACM opened (iface %u)", interface_num);

    /* ------------------------------------------------------------------
     * 6.  Wrap in DTE
     * ------------------------------------------------------------------ */
    auto dte = std::make_shared<esp_modem::DTE>(dte_config,
                                                std::move(usb_term));

    /* ------------------------------------------------------------------
     * 7.  Build the esp_modem_dce_wrap (mirrors esp_modem_new_dev internals)
     *     so the returned pointer is fully compatible with every
     *     esp_modem_* C function.
     * ------------------------------------------------------------------ */
    auto *dce_wrap = new (std::nothrow) esp_modem_dce_wrap;
    if (!dce_wrap) {
        ESP_LOGE(TAG, "esp_modem_dce_wrap alloc failed");
        return nullptr;
    }

    dce_wrap->dte = dte; /* keep a shared_ptr reference */

    dce_factory::Factory f(dce_factory::ModemType::GenericModule);
    dce_wrap->dce = f.build(dce_config, std::move(dte), netif);
    if (!dce_wrap->dce) {
        ESP_LOGE(TAG, "DCE factory build failed");
        delete dce_wrap;
        return nullptr;
    }

    dce_wrap->modem_type = dce_factory::ModemType::GenericModule;
    dce_wrap->dte_type   = esp_modem_dce_wrap::modem_wrap_dte_type::USB;

    ESP_LOGI(TAG, "USB DCE created successfully");
    return dce_wrap;
}
