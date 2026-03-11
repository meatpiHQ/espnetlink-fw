/*
 * Main application (C) using usb_cdc_host_manager to manage CH342 interfaces 0 and 2 separately
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "hw_config.h"
#include "filesystem.h"
#include "config_manager.h"
#include "gps.h"

#include "nvs.h"
#include "nvs_flash.h"

// Select exactly one mode (set one to 1, the other to 0)
#define USB_HOST_MODE        0
#define USB_DEV_MODE         0
#define USB_DEV_ETHERNET_MODE 1

#if USB_HOST_MODE
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"

#include "usb_cdc_host_manager.h"
#include "usb_cdc_uart_bridge.h"
#include "modem_manager.h"
#endif

#if USB_DEV_MODE
#include "usb_dev_bridge.h"
#endif

#if USB_DEV_ETHERNET_MODE
#include "usb_dev_ethernet.h"
#endif

// Change these values to match your needs
#define EXAMPLE_BAUDRATE     (2000000)
#define EXAMPLE_STOP_BITS    (0)      // 0: 1 stopbit, 1: 1.5 stopbits, 2: 2 stopbits
#define EXAMPLE_PARITY       (0)      // 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space
#define EXAMPLE_DATA_BITS    (8)

static const char *TAG = "VCP manager example";

#if USB_DEV_ETHERNET_MODE
// Optional: bring up BG95 PPPoS upstream and share it over USB-NCM.
// (No Kconfig use here; just flip this macro.)
#define ENABLE_LTE_UPSTREAM_PPPOS  1

#if ENABLE_LTE_UPSTREAM_PPPOS
#include "lte_upstream_pppos.h"

static void lte_upstream_task(void *arg)
{
    (void)arg;

    /* Load LTE configuration from config_manager (config.json / schema defaults) */
    bool lte_enabled = true;
    config_get_bool("LTE_ENABLED", &lte_enabled);
    if (!lte_enabled)
    {
        ESP_LOGI(TAG, "LTE disabled in config – skipping LTE upstream");
        vTaskDelete(NULL);
        return;
    }

    static char cfg_apn[64];
    static char cfg_user[32];
    static char cfg_pass[32];
    static char cfg_pin[8];

    bool ncm_sharing = false;
    config_get_bool("NCM_SHARE", &ncm_sharing);

    config_get_str("APN",      cfg_apn,  sizeof(cfg_apn));
    config_get_str("APN_USER", cfg_user, sizeof(cfg_user));
    config_get_str("APN_PASS", cfg_pass, sizeof(cfg_pass));
    config_get_str("APN_PIN",  cfg_pin,  sizeof(cfg_pin));

    ESP_LOGI(TAG, "LTE config: APN='%s' user='%s' pin=%s",
             cfg_apn,
             cfg_user[0] ? cfg_user : "(none)",
             cfg_pin[0]  ? "(set)"  : "(none)");

    const lte_upstream_pppos_config_t cfg = {
        .modem = {
            .uart_port = LTE_UART_PORT,
            .tx_pin = LTE_UART_TX_PIN,
            .rx_pin = LTE_UART_RX_PIN,
            .rts_pin = LTE_RTS_PIN,
            .cts_pin = LTE_CTS_PIN,
            .pwr_key_pin = LTE_PWR_KEY_PIN,
            .status_pin = LTE_STATUS_PIN,
            .dtr_pin = LTE_DTR_PIN,
            .status_on_is_low = true,
        },

        .apn  = cfg_apn,
        .user = cfg_user[0] ? cfg_user : NULL,
        .pass = cfg_pass[0] ? cfg_pass : NULL,
        .pin  = cfg_pin[0]  ? cfg_pin  : NULL,

        .init_modem_manager = true,
        .baud_rate = 3000000,
        .hw_flow_control = true,

        .enable_usb_ncm_sharing = ncm_sharing,
        .connect_timeout_ms = 0,
        /* If the modem gets stuck failing registration, issue a soft reboot. */
        .reboot_on_reg_timeout = true,
        .reboot_wait_ms = 90000,
        .use_cmux = true,
    };

    ESP_LOGI(TAG, "Starting LTE upstream PPPoS...");
    esp_err_t err = lte_upstream_pppos_start(&cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "lte_upstream_pppos_start failed: %s", esp_err_to_name(err));
    }

    vTaskDelete(NULL);
}
#endif // ENABLE_LTE_UPSTREAM_PPPOS
#endif

#if USB_HOST_MODE
static SemaphoreHandle_t device_disconnected_sem;
#endif

// RX callbacks for each interface
#if USB_HOST_MODE
static bool handle_rx0(const uint8_t *data, size_t data_len, void *arg)
{
    ESP_LOGI(TAG, "RX iface0: %.*s", (int)data_len, (const char *)data);
    ESP_LOG_BUFFER_HEX(TAG, data, data_len);
    return true;
}

static void handle_event0(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC-ACM error iface0 err_no=%d", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGI(TAG, "Device disconnected (iface0)");
        xSemaphoreGive(device_disconnected_sem);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        ESP_LOGI(TAG, "Iface0 serial state 0x%04X", event->data.serial_state.val);
        break;
    default: break;
    }
}

static bool handle_rx2(const uint8_t *data, size_t data_len, void *arg)
{
    ESP_LOGI(TAG, "RX iface2: %.*s", (int)data_len, (const char *)data);
    // ESP_LOG_BUFFER_HEX(TAG, data, data_len);
    return true;
}

static void handle_event2(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC-ACM error iface2 err_no=%d", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGI(TAG, "Device disconnected (iface2)");
        xSemaphoreGive(device_disconnected_sem);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        ESP_LOGI(TAG, "Iface2 serial state 0x%04X", event->data.serial_state.val);
        break;
    default: break;
    }
}

static void iface_tx_task(void *arg)
{
    const uint8_t iface = (uint32_t)arg; // 0 or 2
    const char *cmd = "ATI\r";
    while (1) {
        ESP_LOGI(TAG, "Sending command on iface %u: %s", iface, cmd);
        ESP_ERROR_CHECK_WITHOUT_ABORT(usb_cdc_mgr_tx(iface, (const uint8_t *)cmd, strlen(cmd), 1000));
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
#endif // USB_HOST_MODE

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    esp_log_level_set("*", ESP_LOG_NONE);
	gpio_reset_pin(USB_SEL_1);
	gpio_set_direction(USB_SEL_1, GPIO_MODE_INPUT);
	gpio_set_pull_mode(USB_SEL_1, GPIO_FLOATING);

	gpio_reset_pin(USB_SEL_2);
	gpio_set_direction(USB_SEL_2, GPIO_MODE_INPUT);
	gpio_set_pull_mode(USB_SEL_2, GPIO_FLOATING);

    if (gpio_get_level(USB_SEL_2) == 1)
    {
        ESP_LOGI(TAG, "USB_SEL_2 is HIGH: checking modem state");
        /* STATUS pin is LOW when modem is ON (status_on_is_low = true) */
        gpio_reset_pin(LTE_STATUS_PIN);
        gpio_set_direction(LTE_STATUS_PIN, GPIO_MODE_INPUT);
        gpio_set_pull_mode(LTE_STATUS_PIN, GPIO_FLOATING);

        gpio_reset_pin(LTE_PWR_KEY_PIN);
        gpio_set_direction(LTE_PWR_KEY_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(LTE_PWR_KEY_PIN, 0);

        if (gpio_get_level(LTE_STATUS_PIN) != 0)
        {
            /* Modem is OFF – pulse PWR_KEY HIGH for 800 ms to power it on */
            ESP_LOGI(TAG, "Modem is OFF – sending power-on pulse");
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LTE_PWR_KEY_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(800));
            gpio_set_level(LTE_PWR_KEY_PIN, 0);
            ESP_LOGI(TAG, "Modem power-on pulse complete");
        }
        else
        {
            ESP_LOGI(TAG, "Modem is already ON – skipping power-on pulse");
        }

        ESP_LOGI(TAG, "Entering infinite loop");
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if(gpio_get_level(USB_SEL_1) == 0)
    {
        ESP_LOGI(TAG, "USB_SEL_1 is LOW: entering flash mode");
        while (gpio_get_level(USB_SEL_1) == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
    }

    filesystem_init();

    config_manager_init();

    // Initialize GPS UART + parser task early so fix data is collected from boot
    esp_err_t gps_err = gps_init();
    if (gps_err != ESP_OK)
    {
        ESP_LOGE(TAG, "GPS init failed: %s", esp_err_to_name(gps_err));
    }

    #if ((USB_HOST_MODE + USB_DEV_MODE + USB_DEV_ETHERNET_MODE) != 1)
    #error "Select exactly one of USB_HOST_MODE, USB_DEV_MODE, USB_DEV_ETHERNET_MODE"
    #elif USB_HOST_MODE
    //init modem manager here
    
    // Initialize BG95 modem (set 3M baud + HW flow control, ensure command mode)
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
    if (modem_mgr_init(&modem_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Modem init failed");
    }

    vTaskDelay(pdMS_TO_TICKS(15000));

    device_disconnected_sem = xSemaphoreCreateBinary();
    assert(device_disconnected_sem);

    // Install host + CDC-ACM and start event task
    ESP_ERROR_CHECK(usb_cdc_mgr_init());



    ESP_ERROR_CHECK(usb_cdc_mgr_open(USB_VID_QINHENG, USB_PID_CH342, USB_CDC_IFACE_2,
                                     5000, 1024*15, 1024*15, handle_event2, handle_rx2, NULL));
    ESP_ERROR_CHECK(usb_cdc_mgr_set_line_coding(USB_CDC_IFACE_2,
                                                115200,
                                                EXAMPLE_STOP_BITS,
                                                EXAMPLE_PARITY,
                                                EXAMPLE_DATA_BITS));
    // Open interface 0
    // ESP_ERROR_CHECK(usb_cdc_mgr_open(USB_VID_QINHENG, USB_PID_CH342, USB_CDC_IFACE_0,
    //                                  5000, 5120, 5120, handle_event0, handle_rx0, NULL));
    // ESP_ERROR_CHECK(usb_cdc_mgr_set_line_coding(USB_CDC_IFACE_0,
    //                                             2000000,
    //                                             EXAMPLE_STOP_BITS,
    //                                             EXAMPLE_PARITY,
    //                                             EXAMPLE_DATA_BITS));


    // Start UART<->USB bridge: interface 0 <-> modem UART
    usb_cdc_uart_bridge_config_t br = {
        // UART (use LTE_* from hw_config.h)
        .uart_port = LTE_UART_PORT,
        .uart_tx_pin = LTE_UART_TX_PIN,
        .uart_rx_pin = LTE_UART_RX_PIN,
        .use_hw_flow = true,
        .uart_rts_pin = LTE_RTS_PIN,
        .uart_cts_pin = LTE_CTS_PIN,
        .uart_baud = 3000000,       // match modem_mgr target
        .uart_data_bits = 8,
        .uart_parity = 0,
        .uart_stop_bits = 1,

        // USB device selection (CH342 interface 0)
        .usb_vid = USB_VID_QINHENG,
        .usb_pid = USB_PID_CH342,
        .usb_interface = USB_CDC_IFACE_0,

        // USB line coding (match UART)
        .usb_baud = 3000000,
        .usb_data_bits = 8,
        .usb_parity = 0,
        .usb_stop_bits = 0,         // 0=1 stop bit

        // Buffers/timeouts
        .usb_conn_timeout_ms = 5000,
        .usb_tx_timeout_ms = 1000,
        .usb_out_buffer = 1024*15,
        .usb_in_buffer = 1024*15,

        // Optional taps (none)
        .event_cb = NULL,
        .rx_tap_cb = NULL,
        .user_ctx = NULL,
    };

    ESP_ERROR_CHECK(usb_cdc_uart_bridge_start(&br));
    #elif USB_DEV_MODE
    ESP_ERROR_CHECK(usb_dev_bridge_start());
    #elif USB_DEV_ETHERNET_MODE
    usb_dev_ethernet_set_console_enabled(true);
    ESP_ERROR_CHECK(usb_dev_ethernet_start());
    
#if USB_DEV_ETHERNET_MODE && ENABLE_LTE_UPSTREAM_PPPOS
    (void)xTaskCreate(lte_upstream_task, "lte_pppos", 6144, NULL, 5, NULL);
#endif
    #endif
    // Optionally, you can still use interface 0 independently as before
    //init cdc manager for other interfaces

    // Start TX tasks for each interface
    // xTaskCreate(iface_tx_task, "iface0_tx", 4096, (void *)USB_CDC_IFACE_0, 5, NULL);
    // xTaskCreate(iface_tx_task, "iface2_tx", 4096, (void *)USB_CDC_IFACE_2, 5, NULL);
}
