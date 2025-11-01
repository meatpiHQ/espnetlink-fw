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

#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"

#include "usb_cdc_host_manager.h"
#include "modem_manager.h"

// Change these values to match your needs
#define EXAMPLE_BAUDRATE     (115200)
#define EXAMPLE_STOP_BITS    (0)      // 0: 1 stopbit, 1: 1.5 stopbits, 2: 2 stopbits
#define EXAMPLE_PARITY       (0)      // 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space
#define EXAMPLE_DATA_BITS    (8)

static const char *TAG = "VCP manager example";
static SemaphoreHandle_t device_disconnected_sem;

// RX callbacks for each interface
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
    ESP_LOG_BUFFER_HEX(TAG, data, data_len);
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
        ESP_ERROR_CHECK_WITHOUT_ABORT(usb_cdc_mgr_tx(iface, (const uint8_t *)cmd, strlen(cmd), 1000));
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void app_main(void)
{
    //init modem manager here
    
    // Initialize BG95 modem (set 3M baud + HW flow control, ensure command mode)
    if (modem_mgr_init() != ESP_OK) {
        ESP_LOGE(TAG, "Modem init failed");
    }

    vTaskDelay(pdMS_TO_TICKS(10000));
    //init cdc manager
    device_disconnected_sem = xSemaphoreCreateBinary();
    assert(device_disconnected_sem);

    // Install host + CDC-ACM and start event task
    ESP_ERROR_CHECK(usb_cdc_mgr_init());

    // Open interface 0
    ESP_ERROR_CHECK(usb_cdc_mgr_open(USB_VID_QINHENG, USB_PID_CH342, USB_CDC_IFACE_0,
                                     5000, 5120, 5120, handle_event0, handle_rx0, NULL));
    ESP_ERROR_CHECK(usb_cdc_mgr_set_line_coding(USB_CDC_IFACE_0,
                                                EXAMPLE_BAUDRATE,
                                                EXAMPLE_STOP_BITS,
                                                EXAMPLE_PARITY,
                                                EXAMPLE_DATA_BITS));

    // Open interface 2
    ESP_ERROR_CHECK(usb_cdc_mgr_open(USB_VID_QINHENG, USB_PID_CH342, USB_CDC_IFACE_2,
                                     5000, 5120, 5120, handle_event2, handle_rx2, NULL));
    ESP_ERROR_CHECK(usb_cdc_mgr_set_line_coding(USB_CDC_IFACE_2,
                                                EXAMPLE_BAUDRATE,
                                                EXAMPLE_STOP_BITS,
                                                EXAMPLE_PARITY,
                                                EXAMPLE_DATA_BITS));

    // Start TX tasks for each interface
    xTaskCreate(iface_tx_task, "iface0_tx", 4096, (void *)USB_CDC_IFACE_0, 5, NULL);
    xTaskCreate(iface_tx_task, "iface2_tx", 4096, (void *)USB_CDC_IFACE_2, 5, NULL);
}
