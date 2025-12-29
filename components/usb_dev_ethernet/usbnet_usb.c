#include "private/usbnet_usb.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "lwip/tcpip.h"

#include "usbd_core.h"
#include "usbd_cdc_acm.h"
#include "usbd_rndis.h"

#include "usb_cli_console.h"

#include "private/usbnet_dns_proxy.h"
#include "private/usbnet_lwip.h"
#include "private/usbnet_state.h"

static const char *TAG = "usb_dev_eth";

// RNDIS endpoints
#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x02
#define CDC_INT_EP 0x83

#define USBD_VID 0x303A
#define USBD_PID 0x4007

#define USBD_MAX_POWER_MA 100

#define USB_STRING_RNDIS_INTF_INDEX   0x04
#define USB_STRING_CONSOLE_INTF_INDEX 0x05

#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#else
#define CDC_MAX_MPS 64
#endif

#define USB_CONFIG_SIZE_RNDIS     (9 + CDC_RNDIS_DESCRIPTOR_LEN)
#define USB_CONFIG_SIZE_RNDIS_CDC (9 + CDC_RNDIS_DESCRIPTOR_LEN + CDC_ACM_DESCRIPTOR_LEN)

static bool s_console_enabled = false;

static struct usbd_interface s_rndis_intf;

static TaskHandle_t s_rx_task = NULL;
static TaskHandle_t s_evt_task = NULL;

static StaticSemaphore_t s_tx_done_sem_buf;
static SemaphoreHandle_t s_tx_done_sem = NULL;

#define USBNET_EVT_CONFIGURED (1u << 0)
#define USBNET_EVT_DISCONNECT (1u << 1)

// -------------------------
// Descriptors (advance-desc)
// -------------------------

static const uint8_t s_device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
};

static const uint8_t s_config_descriptor_rndis[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE_RNDIS, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER_MA),
    CDC_RNDIS_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MAX_MPS, USB_STRING_RNDIS_INTF_INDEX),
};

static const uint8_t s_config_descriptor_rndis_cdc[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE_RNDIS_CDC, 0x04, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER_MA),
    CDC_RNDIS_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MAX_MPS, USB_STRING_RNDIS_INTF_INDEX),
    // CDC-ACM console uses interface numbers 0x02 and 0x03
    CDC_ACM_DESCRIPTOR_INIT(0x02, 0x84, 0x04, 0x82, CDC_MAX_MPS, USB_STRING_CONSOLE_INTF_INDEX),
};

static const uint8_t s_device_quality_descriptor[] = {
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
};

static const char *s_string_descriptors[] = {
    (const char[]){ 0x09, 0x04 },
    "ESPNetLink",
    "ESPNetLink RNDIS",
    "000000000000",
    "RNDIS",
    "Console",
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return s_device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    (void)speed;

    if (s_console_enabled)
    {
        return s_config_descriptor_rndis_cdc;
    }

    return s_config_descriptor_rndis;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return s_device_quality_descriptor;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void)speed;

    if (index >= (uint8_t)(sizeof(s_string_descriptors) / sizeof(s_string_descriptors[0])))
    {
        return NULL;
    }
    return s_string_descriptors[index];
}

static const struct usb_descriptor s_usb_desc = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
};

// -------------------------
// RX/TX pacing + tasks
// -------------------------

void usbd_rndis_data_recv_done(uint32_t len)
{
    (void)len;

    if (!s_rx_task)
    {
        return;
    }

    if (xPortInIsrContext())
    {
        BaseType_t hp_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(s_rx_task, &hp_task_woken);
        if (hp_task_woken)
        {
            portYIELD_FROM_ISR();
        }
    }
    else
    {
        xTaskNotifyGive(s_rx_task);
    }
}

void usbd_rndis_data_send_done(uint32_t len)
{
    (void)len;

    if (!s_tx_done_sem)
    {
        return;
    }

    if (xPortInIsrContext())
    {
        BaseType_t hp_task_woken = pdFALSE;
        (void)xSemaphoreGiveFromISR(s_tx_done_sem, &hp_task_woken);
        if (hp_task_woken)
        {
            portYIELD_FROM_ISR();
        }
    }
    else
    {
        (void)xSemaphoreGive(s_tx_done_sem);
    }
}

static void usbnet_rx_task(void *arg)
{
    (void)arg;

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        usbnet_state_t *st = usbnet_state();
        if (!st->lwip_ready)
        {
            continue;
        }

        (void)tcpip_callback_with_block(usbnet_lwip_input_in_tcpip, NULL, 1);
    }
}

static void usbnet_evt_task(void *arg)
{
    (void)arg;

    while (true)
    {
        uint32_t bits = 0;
        (void)xTaskNotifyWait(0, UINT32_MAX, &bits, portMAX_DELAY);

        if ((bits & USBNET_EVT_DISCONNECT) != 0u)
        {
            ESP_LOGI(TAG, "USB disconnected/reset");
            (void)usbd_rndis_set_connect(false);
            (void)tcpip_callback_with_block(usbnet_lwip_stop_in_tcpip, NULL, 1);
        }

        if ((bits & USBNET_EVT_CONFIGURED) != 0u)
        {
            ESP_LOGI(TAG, "USB configured");
            (void)usbd_rndis_set_connect(true);

            if (s_console_enabled)
            {
                usb_cli_console_usb_on_configured(0);
            }

            (void)tcpip_callback_with_block(usbnet_lwip_start_in_tcpip, NULL, 1);
        }
    }
}

static void usbnet_usbd_event_handler(uint8_t busid, uint8_t event)
{
    (void)busid;

    if (!s_evt_task)
    {
        return;
    }

    uint32_t notify_bits = 0;

    switch (event)
    {
        case USBD_EVENT_CONFIGURED:
        {
            notify_bits = USBNET_EVT_CONFIGURED;
            break;
        }
        case USBD_EVENT_DISCONNECTED:
        case USBD_EVENT_RESET:
        {
            notify_bits = USBNET_EVT_DISCONNECT;
            break;
        }
        default:
            break;
    }

    if (notify_bits == 0)
    {
        return;
    }

    if (xPortInIsrContext())
    {
        BaseType_t hp_task_woken = pdFALSE;
        (void)xTaskNotifyFromISR(s_evt_task, notify_bits, eSetBits, &hp_task_woken);
        if (hp_task_woken)
        {
            portYIELD_FROM_ISR();
        }
    }
    else
    {
        (void)xTaskNotify(s_evt_task, notify_bits, eSetBits);
    }
}

void usbnet_usb_set_console_enabled(bool enabled)
{
    s_console_enabled = enabled;
    usb_cli_console_set_enabled(enabled);
}

bool usbnet_usb_get_console_enabled(void)
{
    return s_console_enabled;
}

bool usbnet_usb_wait_tx_done(TickType_t ticks)
{
    if (!s_tx_done_sem)
    {
        return false;
    }

    return (xSemaphoreTake(s_tx_done_sem, ticks) == pdTRUE);
}

void usbnet_usb_start(void)
{
    if (!s_rx_task)
    {
        xTaskCreatePinnedToCore(usbnet_rx_task, "usbnet_rx", 4096, NULL, 10, &s_rx_task, 0);
    }

    if (!s_evt_task)
    {
        xTaskCreatePinnedToCore(usbnet_evt_task, "usbnet_evt", 3072, NULL, 11, &s_evt_task, 0);
    }

    if (!s_tx_done_sem)
    {
        s_tx_done_sem = xSemaphoreCreateBinaryStatic(&s_tx_done_sem_buf);
    }

    usbnet_state_t *st = usbnet_state();

    uint8_t dev_mac[6] = { 0 };
    esp_err_t err = esp_read_mac(dev_mac, ESP_MAC_WIFI_STA);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "esp_read_mac failed (%d), using fallback", (int)err);
        dev_mac[0] = 0x02;
        dev_mac[1] = 0x00;
        dev_mac[2] = 0x00;
        dev_mac[3] = 0x00;
        dev_mac[4] = 0x00;
        dev_mac[5] = 0x01;
    }
    else
    {
        dev_mac[0] = (uint8_t)((dev_mac[0] | 0x02) & 0xFE);
    }

    memcpy(st->hwaddr, dev_mac, sizeof(st->hwaddr));
    memcpy(st->rndis_host_mac, dev_mac, sizeof(st->rndis_host_mac));
    st->rndis_host_mac[5] ^= 0x01;
    st->rndis_host_mac[0] = (uint8_t)((st->rndis_host_mac[0] | 0x02) & 0xFE);

    ESP_LOGI(TAG,
             "Starting CherryUSB RNDIS%s (netif MAC %02X:%02X:%02X:%02X:%02X:%02X, Windows adapter MAC %02X:%02X:%02X:%02X:%02X:%02X)",
             s_console_enabled ? "+CDC" : "",
             dev_mac[0],
             dev_mac[1],
             dev_mac[2],
             dev_mac[3],
             dev_mac[4],
             dev_mac[5],
             st->rndis_host_mac[0],
             st->rndis_host_mac[1],
             st->rndis_host_mac[2],
             st->rndis_host_mac[3],
             st->rndis_host_mac[4],
             st->rndis_host_mac[5]);

#ifdef CONFIG_USBDEV_ADVANCE_DESC
    usbd_desc_register(0, &s_usb_desc);
#else
    ESP_LOGW(TAG, "Non-ADVANCE_DESC build not supported for composite descriptors");
    usbd_desc_register(0, &s_usb_desc);
#endif

    struct usbd_interface *intf = usbd_rndis_init_intf(&s_rndis_intf, CDC_OUT_EP, CDC_IN_EP, CDC_INT_EP, st->rndis_host_mac);
    usbd_add_interface(0, intf);

    if (s_console_enabled)
    {
        usb_cli_console_usb_add(0);
    }

    usbnet_dns_proxy_start();

    uintptr_t reg_base = (uintptr_t)ESP_USBD_BASE;
    int usb_ret = usbd_initialize(0, reg_base, usbnet_usbd_event_handler);
    if (usb_ret != 0)
    {
        ESP_LOGE(TAG, "usbd_initialize failed (%d)", usb_ret);
    }
}
