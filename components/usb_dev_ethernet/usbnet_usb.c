#include "private/usbnet_usb.h"

#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "lwip/tcpip.h"

#include "esp_attr.h"
#include "usbd_core.h"
#include "usbd_cdc_acm.h"

#include "usb_cli_console.h"

#include "usbd_cdc_ncm.h"
#include "usbd_rndis.h"

#include "private/usbnet_class.h"
#include "private/usbnet_dns_proxy.h"
#include "private/usbnet_lwip.h"
#include "private/usbnet_state.h"

static const char *TAG = "usb_dev_ethernet";

// USB Ethernet function endpoints (same triple for ECM and RNDIS)
#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x02
#define CDC_INT_EP 0x83

#define USBD_VID 0x303A
#define USBD_PID 0x4007

#define USBD_MAX_POWER_MA 100

#define USB_STRING_SERIAL_INDEX       0x03
/* index 4: RNDIS = interface name string; ECM = the iMACAddress string
 * (12 hex chars) that tells the host which MAC its adapter should use */
#define USB_STRING_ETH_INTF_INDEX     0x04
#define USB_STRING_CONSOLE_INTF_INDEX 0x05

#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#else
#define CDC_MAX_MPS 64
#endif

/* Both classes are runtime-selectable (usbnet_class_select, set before
 * usbnet_usb_start from the usb_dev_ethernet `class` setting) — one config
 * descriptor per (class, console) combination, picked in the callback. */
#define USB_CONFIG_SIZE_NCM       (9 + CDC_NCM_ALT_DESCRIPTOR_LEN)
#define USB_CONFIG_SIZE_NCM_CDC   (9 + CDC_NCM_ALT_DESCRIPTOR_LEN + CDC_ACM_DESCRIPTOR_LEN)
#define USB_CONFIG_SIZE_RNDIS     (9 + CDC_RNDIS_DESCRIPTOR_LEN)
#define USB_CONFIG_SIZE_RNDIS_CDC (9 + CDC_RNDIS_DESCRIPTOR_LEN + CDC_ACM_DESCRIPTOR_LEN)
#define USB_CONFIG_SIZE_CDC_ONLY  (9 + CDC_ACM_DESCRIPTOR_LEN)

static bool s_console_enabled = false;
/* false = the console alone (wifi_modem usb_mode=console, 2026-10-12) */
static bool s_ethernet_enabled = true;

// Both ECM and RNDIS are 2-interface functions (control+data)
static struct usbd_interface s_eth_intf0;
static struct usbd_interface s_eth_intf1;

static TaskHandle_t s_rx_task = NULL;
static TaskHandle_t s_evt_task = NULL;

static StaticSemaphore_t s_tx_done_sem_buf;
static SemaphoreHandle_t s_tx_done_sem = NULL;

#define USBNET_EVT_CONFIGURED (1u << 0)
#define USBNET_EVT_DISCONNECT (1u << 1)
#define USBNET_EVT_LINK_UP    (1u << 2) /* NCM: host selected data alt 1 */
#define USBNET_EVT_LINK_DOWN  (1u << 3)

// -------------------------
// Descriptors (advance-desc)
// -------------------------

static const uint8_t s_device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
};

static const uint8_t s_config_descriptor_ncm[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE_NCM, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER_MA),
    CDC_NCM_ALT_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MAX_MPS, 1514, USB_STRING_ETH_INTF_INDEX),
};

static const uint8_t s_config_descriptor_ncm_cdc[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE_NCM_CDC, 0x04, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER_MA),
    CDC_NCM_ALT_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MAX_MPS, 1514, USB_STRING_ETH_INTF_INDEX),
    // CDC-ACM console uses interface numbers 0x02 and 0x03
    CDC_ACM_DESCRIPTOR_INIT(0x02, 0x84, 0x04, 0x82, CDC_MAX_MPS, USB_STRING_CONSOLE_INTF_INDEX),
};

static const uint8_t s_config_descriptor_rndis[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE_RNDIS, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER_MA),
    CDC_RNDIS_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MAX_MPS, USB_STRING_ETH_INTF_INDEX),
};

static const uint8_t s_config_descriptor_rndis_cdc[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE_RNDIS_CDC, 0x04, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER_MA),
    CDC_RNDIS_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MAX_MPS, USB_STRING_ETH_INTF_INDEX),
    // CDC-ACM console uses interface numbers 0x02 and 0x03
    CDC_ACM_DESCRIPTOR_INIT(0x02, 0x84, 0x04, 0x82, CDC_MAX_MPS, USB_STRING_CONSOLE_INTF_INDEX),
};

/* The CDC-ACM console ALONE: interfaces 0 and 1 (CherryUSB numbers
 * interfaces in registration order, and the console is the only function
 * registered then), the console's own endpoints, no Ethernet function. */
static const uint8_t s_config_descriptor_cdc_only[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE_CDC_ONLY, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER_MA),
    CDC_ACM_DESCRIPTOR_INIT(0x00, 0x84, 0x04, 0x82, CDC_MAX_MPS, USB_STRING_CONSOLE_INTF_INDEX),
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

#define USB_MAC_STRING_DESC_LEN sizeof("001122334455")
#define USB_ETH_STRING_DESC_LEN sizeof("RNDIS 001122334455")

static char s_serial_string_descriptor[USB_MAC_STRING_DESC_LEN] = "000000000003";
/* RNDIS: interface name; ECM: the host adapter's MAC as 12 hex chars */
static char s_eth_string_descriptor[USB_ETH_STRING_DESC_LEN] = "000000000002";

static const char *s_string_descriptors[] = {
    (const char[]){ 0x09, 0x04 },
    /* Windows composes the adapter name as "<manufacturer> <product>", so
     * the product must not repeat the manufacturer (was "ESPNetLink
     * ESPNetLink CDC-NCM" in Device Manager). Same scheme as WiCAN Pro. */
    "MeatPi Electronics",
    "ESPNetLink",
    s_serial_string_descriptor,
    s_eth_string_descriptor,
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

    if (!s_ethernet_enabled)
    {
        return s_config_descriptor_cdc_only;
    }

    if (usbnet_class_kind() == USBNET_CLASS_KIND_RNDIS)
    {
        return s_console_enabled ? s_config_descriptor_rndis_cdc
                                 : s_config_descriptor_rndis;
    }

    return s_console_enabled ? s_config_descriptor_ncm_cdc
                             : s_config_descriptor_ncm;
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

    if ((index == USB_STRING_SERIAL_INDEX) || (index == USB_STRING_ETH_INTF_INDEX))
    {
        uint8_t mac[6] = { 0 };
        esp_err_t err = esp_read_mac(mac, ESP_MAC_WIFI_STA);

        if (err == ESP_OK)
        {
            (void)snprintf(s_serial_string_descriptor,
                           sizeof(s_serial_string_descriptor),
                           "%02X%02X%02X%02X%02X%02X",
                           mac[0],
                           mac[1],
                           mac[2],
                           mac[3],
                           mac[4],
                           mac[5]);

            if (usbnet_class_kind() == USBNET_CLASS_KIND_NCM)
            {
                /* iMACAddress: the MAC the HOST adapter uses — same
                 * locally administered dev_mac^1 the RNDIS class assigns
                 * (set up in usbnet_usb_start before usbd_initialize). */
                const uint8_t *hm = usbnet_state()->rndis_host_mac;

                (void)snprintf(s_eth_string_descriptor,
                               sizeof(s_eth_string_descriptor),
                               "%02X%02X%02X%02X%02X%02X",
                               hm[0], hm[1], hm[2], hm[3], hm[4], hm[5]);
            }
            else
            {
                /* RNDIS: index 4 is a display name, not iMACAddress */
                (void)snprintf(s_eth_string_descriptor,
                               sizeof(s_eth_string_descriptor),
                               "RNDIS %02X%02X%02X%02X%02X%02X",
                               mac[0],
                               mac[1],
                               mac[2],
                               mac[3],
                               mac[4],
                               mac[5]);
            }
        }
        else
        {
            ESP_LOGW(TAG, "esp_read_mac failed for USB string (%d)", (int)err);
            (void)snprintf(s_serial_string_descriptor, sizeof(s_serial_string_descriptor), "000000000003");
            if (usbnet_class_kind() == USBNET_CLASS_KIND_NCM)
            {
                (void)snprintf(s_eth_string_descriptor, sizeof(s_eth_string_descriptor), "020000000002");
            }
            else
            {
                (void)snprintf(s_eth_string_descriptor, sizeof(s_eth_string_descriptor), "RNDIS");
            }
        }
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

void usbnet_usb_notify_rx(void)
{
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

void usbnet_usb_notify_tx_done(void)
{
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

static void usbnet_notify_evt(uint32_t bits);

void usbnet_usb_notify_link(bool up)
{
    usbnet_notify_evt(up ? USBNET_EVT_LINK_UP : USBNET_EVT_LINK_DOWN);
}

/* RNDIS data callbacks (NCM's live in usbnet_ncm.c — its RX needs the
 * NTB length latched before the wake). Per-class symbol names, so both
 * sets link; only the registered class's set ever fires. */
void usbd_rndis_data_recv_done(uint32_t len)
{
    (void)len;
    usbnet_usb_notify_rx();
}

void usbd_rndis_data_send_done(uint32_t len)
{
    (void)len;
    usbnet_usb_notify_tx_done();
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

        if (!s_ethernet_enabled)
        {
            /* console only: no data path, no netif; CONFIGURED arms the
             * console, a reset/disconnect needs nothing from us */
            if ((bits & USBNET_EVT_CONFIGURED) != 0u)
            {
                ESP_LOGI(TAG, "USB configured (CDC console only)");
                if (s_console_enabled)
                {
                    usb_cli_console_usb_on_configured(0);
                }
            }
            continue;
        }

        if (usbnet_class_kind() == USBNET_CLASS_KIND_NCM)
        {
            /* NCM: the data path opens when the host selects data alt 1
             * (link_event), NOT at set-configuration — the data endpoints
             * don't exist in alt 0. CONFIGURED only arms the console. */
            if ((bits & (USBNET_EVT_DISCONNECT | USBNET_EVT_LINK_DOWN)) != 0u)
            {
                ESP_LOGI(TAG, "USB %s", (bits & USBNET_EVT_DISCONNECT) != 0u
                                            ? "disconnected/reset" : "NCM link down");
                usbnet_ncm_stop();
                (void)tcpip_callback_with_block(usbnet_lwip_stop_in_tcpip, NULL, 1);
            }

            if ((bits & USBNET_EVT_CONFIGURED) != 0u)
            {
                ESP_LOGI(TAG, "USB configured");

                if (s_console_enabled)
                {
                    usb_cli_console_usb_on_configured(0);
                }
            }

            if ((bits & USBNET_EVT_LINK_UP) != 0u)
            {
                ESP_LOGI(TAG, "NCM link up (data alt 1)");
                usbnet_ncm_start();
                (void)tcpip_callback_with_block(usbnet_lwip_start_in_tcpip, NULL, 1);
            }
        }
        else
        {
            /* RNDIS: the data path opens at set-configuration; the device
             * asserts carrier itself (usbd_rndis_set_connect). */
            if ((bits & USBNET_EVT_DISCONNECT) != 0u)
            {
                ESP_LOGI(TAG, "USB disconnected/reset");
                usbnet_class_set_connect(false);
                (void)tcpip_callback_with_block(usbnet_lwip_stop_in_tcpip, NULL, 1);
            }

            if ((bits & USBNET_EVT_CONFIGURED) != 0u)
            {
                ESP_LOGI(TAG, "USB configured");
                usbnet_class_set_connect(true);

                if (s_console_enabled)
                {
                    usb_cli_console_usb_on_configured(0);
                }

                (void)tcpip_callback_with_block(usbnet_lwip_start_in_tcpip, NULL, 1);
            }
        }
    }
}

static void usbnet_notify_evt(uint32_t bits)
{
    if (!s_evt_task || bits == 0)
    {
        return;
    }

    if (xPortInIsrContext())
    {
        BaseType_t hp_task_woken = pdFALSE;
        (void)xTaskNotifyFromISR(s_evt_task, bits, eSetBits, &hp_task_woken);
        if (hp_task_woken)
        {
            portYIELD_FROM_ISR();
        }
    }
    else
    {
        (void)xTaskNotify(s_evt_task, bits, eSetBits);
    }
}

static void usbnet_usbd_event_handler(uint8_t busid, uint8_t event)
{
    (void)busid;

    switch (event)
    {
        case USBD_EVENT_CONFIGURED:
        {
            usbnet_notify_evt(USBNET_EVT_CONFIGURED);
            break;
        }
        case USBD_EVENT_DISCONNECTED:
        case USBD_EVENT_RESET:
        {
            usbnet_notify_evt(USBNET_EVT_DISCONNECT);
            break;
        }
        default:
            break;
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

void usbnet_usb_set_ethernet_enabled(bool enabled)
{
    s_ethernet_enabled = enabled;
}

bool usbnet_usb_get_ethernet_enabled(void)
{
    return s_ethernet_enabled;
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
        /* §2: static stack in PSRAM (pbuf shuttling — no flash writes). */
        static StaticTask_t s_rx_tcb;
        static StackType_t s_rx_stack[4096] EXT_RAM_BSS_ATTR;

        s_rx_task = xTaskCreateStaticPinnedToCore(usbnet_rx_task, "usbnet_rx",
                                                  4096, NULL, 10, s_rx_stack,
                                                  &s_rx_tcb, 0);
    }

    if (!s_evt_task)
    {
        /* §2: static stack in PSRAM. */
        static StaticTask_t s_evt_tcb;
        static StackType_t s_evt_stack[3072] EXT_RAM_BSS_ATTR;

        s_evt_task = xTaskCreateStaticPinnedToCore(usbnet_evt_task, "usbnet_evt",
                                                   3072, NULL, 11, s_evt_stack,
                                                   &s_evt_tcb, 0);
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
             "Starting CherryUSB %s%s (netif MAC %02X:%02X:%02X:%02X:%02X:%02X, host adapter MAC %02X:%02X:%02X:%02X:%02X:%02X)",
             s_ethernet_enabled ? usbnet_class_name() : "CDC console only",
             (s_ethernet_enabled && s_console_enabled) ? "+CDC" : "",
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

    // Add interfaces in the exact order they appear in the config descriptor:
    // - USB Ethernet (NCM or RNDIS): interfaces 0..1 (when enabled)
    // - CDC-ACM console (optional): interfaces 2..3, or 0..1 on its own
    if (s_ethernet_enabled)
    {
        if (usbnet_class_kind() == USBNET_CLASS_KIND_NCM)
        {
            usbd_add_interface(0, usbd_cdc_ncm_init_intf(&s_eth_intf0, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP));
            usbd_add_interface(0, usbd_cdc_ncm_init_data_intf(&s_eth_intf1));
        }
        else
        {
            usbd_add_interface(0, usbd_rndis_init_intf(&s_eth_intf0, CDC_OUT_EP, CDC_IN_EP, CDC_INT_EP, st->rndis_host_mac));
            usbd_add_interface(0, usbd_rndis_init_intf(&s_eth_intf1, CDC_OUT_EP, CDC_IN_EP, CDC_INT_EP, st->rndis_host_mac));
        }
    }

    if (s_console_enabled)
    {
        usb_cli_console_usb_add(0);
    }

    /* Control-channel mode (no sharing): leave UDP/53 on 192.168.7.1 CLOSED.
     * The IDF DHCP server always offers our address as DNS (option 6 even
     * with the DNS offer disabled), and a proxy with no upstream made the
     * host's resolver wait on it; a closed port answers ICMP port-unreachable
     * so the host moves on to its own resolvers at once. No netif at all
     * without the Ethernet function. */
    if (s_ethernet_enabled && st->offer_router)
    {
        usbnet_dns_proxy_start();
    }

    uintptr_t reg_base = (uintptr_t)ESP_USBD_BASE;
    int usb_ret = usbd_initialize(0, reg_base, usbnet_usbd_event_handler);
    if (usb_ret != 0)
    {
        ESP_LOGE(TAG, "usbd_initialize failed (%d)", usb_ret);
    }
}
