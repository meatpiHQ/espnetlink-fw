/**
 * @file usb_dev_ethernet_settings.c
 * @brief settings_manager descriptor for usb_dev_ethernet (standard §4.1).
 *
 * One field: `class` — which USB communications class the device presents
 * on the USB Ethernet link (192.168.7.1). CDC-NCM is the default (in-box
 * driver on Windows 11 / macOS / Linux); RNDIS remains for legacy Windows
 * 10 hosts and for USB hosts that prefer it. Reboot-to-apply: the class
 * shapes the USB descriptor tables, so it is consumed once at
 * usb_dev_ethernet_start(). A WiCAN host in `usb_rndis` mode sets this
 * remotely over PUT /api/settings/usb_dev_ethernet + /api/settings/submit
 * (docs/espnetlink_wican_integration.md).
 */
#include <string.h>

#include "esp_err.h"

#include "cJSON.h"
#include "settings_manager.h"

#include "sdkconfig.h"

#include "usb_dev_ethernet.h"

#if CONFIG_USBNET_CLASS_RNDIS
#define USBNET_CLASS_DEFAULT "rndis"
#else
#define USBNET_CLASS_DEFAULT "ncm"
#endif

static const settings_field_t FIELDS[] =
{
    SETTINGS_STR_ENUM("class", "ncm,rndis", USBNET_CLASS_DEFAULT),
};

static usb_dev_ethernet_class_t s_class = USB_DEV_ETHERNET_CLASS_NCM;

usb_dev_ethernet_class_t usb_dev_ethernet_class(void)
{
    return s_class;
}

static esp_err_t on_apply(const cJSON *settings)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(settings, "class");
    const char *cls = (cJSON_IsString(item) && item->valuestring != NULL)
                          ? item->valuestring
                          : USBNET_CLASS_DEFAULT;

    s_class = (strcmp(cls, "rndis") == 0) ? USB_DEV_ETHERNET_CLASS_RNDIS
                                          : USB_DEV_ETHERNET_CLASS_NCM;
    return ESP_OK;
}

esp_err_t usb_dev_ethernet_register_settings(void)
{
    static const settings_descriptor_t DESC =
    {
        .name        = "usb_dev_ethernet",
        .version     = 1,
        .fields      = FIELDS,
        .field_count = sizeof(FIELDS) / sizeof(FIELDS[0]),
        .on_apply    = on_apply,
    };

    return settings_manager_register(&DESC);
}
