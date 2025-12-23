#include <string.h>

#include "tusb.h"
#include "sdkconfig.h"

// Minimal composite device: 2x CDC-ACM (CDC0 + CDC1)

enum {
    ITF_NUM_CDC0 = 0,
    ITF_NUM_CDC0_DATA,
    ITF_NUM_CDC1,
    ITF_NUM_CDC1_DATA,
    ITF_NUM_TOTAL
};

// Endpoint allocation (Full-Speed)
#define EPNUM_CDC0_NOTIF   0x81
#define EPNUM_CDC0_OUT     0x02
#define EPNUM_CDC0_IN      0x82

#define EPNUM_CDC1_NOTIF   0x83
#define EPNUM_CDC1_OUT     0x04
#define EPNUM_CDC1_IN      0x84

#define USB_VID 0x303A // Espressif (default; change for production)
#define USB_PID 0x4002 // Generic CDC device (example)
#define USB_BCD 0x0100

static tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,

    // Use IAD for composite
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,

#ifdef CFG_TUD_ENDPOINT0_SIZE
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
#else
    .bMaxPacketSize0 = 64,
#endif

    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = USB_BCD,

    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,

    .bNumConfigurations = 0x01,
};

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + (2 * TUD_CDC_DESC_LEN))

static uint8_t const desc_configuration[] = {
    // config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, 0, 100),

    // CDC0
    TUD_CDC_DESCRIPTOR(
        ITF_NUM_CDC0, 4,
        EPNUM_CDC0_NOTIF, 8,
        EPNUM_CDC0_OUT, EPNUM_CDC0_IN,
        CFG_TUD_CDC_EP_BUFSIZE),

    // CDC1
    TUD_CDC_DESCRIPTOR(
        ITF_NUM_CDC1, 5,
        EPNUM_CDC1_NOTIF, 8,
        EPNUM_CDC1_OUT, EPNUM_CDC1_IN,
        CFG_TUD_CDC_EP_BUFSIZE),
};

static char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04}, // 0: English (0x0409)
    "ESPNetLink",              // 1: Manufacturer
    "ESPNetLink USB Bridge",   // 2: Product
    "000000000000",            // 3: Serial (placeholder)
    "LTE",                     // 4: CDC0
    "GPS",                     // 5: CDC1
};

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_configuration;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;

    static uint16_t desc_str[32];
    uint8_t chr_count = 0;

    if (index == 0) {
        memcpy(&desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= (sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) {
            return NULL;
        }
        const char *str = string_desc_arr[index];
        size_t len = strlen(str);
        if (len > 31) {
            len = 31;
        }
        chr_count = (uint8_t)len;
        for (uint8_t i = 0; i < chr_count; i++) {
            desc_str[1 + i] = (uint16_t)str[i];
        }
    }

    desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
    return desc_str;
}
