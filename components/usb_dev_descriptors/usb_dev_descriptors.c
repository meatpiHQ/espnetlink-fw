#include "usb_dev_descriptors.h"

#include <string.h>

#include "sdkconfig.h"
#include "tusb.h"

// This component provides TinyUSB descriptor callbacks for either:
// - CDC bridge (2x CDC-ACM) mode
// - CDC-NCM (USB Ethernet) mode with Windows MS OS 2.0 descriptors

// -------------------------
// Shared strings
// -------------------------

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,

#if CONFIG_ESP_NETLINK_MODE_USB_DEV_BRIDGE
    STRID_CDC0,
    STRID_CDC1,
#endif

#if CONFIG_ESP_NETLINK_MODE_USB_DEV_NCM
    STRID_NET,
    STRID_MAC,
#endif
};

static char s_mac_str[13] = "000000000000"; // 12 hex chars + NUL

void usb_dev_descriptors_set_mac(const uint8_t mac_addr[6])
{
    if (!mac_addr) {
        return;
    }
    // Uppercase hex, no separators (matches what many hosts expect)
    (void)snprintf(s_mac_str, sizeof(s_mac_str), "%02X%02X%02X%02X%02X%02X",
                   mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
}

static const char *s_string_desc_arr[] = {
    (const char[]){0x09, 0x04},
    "ESPNetLink",
#if CONFIG_ESP_NETLINK_MODE_USB_DEV_BRIDGE
    "ESPNetLink USB Bridge",
#else
    "ESPNetLink USB Ethernet",
#endif
    "000000000000",

#if CONFIG_ESP_NETLINK_MODE_USB_DEV_BRIDGE
    "LTE",
    "GPS",
#endif

#if CONFIG_ESP_NETLINK_MODE_USB_DEV_NCM
    "USB net",
    NULL, // MAC is handled dynamically
#endif
};

// -------------------------
// Device descriptor
// -------------------------

static tusb_desc_device_t const s_desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
#if CONFIG_ESP_NETLINK_MODE_USB_DEV_NCM
    .bcdUSB = 0x0201,
#else
    .bcdUSB = 0x0200,
#endif

    // Composite / IAD
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

    // Development VID/PID only. For production, use your own VID/PID.
    .idVendor = 0x303A,
    .idProduct = 0x4002,
    .bcdDevice = 0x0100,

    .iManufacturer = STRID_MANUFACTURER,
    .iProduct = STRID_PRODUCT,
    .iSerialNumber = STRID_SERIAL,

    .bNumConfigurations = 0x01,
};

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&s_desc_device;
}

// -------------------------
// Configuration descriptor
// -------------------------

#if CONFIG_ESP_NETLINK_MODE_USB_DEV_BRIDGE

#if (CFG_TUD_CDC < 2)
#error "ESPNetLink CDC bridge mode requires TinyUSB CDC count >= 2 (CONFIG_TINYUSB_CDC_COUNT=2)"
#endif

enum {
    ITF_NUM_CDC0 = 0,
    ITF_NUM_CDC0_DATA,
    ITF_NUM_CDC1,
    ITF_NUM_CDC1_DATA,
    ITF_NUM_TOTAL
};

#define EPNUM_CDC0_NOTIF   0x81
#define EPNUM_CDC0_OUT     0x02
#define EPNUM_CDC0_IN      0x82

#define EPNUM_CDC1_NOTIF   0x83
#define EPNUM_CDC1_OUT     0x04
#define EPNUM_CDC1_IN      0x84

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + (2 * TUD_CDC_DESC_LEN))

static uint8_t const s_desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0, 100),

    TUD_CDC_DESCRIPTOR(
        ITF_NUM_CDC0, STRID_CDC0,
        EPNUM_CDC0_NOTIF, 8,
        EPNUM_CDC0_OUT, EPNUM_CDC0_IN,
        CFG_TUD_CDC_EP_BUFSIZE),

    TUD_CDC_DESCRIPTOR(
        ITF_NUM_CDC1, STRID_CDC1,
        EPNUM_CDC1_NOTIF, 8,
        EPNUM_CDC1_OUT, EPNUM_CDC1_IN,
        CFG_TUD_CDC_EP_BUFSIZE),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return s_desc_configuration;
}

#endif // CONFIG_ESP_NETLINK_MODE_USB_DEV_BRIDGE

#if CONFIG_ESP_NETLINK_MODE_USB_DEV_NCM

#if !CFG_TUD_NCM
#error "ESPNetLink NCM mode requires TinyUSB Network mode NCM enabled (TinyUSB Stack -> Network mode -> NCM)"
#endif

enum {
    ITF_NUM_NET = 0,
    ITF_NUM_NET_DATA,
    ITF_NUM_TOTAL
};

#define EPNUM_NET_NOTIF   0x81
#define EPNUM_NET_OUT     0x02
#define EPNUM_NET_IN      0x82

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_NCM_DESC_LEN)

static uint8_t const s_desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0, 100),
    TUD_CDC_NCM_DESCRIPTOR(ITF_NUM_NET, STRID_NET, STRID_MAC,
                           EPNUM_NET_NOTIF, 64,
                           EPNUM_NET_OUT, EPNUM_NET_IN,
                           CFG_TUD_NET_ENDPOINT_SIZE, CFG_TUD_NET_MTU),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return s_desc_configuration;
}

// -------------------------
// BOS + MS OS 2.0 descriptors (Windows auto-bind to UsbNcm)
// -------------------------

#define MS_OS_20_DESC_LEN 0xB2
#define BOS_TOTAL_LEN (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

static uint8_t const s_desc_bos[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, 1),
};

uint8_t const *tud_descriptor_bos_cb(void)
{
    return s_desc_bos;
}

// MS OS 2.0 descriptor set: compatible ID WINNCM + DeviceInterfaceGUIDs
static uint8_t const s_desc_ms_os_20[] = {
    // Set header: length, type, windows version, total length
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

    // Configuration subset header: length, type, configuration index, reserved, configuration total length
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A),

    // Function subset header: length, type, first interface, reserved, subset length
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), ITF_NUM_NET, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08),

    // Compatible ID: "WINNCM"
    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
    'W', 'I', 'N', 'N', 'C', 'M', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // Registry property descriptor (DeviceInterfaceGUIDs)
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08 - 0x08 - 0x14), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
    U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A),
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00,
    'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00, 0x00, 0x00,
    U16_TO_U8S_LE(0x0050),

    // bPropertyData (UTF-16): {7B36B70A-12A1-4E73-9F0B-4B2B7C4B88C8}
    '{', 0x00, '7', 0x00, 'B', 0x00, '3', 0x00, '6', 0x00, 'B', 0x00, '7', 0x00, '0', 0x00, 'A', 0x00, '-', 0x00,
    '1', 0x00, '2', 0x00, 'A', 0x00, '1', 0x00, '-', 0x00, '4', 0x00, 'E', 0x00, '7', 0x00, '3', 0x00, '-', 0x00,
    '9', 0x00, 'F', 0x00, '0', 0x00, 'B', 0x00, '-', 0x00, '4', 0x00, 'B', 0x00, '2', 0x00, 'B', 0x00, '7', 0x00,
    'C', 0x00, '4', 0x00, 'B', 0x00, '8', 0x00, '8', 0x00, 'C', 0x00, '8', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00
};

TU_VERIFY_STATIC(sizeof(s_desc_ms_os_20) == MS_OS_20_DESC_LEN, "Incorrect MS OS 2.0 descriptor size");

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    if (stage != CONTROL_STAGE_SETUP) {
        return true;
    }

    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR) {
        return false;
    }

    // Windows requests MS OS 2.0 descriptor set with bRequest=1, wIndex=7
    if (request->bRequest == 1 && request->wIndex == 7) {
        uint16_t total_len = 0;
        memcpy(&total_len, s_desc_ms_os_20 + 8, 2);
        return tud_control_xfer(rhport, request, (void *)(uintptr_t)s_desc_ms_os_20, total_len);
    }

    return false;
}

#endif // CONFIG_ESP_NETLINK_MODE_USB_DEV_NCM

// -------------------------
// String descriptor callback
// -------------------------

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;

    static uint16_t desc_str[32];
    uint8_t chr_count = 0;

    if (index == 0) {
        memcpy(&desc_str[1], s_string_desc_arr[0], 2);
        chr_count = 1;
    } else {
#if CONFIG_ESP_NETLINK_MODE_USB_DEV_NCM
        if (index == STRID_MAC) {
            const char *str = s_mac_str;
            size_t len = strlen(str);
            if (len > 31) {
                len = 31;
            }
            chr_count = (uint8_t)len;
            for (uint8_t i = 0; i < chr_count; i++) {
                desc_str[1 + i] = (uint16_t)str[i];
            }
            desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
            return desc_str;
        }
#endif

        size_t arr_len = sizeof(s_string_desc_arr) / sizeof(s_string_desc_arr[0]);
        if (index >= arr_len || s_string_desc_arr[index] == NULL) {
            return NULL;
        }

        const char *str = s_string_desc_arr[index];
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
