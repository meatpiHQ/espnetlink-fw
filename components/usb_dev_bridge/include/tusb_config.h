#pragma once

// TinyUSB configuration for ESP32-S3 USB-OTG device mode.
// Kept minimal: 2x CDC-ACM ports for UART bridging.

#ifdef __cplusplus
extern "C" {
#endif

// Pull in ESP-IDF configuration (e.g., CONFIG_TINYUSB_NET_MODE_NCM)
#include "sdkconfig.h"

// Enable device stack
#define CFG_TUD_ENABLED 1

// Root hub port (device)
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE)

// Speed
#ifndef CFG_TUD_MAX_SPEED
#define CFG_TUD_MAX_SPEED OPT_MODE_FULL_SPEED
#endif

// Device class drivers
#define CFG_TUD_CDC 2

// CDC-NCM (USB Ethernet)
// TinyUSB uses CFG_TUD_NCM to compile the NCM class driver.
// Keep it enabled when the project config enables NCM mode.
#if defined(CONFIG_TINYUSB_NET_MODE_NCM) && CONFIG_TINYUSB_NET_MODE_NCM
#define CFG_TUD_NCM 1
#endif

// Endpoint0 size
#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE 64
#endif

// CDC endpoint buffer sizes (Full-Speed max packet is 64)
#ifndef CFG_TUD_CDC_EP_BUFSIZE
#define CFG_TUD_CDC_EP_BUFSIZE 64
#endif

// CDC RX ring buffer within TinyUSB
#ifndef CFG_TUD_CDC_RX_BUFSIZE
#define CFG_TUD_CDC_RX_BUFSIZE 512
#endif

#ifndef CFG_TUD_CDC_TX_BUFSIZE
#define CFG_TUD_CDC_TX_BUFSIZE 512
#endif

// OS/RTOS
#define CFG_TUSB_OS OPT_OS_FREERTOS

// Memory section hints (keep default)
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))

// Debug
#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif

#ifdef __cplusplus
}
#endif
