#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Optional: set a stable MAC address string/address for the USB network interface.
// If not called, a placeholder MAC string may be used.
void usb_dev_descriptors_set_mac(const uint8_t mac_addr[6]);

#ifdef __cplusplus
}
#endif
