#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Starts USB Device mode:
//  - CDC0: LTE UART passthrough
//  - CDC1: GPS UART stream by default; send "+++++++" to enter command console mode
esp_err_t usb_dev_bridge_start(void);

#ifdef __cplusplus
}
#endif
