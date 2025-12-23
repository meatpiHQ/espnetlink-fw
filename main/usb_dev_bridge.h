#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Starts USB Device (2x CDC ACM) <-> (2x UART) bridges:
//  - CDC0 <-> LTE UART
//  - CDC1 <-> GPS UART
esp_err_t usb_dev_bridge_start(void);

#ifdef __cplusplus
}
#endif
