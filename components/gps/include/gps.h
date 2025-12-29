#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize GPS UART + parser task (idempotent).
esp_err_t gps_init(void);

// Enable/disable console streaming mode (raw NMEA sentences).
void gps_set_streaming(bool enabled);
bool gps_is_streaming(void);

// Register the `gps` console command (idempotent).
// This is called by usb_cli_console so new components can add commands.
void gps_console_register(void);

#ifdef __cplusplus
}
#endif
