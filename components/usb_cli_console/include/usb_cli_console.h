#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void usb_cli_console_set_enabled(bool enabled);
bool usb_cli_console_is_enabled(void);

// Called by the USB device composition code to register CDC-ACM interfaces/endpoints.
void usb_cli_console_usb_add(uint8_t busid);

// Called on USBD_EVENT_CONFIGURED to arm OUT transfers and start the console.
void usb_cli_console_usb_on_configured(uint8_t busid);

// GPS stream input mode: while enabled, console ignores normal command lines and
// only watches for the escape sequence (+++++++ or ======= + Enter) to exit.
void usb_cli_console_set_gps_stream_mode(bool enabled);

#ifdef __cplusplus
}
#endif
