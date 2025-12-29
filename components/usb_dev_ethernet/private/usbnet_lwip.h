#pragma once

#include "lwip/err.h"

#ifdef __cplusplus
extern "C" {
#endif

void usbnet_lwip_input_in_tcpip(void *arg);
void usbnet_lwip_start_in_tcpip(void *arg);
void usbnet_lwip_stop_in_tcpip(void *arg);

#ifdef __cplusplus
}
#endif
