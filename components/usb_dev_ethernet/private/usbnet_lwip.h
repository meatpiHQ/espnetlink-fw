#pragma once

#include "lwip/err.h"

#ifdef __cplusplus
extern "C" {
#endif

void usbnet_lwip_input_in_tcpip(void *arg);
void usbnet_lwip_start_in_tcpip(void *arg);
void usbnet_lwip_stop_in_tcpip(void *arg);

/**
 * Called from within the tcpip thread. If enable_sharing() was invoked
 * before the USB LwIP interface was ready, this applies the deferred
 * request (NAT enable + DHCP DNS update) immediately.
 */
void usbnet_rndis_apply_pending_share(void);

#ifdef __cplusplus
}
#endif
