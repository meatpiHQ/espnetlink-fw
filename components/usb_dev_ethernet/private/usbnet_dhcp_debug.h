#pragma once

#include "lwip/pbuf.h"

// Set to 1 to log decoded DHCP message types/options seen on RX/TX.
// (Kept local to this component; no Kconfig required.)
#ifndef USBNET_DHCP_DEBUG
#define USBNET_DHCP_DEBUG 1
#endif

#if USBNET_DHCP_DEBUG
void usbnet_debug_dhcp_pbuf(const char *dir, struct pbuf *p);
#endif
