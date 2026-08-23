# usb_dev_ethernet

**Summary** — The shipped USB mode: a CherryUSB **RNDIS** device (VID 0x303A / PID 0x4007, serial = device MAC) that makes ESPNetLink appear as a USB Ethernet adapter. Bridges RNDIS frames into a raw lwIP netif at 192.168.7.1/24 with a per-netif DHCP server, runs a tiny DNS proxy on UDP/53 that forwards USB-client queries to the upstream (LTE) resolver, and enables lwIP NAPT so host traffic rides the PPPoS uplink. Optionally composes the `usb_cli_console` CDC-ACM interface into the same USB configuration. `main` starts it; `lte_upstream_pppos` calls `usb_dev_ethernet_enable_sharing()` on PPP GOT_IP.

## API

- `esp_err_t usb_dev_ethernet_start(void)` — bring up the USB device stack + RNDIS, lwIP netif, DHCP server, and DNS proxy.
- `void usb_dev_ethernet_set_console_enabled(bool)` — include the CDC-ACM console in the composite; must be called **before** `usb_dev_ethernet_start()`.
- `esp_err_t usb_dev_ethernet_enable_sharing(uint32_t dns_ipv4_addr)` — record the upstream DNS and enable NAPT on the USB interface; safe to call before USB is up (the request is stored and applied once lwIP starts).

## Dependencies

- `cherry-embedded__cherryusb` — device stack + RNDIS class
- `lwip` — raw netif, etharp, sockets, NAPT (**`CONFIG_LWIP_IPV4_NAPT=y` required for sharing**); `esp_netif` (per-netif `dhcpserver`, coexists with the WiFi AP's DHCP on UDP/67)
- `usb_cli_console` — composite CDC console (its endpoints are reserved around RNDIS's 0x81/0x02/0x83)
- Order: `set_console_enabled()` → `start()`; `enable_sharing()` any time after (normally from lte_upstream_pppos's GOT_IP handler).

## Settings

None (no persisted settings — sharing is driven by `lte_upstream_pppos`'s `ncm_share` setting; console enable and the DHCP router/DNS offer (`usb_dev_ethernet_set_router_offer()`, on only when sharing) are wired by `main`). The USB link is a control channel unless shared: the host's lease carries address + netmask only.

## Memory footprint (estimated)

**All numbers estimated.**

- Static (`.bss`/`.data`): `usbnet_state_t` (netif + DHCP + pending-share state, ~100 B); USB descriptors + string buffers (<300 B); RNDIS endpoint/frame buffers live inside the CherryUSB class driver (config-sized).
- Task stacks (all static PSRAM, `EXT_RAM_BSS_ATTR`, pinned core 0): `usbnet_rx` 4096, prio 10; `usbnet_evt` 3072, prio 11; `usbnet_dns` 4096, prio 9 (keeps a 512 B packet buffer on its stack).
- Peak heap: not yet measured (frame pbufs come from the lwIP pools per packet).
