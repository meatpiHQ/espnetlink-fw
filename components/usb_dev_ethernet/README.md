# usb_dev_ethernet

**Summary**: the USB Ethernet device function: a CherryUSB **CDC-NCM or
RNDIS** device (VID 0x303A / PID 0x4007, serial = device MAC, class picked
at RUNTIME by the `class` setting since 2026-08-25, both compile in; the
default is RNDIS since 2026-10-12, matching the WiCAN's default pairing
mode so a fresh pair needs no dongle restart; NCM is the class Windows 11
/ macOS / Linux carry an in-box driver for) that makes ESPNetLink appear
as a USB Ethernet adapter. Bridges the class frames into a raw lwIP netif
at 192.168.7.1/24 with a per-netif DHCP server, runs a tiny DNS proxy on
UDP/53 that forwards USB-client queries to the upstream (LTE) resolver,
and enables lwIP NAPT so host traffic rides the PPPoS uplink. Composes
the `usb_cli_console` CDC-ACM interface into the same USB configuration,
and since 2026-10-12 can present that console ALONE (no Ethernet
function, interfaces 0 and 1, no netif): `wifi_modem usb_mode=console`.
`main` starts it in every mode and tells it which function to present
(`wifi_modem_usb_ethernet_enabled()`); `lte_upstream_pppos` calls
`usb_dev_ethernet_enable_sharing()` on PPP GOT_IP. A WiCAN host in
`usb_ncm`/`usb_rndis` mode sets the `class` remotely over
`PUT /api/settings/usb_dev_ethernet` + submit
(`docs/espnetlink_wican_integration.md`).

## API

- `esp_err_t usb_dev_ethernet_register_settings(void)` — the settings
  descriptor (below); register with the other settings init steps.
- `usb_dev_ethernet_class_t usb_dev_ethernet_class(void)` — the boot-applied
  class (`USB_DEV_ETHERNET_CLASS_NCM`/`_RNDIS`).
- `esp_err_t usb_dev_ethernet_start(void)` — bring up the USB device stack
  with the selected class, lwIP netif, DHCP server, and DNS proxy.
- `void usb_dev_ethernet_set_console_enabled(bool)`: include the CDC-ACM
  console in the composite; must be called **before** `usb_dev_ethernet_start()`.
- `void usb_dev_ethernet_set_ethernet_enabled(bool)`: present the USB
  Ethernet function at all (default true). `false` = the console alone
  (a fifth configuration descriptor: CDC-ACM at interfaces 0 and 1, the
  console's own endpoints, no NCM/RNDIS interfaces, no netif, no DHCP, no
  DNS proxy; the event task only arms the console on CONFIGURED). Before
  `usb_dev_ethernet_start()`; the console must be enabled with it.
- `esp_err_t usb_dev_ethernet_enable_sharing(uint32_t dns_ipv4_addr)` —
  record the upstream DNS and enable NAPT on the USB interface; safe to call
  before USB is up (the request is stored and applied once lwIP starts).

Internals: `usbnet_class.c` is the runtime dispatcher (rx/tx/set_connect +
which link-up model: NCM opens the data path on the host's data-alt-1
selection, RNDIS at set-configuration); `usbnet_ncm.c` adapts the
NTB-level WiCAN-authored NCM class to the shared pbuf contract;
`usbnet_usb.c` holds one config descriptor per (class, console)
combination and registers the selected class's interfaces.

## Dependencies

- `cherryusb` (vendored fork) — device stack + NCM/RNDIS classes (both
  always compiled; only the selected one is registered)
- `lwip` — raw netif, etharp, sockets, NAPT (**`CONFIG_LWIP_IPV4_NAPT=y`
  required for sharing**); `esp_netif` (per-netif `dhcpserver`, coexists
  with the WiFi AP's DHCP on UDP/67)
- `usb_cli_console` — composite CDC console (its endpoints are reserved
  around the eth function's 0x81/0x02/0x83)
- `settings_manager` + `espressif__cjson` — the `class` setting
- Order: `register_settings()` (init) → `set_console_enabled()` →
  `start()`; `enable_sharing()` any time after (normally from
  lte_upstream_pppos's GOT_IP handler).

## Settings (`"usb_dev_ethernet"`, version 1)

| Key | Type / default | Notes |
|---|---|---|
| `class` | `ncm` \| `rndis`, default per Kconfig `USBNET_CLASS` (`rndis` since 2026-10-12, `ncm` before) | which USB Ethernet class the device presents; reboot-to-apply (descriptor tables are built at start). RNDIS is the default because the WiCAN pairs in `usb_rndis` mode by default and a matching dongle default means no dongle restart at the first contact. CDC-ECM was dropped with the runtime selection (no Windows in-box driver). |

Sharing itself stays driven by `lte_upstream_pppos.ncm_share`; console
enable and the DHCP router/DNS offer (`usb_dev_ethernet_set_router_offer()`,
on only when sharing) are wired by `main`. The USB link is a control
channel unless shared: the host's lease carries address + netmask only.

## Memory footprint (estimated)

**All numbers estimated.**

- Static (`.bss`/`.data`): `usbnet_state_t` (netif + DHCP + pending-share
  state, ~100 B); USB descriptors (5 config variants) + string buffers
  (<600 B); endpoint/frame buffers live inside the CherryUSB class drivers
  (config-sized; the unselected class's static buffers are resident now
  that both link — RNDIS ~3.4 KB, NCM's NTBs are heap-allocated on start).
- Task stacks (all static PSRAM, `EXT_RAM_BSS_ATTR`, pinned core 0):
  `usbnet_rx` 4096, prio 10; `usbnet_evt` 3072, prio 11; `usbnet_dns`
  4096, prio 9 (keeps a 512 B packet buffer on its stack).
- Peak heap: not yet measured (frame pbufs come from the lwIP pools per
  packet).
