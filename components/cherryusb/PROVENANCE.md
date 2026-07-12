# cherryusb — vendored component

> **ESPNetLink (2026-07-12):** vendored unchanged from the WiCAN v6 tree
> (replacing the `cherry-embedded/cherryusb 1.6.1` managed component) to
> get the WiCAN-authored device-side CDC-NCM class — the `USBNET_CLASS`
> default. One local CMakeLists addition (see the ESP branch): device
> CDC-ECM + the RNDIS/ECM `USING_LWIP` pbuf glue that espnetlink's
> usb_dev_ethernet raw-lwip netif consumes. ESPNetLink runs the DEVICE
> stack (NCM/RNDIS/ECM + CDC-ACM console); the host stack compiles but
> is unused here. NCM data-path note: `usbd_cdc_ncm_eth_start_write()`
> returns the positive NTB total on success — usbnet_ncm.c maps it to
> the 0-on-success lwIP contract.

Vendored 2026-07-07 from **meatpi's fork**
https://github.com/meatpiHQ/CherryUSB branch `dynamic-shared-buffer`
(local clone's `sharred-buffer`, same commit
`bf7af0f6b044b4f6be99fd6c20e3e1a258fbd282` — "Fix ASIX RX buffer
overflow and frame parsing"). Upstream base: CherryUSB 1.5.2.

The fork's key change (NOT present in the upstream
`cherry-embedded__cherryusb 1.5.2~2` managed component the legacy
build used): `common/usbh_eth_shared_buf.{c,h}` — all five USB-eth
class drivers (CDC-ECM/NCM, RNDIS, ASIX, RTL8152) alias their RX/TX/
INT/CTRL buffers onto ONE shared DMA pool sized to the largest enabled
driver, instead of five private static sets. Only one eth class runs
at a time on this hardware.

Local changes on top of the fork (this repo):
- Trimmed: demo/, docs/, zephyr/, tools/, third_party/ (MSC-fatfs
  template only), port/* except port/dwc2 (ESP32-S3), README/images,
  idf_component.yml (vendored, not managed).
- `usbh_eth_shared_buf` made **heap-backed** (internal-RAM budget,
  2026-07-07): the pool is allocated from INTERNAL|DMA heap by
  `usbh_eth_shared_buf_alloc()` when an eth class starts and freed on
  stop, so the RAM is only consumed while a USB-eth adapter is
  actually attached. See the file header for the contract.

- OSAL thread stacks → **PSRAM** (2026-07-07 pm, `osal/idf/
  usb_osal_idf.c`): every CherryUSB thread (usbh hub/psc 4 KB + the
  active eth-class RX 2 KB) now uses `xTaskCreatePinnedToCoreWithCaps`
  with SPIRAM stack caps (TCB stays internal via pvPortMalloc);
  `usb_osal_thread_delete` routes through `vTaskDeleteWithCaps`, which
  supports CherryUSB's self-delete pattern (delete(NULL)) via IDF's
  cleanup-task mechanism. Safe because the threads never touch flash
  and URBs/class state live in internal .bss (the IRAM DWC2 ISR never
  reads a PSRAM stack). Measured: +5.9 KB internal free / +5.6 KB
  largest block at full bench density; `USB ETH TARGET PASS` +
  `TS TARGET PASS` re-verified after. NOT moved (deliberately):
  `g_usbhost_bus` (~4 KB .bss — the ISR walks pipe/URB state in it;
  PSRAM would fault during flash-cache-off windows), `ep0_request_
  buffer`/`g_setup_buffer` (DMA), the ~10 KB IRAM ISR code (cache-off
  safety).

Consumers: `usb_eth_host` (+ `usb_acm_cli` later). The CherryUSB
platform glue `platform/idf/usbh_net.c` must stay EXCLUDED from the
build (usb_eth_host's CMake marks it HEADER_FILE_ONLY) — its strong
usbh_*_run/stop/eth_input symbols would bypass our netif glue.

- **DEVICE stack enabled for role=device (2026-07-07 pm, `CMakeLists.txt`
  ESP branch):** added `CONFIG_CHERRYUSB_DEVICE 1` +
  `CONFIG_CHERRYUSB_DEVICE_CDC_RNDIS 1` + `CONFIG_CHERRYUSB_DEVICE_CDC_ACM 1`
  so the device core (`core/usbd_core.c`) + RNDIS (`class/wireless/
  usbd_rndis.c`) + CDC-ACM device classes COMPILE. This is for the
  J2534 USB transport (usb_host_manager `role=device` → WiCAN is a
  USB-Ethernet device to a PC, TASK_j2534_server.md §5). Host + device
  stacks BOTH compile; only one runs at boot (the S3 OTG is host XOR
  device — role-gated; device init is never called in role=host).
  NCM device is NOT vendored (no `class/cdc/usbd_cdc_ncm.c` despite the
  cherryusb.cmake stub) so RNDIS (Windows-native) is the USB-Ethernet
  device class. BUILD-PROVEN + no host-mode regression (device-enabled
  firmware boots clean in role=host, WiFi + `J2534 REFLASH/DRIVER PASS`
  re-verified, 21 KB internal free). The device→esp_netif bridge
  component (usb_net_device) is the remaining Phase-3 bring-up.

- **`class/cdc/usbd_cdc_ncm.{c,h}` — WICAN-AUTHORED (2026-07-08), not
  upstream:** CherryUSB has NO device-side NCM class at any version
  (only `usbh_cdc_ncm`); the cherryusb.cmake `CONFIG_CHERRYUSB_DEVICE_
  CDC_NCM` stub existed with no file behind it. We wrote the class
  (NTB16-only, one NDP/NTB on TX, spec-conformant RX parse) because
  **Windows 11 24H2 removed the legacy RNDIS driver** — CDC-NCM is what
  the inbox `usbncm.inf` (UsbNcm.sys) binds. Differences vs the unused
  `CDC_NCM_DESCRIPTOR_INIT` template in `usb_cdc.h` (kept as-is): our
  `CDC_NCM_ALT_DESCRIPTOR_INIT` adds the **alt-0 (no EPs) / alt-1 data
  interface pair** (NCM 1.0 §5.3 — Windows requires it; link-up hangs
  off SET_INTERFACE alt 1) and sets data-interface protocol 0x01 (NTB).
  Style/API mirrors `usbd_cdc_ecm.c` (weak recv/send-done + link_event
  hooks; ESP port = USB-ISR context). Enabled in the CMakeLists ESP
  branch. Consumer: `usb_net_device` (NCM → esp_netif + DHCP server).
  BENCH-PASSED 2026-07-08: enumerates + starts on Windows 11 26200
  first try, DHCP lease over USB, `J2534 DRIVER TEST PASS` end-to-end
  through the cable. Sync note: if upstream ever ships a device NCM
  class, ours will collide on file name — diff before adopting theirs.
