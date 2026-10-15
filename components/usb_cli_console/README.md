# usb_cli_console

**Summary** — The `esp> ` CLI console on a CherryUSB CDC-ACM function, composed alongside RNDIS by `usb_dev_ethernet` (endpoints 0x04 OUT / 0x82 IN / 0x84 INT). Runs esp_console/cmdline_manager commands with OK/ERROR trailers, optional device-side echo, a GPS NMEA stream mode with `+++++++` / `=======` escape, and a raw-RX diversion used by its `ota` command to stream a firmware image into the inactive OTA slot with NAK-based CDC flow control (nothing is ever dropped).

## API

Public (`include/usb_cli_console.h`):

- `usb_cli_console_register_settings(void)` — register the `"usb_cli_console"` `{cli}` descriptor (main: between settings init and start).
- `usb_cli_console_set_enabled(bool)` / `usb_cli_console_is_enabled(void)` — gate the console transport.
- `usb_cli_console_usb_add(uint8_t busid)` — add the CDC-ACM interfaces/endpoints to the composite USB device (called by usb_dev_ethernet before `usbd_initialize`).
- `usb_cli_console_usb_on_configured(uint8_t busid)` — arm OUT transfers + start the console task on `USBD_EVENT_CONFIGURED`.
- `usb_cli_console_set_gps_stream_mode(bool)` — while enabled, ignore command lines and watch only for the escape sequence.

Console commands owned here (registered on the settings boot apply): `echo 0|1`, `ota <size_bytes> [sha256_hex]`.

## Dependencies

- `cherry-embedded__cherryusb` — CDC-ACM device class (the device stack itself is owned by usb_dev_ethernet)
- `gps` — passthrough auto-start reads the gps settings + stream hooks
- `console`, `vfs`, `cmdline_manager`, `settings_manager`, `json` — command plumbing + `{cli}` knob
- `hw_config`, `driver`, `esp_hw_support`; PRIV: `app_update`, `mbedtls` (OTA write + SHA-256)
- Composition order: `usb_cli_console_usb_add()` before the USB device initialises; the console runs only after the host configures the device.

## Settings

Descriptor `"usb_cli_console"`, version 1 (`usb_cli_console_settings.c`) — §6b minimal knob:

| field | type | default | meaning |
|---|---|---|---|
| `cli` | bool | `true` | register this transport's own commands (`echo`, `ota`) |

## Memory footprint (estimated)

**All numbers estimated.**

- Static (internal `.bss` — no `EXT_RAM_BSS_ATTR` here): OTA stream buffer 16 KB + OTA write chunk 4 KB (`usb_cli_ota.c`; consumed on the flash-writing console task), TX ring 4 KB (2026-10-15, was 2 KB; a write waits up to 1 s for the host to drain the ring instead of dropping, so a ~2 KB `lte -j` arrives whole), RX endpoint buffer 256 B, line buffer 128 B, TX in-flight chunk 64 B.
- Task stacks: `usb_cli` 6144, prio 8, pinned core 0, dynamic **internal** — required: the `ota` handler performs flash writes from this stack (§2 corollary).
- Peak heap: not yet measured.
