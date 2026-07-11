# usb_dev_bridge

**Summary** — **Legacy compile-time alternative USB mode** (`USB_DEV_MODE` in `main`): a TinyUSB composite device exposing CDC0 = transparent BG95 LTE UART passthrough (3 Mbaud) and CDC1 = GPS NMEA stream that switches into an interactive command console on the `+++++++` escape. It is **not composed in the shipped `USB_DEV_ETHERNET` mode** (superseded by `usb_dev_ethernet` + `usb_cli_console`); kept for bench/diagnostic builds.

## API

- `esp_err_t usb_dev_bridge_start(void)` — install TinyUSB (custom descriptors in `usb_descriptors.c`, config in `include/tusb_config.h`), install the GPS and LTE UART drivers, start the bridge/parser/console tasks.

No `_stop`: the mode is selected at compile time and runs for the life of the firmware.

## Dependencies

- `espressif__esp_tinyusb` (managed) — the only component on TinyUSB; the shipped mode uses CherryUSB instead
- `modem_manager` — the modem must already be powered and configured at 3 Mbaud
- `igrr__libnmea` — local NMEA fix parsing for the console's `gps` command
- `hw_config` (pins), `console`

## Settings

None (no persisted settings; predates the settings_manager port).

## Memory footprint (estimated)

**All numbers estimated.**

- Static (`.bss`/`.data`): escape buffer 32 B; string-descriptor scratch 64 B; TinyUSB CDC FIFOs 512 B RX + 512 B TX per CDC port (`tusb_config.h`); sentence queue + fix struct.
- Task stacks (all dynamic, internal): `gps_parser` 4096 prio 4; `gps_to_usb` 4096 prio 4; `usb_console` 4096 prio 4; `uart_to_usb` 4096 prio 5.
- Heap: UART driver rings — GPS 8 KB RX; LTE 8 KB RX + 8 KB TX. Peak heap: not yet measured.
