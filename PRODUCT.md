# ESPNetLink — Product Profile

Product facts required by `components/MEATPI_COMPONENT_STANDARD.md` (rev 1.0,
"Product profile" section). This file is the single place that records the
hardware/toolchain facts the standard refers to generically.

## Hardware

- **MCU:** ESP32-S3 (QFN56, rev v0.2), dual-core Xtensa @ 240 MHz
- **PSRAM:** 8 MB embedded octal (AP_3v3) — §2 applies in full
  (`CONFIG_SPIRAM_MODE_OCT`, bss/stack/noinit external allowed, memtest off)
- **Flash:** 16 MB QIO/QOUT @ 80 MHz
- **Peripherals:** Quectel BG95 LTE modem (UART, 3 Mbaud + HW flow control,
  PPPoS + CMUX), GNSS module (UART, NMEA + AGNSS/PAIR), ICM-42670 IMU (I2C),
  USB-OTG in device mode (CherryUSB composite: RNDIS + CDC-ACM console),
  USB mux (USB_SEL GPIOs: ESP / flash-UART / modem passthrough), XR21V1414
  quad UART bridge (channel A = UART0 boot console/flash @ COM-port)
- Schematic: `espnetlink_sch_v160.pdf`

## Toolchain (pinned)

- **ESP-IDF v5.5.3** — the exact version the product builds and tests with.
  (The standard's IDF v6 cJSON note does not apply yet: v5.5 still bundles
  the `json` component; the ported core components document this in their
  CMakeLists/idf_component.yml and switch to `espressif/cjson` on the v6
  migration.)
- Canonical minimal config: repo-root `sdkconfig.defaults`
- Partition table: repo-root `espnetlink_partitions_table.csv`
  (`ota_0`/`ota_1` 5 MB app slots, `settings` 256 KB LittleFS — the Settings
  Manager's dedicated partition per §4.4, `storage` 4 MB LittleFS)

## Network identity

- USB RNDIS: device = `192.168.7.1/24`, per-netif DHCP server, DNS proxy on
  the device address, NAT to the LTE (PPPoS) upstream
- WiFi AP (wifi_manager defaults): SSID `ESPNetLink_<MAC-tail>`, `192.168.80.1/24`
- HTTP API on `:80` (http_server_manager): `/api/settings*` (api_http),
  `/api/wifi/*` (wifi_manager)

## Firmware update

- **Primary:** `tools/cdc_ota.py` — `ota <size> [sha256]` command on the USB
  CDC console (`esp> `), streams the image into the inactive OTA slot,
  SHA-256 verified, reboot-to-swap.
- **Recovery/factory:** esptool over the UART0 bridge (see README §1).

## Core platform composition (standard §"Core platform components")

`log_manager`, `settings_manager`, `cmdline_manager`, `http_server_manager`
(+ `dev_status_manager`, `wifi_manager`, `api_http`) are shared with WiCAN —
ported from the WiCAN v6 tree, not rewritten. Product-specific deviations are
marked with "ESPNetLink port" comments at the deviation site.
