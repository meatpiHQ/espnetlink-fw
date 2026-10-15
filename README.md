# ESPNetLink

<p align="center">
  <img src="https://github.com/meatpiHQ/espnetlink-fw/assets/94690098/9cc7a7a6-045a-4f0f-9de6-a1b4978c9b16" alt="First Image" width="45%">
  <img src="https://github.com/meatpiHQ/espnetlink-fw/assets/94690098/d15d30bc-d6b0-4a3a-8eea-b675ab139639" alt="Second Image" width="45%">
</p>

## Table of Contents

1. [Flashing the Firmware](#1-flashing-the-firmware)
   - 1.1 [Requirements](#11-requirements)
   - 1.2 [Flashing with esptool-js (Web Browser)](#12-flashing-with-esptool-js-web-browser)
   - 1.3 [Flashing with esptool (Command Line)](#13-flashing-with-esptool-command-line)
2. [CLI Command Reference](#2-cli-command-reference)
   - 2.1 [General](#21-general)
   - 2.2 [System](#22-system)
   - 2.3 [Network](#23-network)
   - 2.4 [LTE](#24-lte)
   - 2.5 [GPS](#25-gps)
   - 2.6 [AGNSS](#26-agnss)
   - 2.7 [Power / Sleep](#27-power--sleep)
   - 2.8 [Production test](#28-production-test)

> **Developers:** building from source, the dev-loop flash paths (CDC
> OTA / HTTP OTA / UART recovery), monitoring, and on-target tests are
> documented in [BUILDING.md](BUILDING.md).

---

## 1. Flashing the Firmware

### 1.1 Requirements

Download the latest release files from the [Releases](../../releases) page. You will need these four files:

| File | Flash Address |
|------|--------------|
| `bootloader.bin` | `0x0` |
| `partition-table.bin` | `0x8000` |
| `ota_data_initial.bin` | `0x29000` |
| `espnetlink-fw_vXXX.bin` | `0x30000` |

### 1.2 Flashing with esptool-js (Web Browser)

1. Open [https://espressif.github.io/esptool-js/](https://espressif.github.io/esptool-js/) in a Chrome-based browser.
2. Set **Baudrate** to `460800`.
3. Click **Connect** and select your ESP32-S3 COM port.
4. Under **Flash Address**, add each file with its corresponding address:

   | Flash Address | File |
   |--------------|------|
   | `0x0` | `bootloader.bin` |
   | `0x8000` | `partition-table.bin` |
   | `0x29000` | `ota_data_initial.bin` |
   | `0x30000` | `espnetlink-fw_vXXX.bin` |

5. Click **Program** and wait for flashing to complete.
6. Reset the device when prompted.

### 1.3 Flashing with esptool (Command Line)

```bash
python esptool.py -p <PORT> -b 460800 \
  --before default_reset --after hard_reset \
  --chip esp32s3 write_flash \
  --flash_mode dio --flash_freq 80m --flash_size detect \
  0x0      bootloader/bootloader.bin \
  0x8000   partition_table/partition-table.bin \
  0x29000  ota_data_initial.bin \
  0x30000  espnetlink-fw_vXXX.bin
```

Replace `<PORT>` with your serial port (e.g., `COM10` on Windows, `/dev/ttyUSB0` on Linux).

---

## 2. CLI Command Reference

The device exposes a USB CDC-ACM serial console. Connect at any baud rate and type commands.

### 2.1 General

| Command | Description |
|---------|-------------|
| `ver` | Print firmware version and ESP-IDF version |
| `echo 0\|1` | Get or set device-side input echo |
| `debug 0\|1` | Get or set global log output (0 = off, 1 = debug) |
| `config list [-j]` | Print all configuration keys (add `-j` for JSON) |
| `config get/set <KEY> [VAL]` | Read or persist one key (applies on next boot) |
| `ota <size> [sha256]` | Firmware update over this console — see `tools/cdc_ota.py` |

Firmware updates: `python tools/cdc_ota.py` streams the current build into
the inactive OTA slot over this console (SHA-256 verified) and reboots —
no bootloader/UART access needed after the first flash.

The device also serves an HTTP settings API on the WiFi hotspot
(`ESPNetLink_<id>` → `http://192.168.80.1/api/settings`) and over USB
Ethernet (`http://192.168.7.1/...`) — see `components/HTTP_API.md`.

### 2.2 System

```
system -v                  Print firmware version string
system -i                  Print chip model, CPU cores, free heap, MAC address
system -s <unix_timestamp> Set system time (UTC); also triggers AGNSS time injection
reboot                     Cleanly stop LTE/PPP stack then reboot
```

### 2.3 Network

```
ping [ip] [-c count] [-i interval_ms]
    ICMP ping an IP address.
    Default target: 8.8.8.8, count: 4, interval: 1000 ms

speedtest [-d] [-u] [-b <bytes>]
    LTE throughput test via Cloudflare (speed.cloudflare.com).
    (no flags)  download + upload
    -d          download only
    -u          upload only
    -b <n>      bytes to download (default 102400); upload uses half
```

### 2.4 LTE

```
lte -s   Full modem status
lte -r   Signal quality (RSSI / BER)
lte -n   Network attachment state
lte -p   PPP connection status
lte -o   Operator name
lte -i   Current IP address
lte -q   Radio quality + PSM / eDRX power-saving state
lte -j   All parameters as JSON
```

`lte -j` also reports `vbat_mv` (the modem's supply voltage, AT+CBC) and
`modem_temp_c` (AT+QTEMP) when the modem answers them.

### 2.5 GPS

```
gps -p [-j]    Print current fix (add -j for JSON output)
gps -s         Stream raw NMEA sentences to the console
gps -t [0|1]   Enable (1) or disable (0) NMEA passthrough mode
gps -c         Check GPS UART health
gps -V         Query the module firmware version ($PQTMVERNO)
```

### 2.6 AGNSS

Assisted GNSS speeds up time-to-first-fix by injecting cached time and position into the GPS module.

```
agnss -s             Show AGNSS enabled state, injection status, and cached position
agnss -v             Save current EASY navigation data to NVS
agnss -e             Enable AGNSS (takes effect on next boot)
agnss -d             Disable AGNSS (takes effect on next boot)
agnss -i             Get current cache interval (seconds)
agnss -i <seconds>   Set cache interval (minimum 10 s)
```

### 2.7 Power / Sleep

```
sleep 1              Light sleep: power off GPS and LTE, keep USB active.
                     Resume with the 'wake' command.

sleep 2 [secs]       Deep sleep: power off everything including USB.
                     Wake on timer (if seconds given) or reset button.

sleep 3              Deep sleep: USB mux off, wake on WAKE_GPIO pin (active-low).

sleep 4 [thresh]     Deep sleep: wake on IMU motion (Wake-on-Motion).
                     thresh = acceleration threshold in mg, 1–255 (default 50).

wake                 Wake from sleep mode 1 (reboots to re-initialise peripherals).
```

### 2.8 Production test

The device side of the production tester (`tools/production/prodtest.py`
drives it over this console; every answer is one JSON line):

```
factory selftest [-w <ssid>]            Hardware self-test: identity, PSRAM, boot
                                        counters, USB mux readback, IMU (id, gravity,
                                        INT1 trace), LED, GPS (UART rx, module version,
                                        satellites), storage, LTE stage/SIM; -w adds a
                                        WiFi scan reporting that SSID's RSSI
factory button <ms>                     Wait up to <ms> for a button press and release
factory usbmux esp|ch343|bg95|off <ms>  Route the USB connector for <ms>, then back to
                                        the DIP route (the console drops meanwhile)
factory led red 0|1                     Drive the red LED on GPIO41
factory mark [<serial> [station]]       Write (or print) the factory record
                                        /data/factory.json
factory coredump [erase]                Print the stored panic summary (task, cause, PC,
                                        backtrace addresses) as JSON, or erase it
factory gpio <n>                        Dump one pin's live configuration and level
factory bootloader                      Reboot into ROM download mode (USB 303A:0009)
                                        for an esptool erase or re-flash with the
                                        DIP switch left in the run position
```
