# ESPNetLink WiFi-modem mode — how to use it, and what changed

*2026-08-22. State: shipped on `refactor` as the `wifi_modem` component
(standard-compliant port of the `gps-test-sandbox` MVP, commit 580fd36).
Bench-verified the same day on an old-layout ESPNetLink sample with rpi001
standing in for the WiCAN.*

Component reference: `components/wifi_modem/README.md`. Bench evidence for
the GPS-desense root cause: `gps-test-sandbox` branch,
`bench2_out/full_system/CH343_ROUTE_RESULTS.md`.

## 1. What it is, in one paragraph

The WiCAN Pro's USB host transmitter desenses the ESPNetLink's GPS while
the USB-Ethernet (NCM) link is active — the receiver never gets a cold
fix. The fix needs no hardware change: **the USB-C cable carries 5 V
only**, and everything else rides WiFi. The ESPNetLink cuts its USB data
lines at its mux, runs its WiFi AP, NATs that AP into the LTE link and
serves GPS over HTTP. The WiCAN joins the ESPNetLink's AP as a normal
(fallback) WiFi network, gets internet through it, and polls the fix from
`GET /api/gps` into the same `gps_*` parameters it used to get over USB.

Bench numbers (2026-08-22, this firmware, rpi001 `wtest1` as the client):
`GET /api/wifi_modem` 200 on the first request after joining, HTTP
`example.com` 200 in 0.7 s, HTTPS 200 in 0.5 s, DNS through the AP's
proxy, 50 KB ranged download at ~10–14 KB/s (Cat-M1), ICMP to the AP
answers in ~2.5 ms, GPS fix with 23–30 satellites, LTE ALDI Mobile /
eMTC band 28 at −51 dBm.

## 2. Setting it up

### 2.1 Zero-touch pairing (the product flow)

The dongle ships **stateless** about who is on the cable: `usb_mode=auto`
and a per-device AP key already provisioned. On every boot it enumerates
as NCM + CDC (USB data on), then:

- a **PC** gets tethering as before (web UI at `http://192.168.7.1`);
- a **WiCAN** uses the USB link as a point-to-point pairing channel:
  1. `GET http://192.168.7.1/api/info` → `device_type=espnetlink`, `device_id`
  2. `GET /api/wifi_modem/credentials` → `{ssid, password, device_id}` —
     served **only over USB** (403 over WiFi); stored/verified in the
     WiCAN's fallback slot
  3. `POST /api/wifi_modem/usb_data {"enabled":false}` → 500 ms later the
     data lines are cut at the mux. The cable is 5 V only, the WiCAN sees
     the USB device drop (expected) and joins the AP when it needs the
     uplink; GPS acquires undisturbed.

**Since 2026-10-12 the sequence above runs at EVERY dongle boot** and
nothing about the cut persists: the dongle always starts with the data
lines on (RNDIS by default, or NCM, plus the CDC console), the WiCAN
identifies it, re-reads the key (unchanged: no restart) and cuts again,
about 10 to 20 s after power-on; then the bus is quiet for the GPS. The
boot-cut hint of 2026-08-24 (a cut at 3 s of every later boot with a 60 s
self-restore) is gone: a user who forgot the AP password of such a dongle
had no way in. A runtime restore (`wifi_modem -d 1` / `POST
/api/wifi_modem/usb_data {"enabled":true}`) or simply the next boot brings
the USB link back. No pairing state, nothing to un-pair: a new or
factory-reset WiCAN reads the key again whenever the USB link is up.

Manual overrides (settings PUT / `wifi_modem -u ...` + reboot):
`usb_mode=ncm` locks the data lines on (the cut is refused with 409);
`usb_mode=console` is the charger-powered, AP-only mode: USB presents the
CDC console ALONE (no USB Ethernet), so `wifi_modem -k` and `wifi_modem
-u auto` are always one cable away (v1's `power_only` migrates to it).

**The USB link is an internet path again by default.**
`lte_upstream_pppos.ncm_share` defaults to `true` since 2026-10-12: the
USB DHCP server offers the dongle as default router and resolver and NAPT
sends the host's traffic into LTE. This is what a WiCAN in its default
`usb_rndis` mode needs, and a matching dongle default spares the dongle
restart at the first contact. A PC on the dongle therefore routes its
internet through the Cat-M1 link unless `ncm_share` is turned off per
device (then `192.168.7.1` stays reachable by the subnet route: web UI,
pairing API, and nothing else moves).

### 2.2 ESPNetLink checklist

1. DIP switch in the **run** position (old-layout board: sw0 = 1). In the
   flash position the firmware parks in `USB_SEL_1 is LOW: entering flash
   mode`.
2. SIM seated (symptom of a half-seated card: `SIM not ready after
   30000 ms` / `AT+CIMI failed`), LTE + GPS antennas connected.
3. **AP password.** The first boot of this firmware replaces the stock
   `12345678` with a random 12-character per-device password and reboots
   once (`per-device AP password provisioned`). The WiCAN reads it over
   USB; a human reads it with `wifi_modem -k` on a console or sets a new
   one in the web UI (WiFi → AP password). It never appears in logs.
4. Set the APN in `lte_upstream_pppos` if the SIM needs one (bench SIM:
   `telstra.internet`).

Healthy boot log (UART0, 115200; `usb_mode=auto`, WiCAN on the cable):

```
wifi_modem: usb_mode=auto ap_share=1
wifi_modem: usb_mode=auto: mux left on the DIP route (SEL1=1 SEL2=0)
wifi_manager: AP started
wifi_modem: DNS proxy listening on UDP/53
wifi_modem: AP sharing armed: NAPT on 192.168.80.1, DNS offered = AP (proxy)
usb_dev_ethernet: Starting CherryUSB NCM+CDC ...
usb_dev_ethernet: USB configured
wifi_modem: AP credentials handed to the USB host
wifi_modem: host requested usb_data=0 (applying)
wifi_modem: USB data CUT at the mux by the host (readback SEL1=1 SEL2=1) - cable is power only
usb_dev_ethernet: USB disconnected/reset
lte_upstream_pppos: SIM ready (IMSI: ...)
lte_upstream_pppos: PPP GOT IP
wifi_modem: PPP up: upstream DNS 101.168.244.101
```

Restoring the data lines at runtime (e.g. to reach the USB console from
the AP side): `POST /api/wifi_modem/usb_data {"enabled":true}` or
`wifi_modem -d 1` on the UART0 console — the host re-enumerates the
device, no reboot.

### 2.3 WiCAN

Firmware with the `espnetlink_link` component (meatpi-components). The
zero-touch sequence of §2.1 is IMPLEMENTED there (2026-08-24,
bench-verified: attach, identify, credentials, cut in ~2 s of USB link;
since 2026-10-12 it repeats at every dongle boot). The WiCAN's default
`espnetlink.mode` is `usb_rndis` since 2026-10-12 (the dongle stays a USB
LTE modem on the wire, GPS and health polled at 192.168.7.1); pick
`wifi_modem` on the WiCAN for the cut-after-pairing flow of §2.1.
`espnetlink.enabled=true` and `auto_pair=true` are the defaults. The
manual pairing path below still works when USB is not an option (key from
`wifi_modem -k`):

- WiFi: mode `apsta`, primary STA = the home network; leave a fallback
  slot free for the dongle.
- Pair (writes the wifi_manager fallback slot AND the component's own
  ssid, then reboot):

  ```
  POST http://<wican>/api/espnetlink/pair
  {"ssid":"ESPNetLink_894A5D","password":"<the dongle's AP password>"}
  → {"ok":true,"slot":1,"reboot_required":true}
  POST http://<wican>/api/settings/submit → {"reboot":true}
  ```

  or on the console: `espnetlink pair ESPNetLink_894A5D <password>` then
  `system -r`.
- Recommended for the car: `wifi_manager.sta_roam_interval_s = 60`
  (default 300) — how often the WiCAN looks for the home network while
  parked on the dongle. If the phone app is used over BLE in the car, set
  `interface_manager.sta_ble_handover = false` (a BLE client suspends the
  WiCAN's STA and the dongle uplink drops — phase-2 item).

### 2.4 Verify

- WiCAN console `espnetlink` (or `GET /api/espnetlink`): `uplink=espnetlink
  host=192.168.80.1`, `gps: valid=1`, `dongle: lte=1`, `polls=N fail=0`.
- Internet through the dongle: WiCAN `wcat http://example.com/` →
  `WCAT: 200`.
- Dongle health from any AP client: `GET http://192.168.80.1/api/wifi_modem`.
- Dongle status on its console: `wifi_modem` →
  `usb_mode=auto usb_data=0 usb_ethernet=1 ap_share=1 napt=1
  ap_ip=192.168.80.1 clients=1 upstream_dns=... mux SEL1=1 SEL2=1`
  (`usb_data=0` = cut by the host until the next boot).

### 2.5 Troubleshooting

| symptom | cause / fix |
|---|---|
| log ends at `USB_SEL_1 is LOW: entering flash mode` | DIP in the flash position → run position, power cycle |
| `SIM not ready` / `AT+CIMI failed` / `+QCSQ: "NOSERVICE"` forever | card not seated (no `SIM PIN required` line = no card seen) |
| WiCAN `uplink=none`, scan shows the dongle | wrong SSID/password in the fallback slot; re-pair with `wifi_modem -k`'s value |
| WiCAN on the dongle but `dongle: lte=0` | LTE not registered on the dongle (SIM/APN/antenna) — GPS still works unaided |
| WiCAN settings `mode` suddenly `ap` | a partial settings PUT resets the missing keys to defaults — always GET → modify → PUT the full object |
| nothing enumerates on the PC | the DIP must be in the run position; or the data lines were cut at runtime (`GET /api/wifi_modem` → `usb_data:false`) — `POST /api/wifi_modem/usb_data {"enabled":true}` over the AP, or power-cycle |
| `POST /api/wifi_modem/usb_data` → 409 | `usb_mode=ncm` locks the lines on; set `auto` |
| `GET /api/wifi_modem/credentials` → 403 | by design over WiFi — only the USB host gets the key |

## 3. Flashing a dongle that is plugged into a PC (bench gotcha)

In ROM download mode the firmware is not running, so the mux sits on the
DIP route (run position = ESP native USB → the connector). With the
dongle powered from a PC, the S3's USB-Serial-JTAG enumerates on that PC
(`USB JTAG/serial debug unit`, VID 303A:1001, a COM port) and **UART0
esptool flashing gets corrupted** (`Invalid head of packet`, the stub
upload fails). Flash over the USB-Serial-JTAG port instead — it is also
much faster:

```
esptool --chip esp32s3 -p <USB-JTAG COM> --before no-reset --after no-reset \
    write-flash 0x0 bootloader.bin 0x8000 partition-table.bin \
    0x29000 ota_data_initial.bin 0x30000 espnetlink-fw_*.bin
```

then reset the board (RTS pulse on the UART0 adapter, or power cycle —
an EN reset, not `esp_restart`: a software reboot keeps the USB PHY
enumerated on the PC). While the app runs the serial-JTAG is detached
from the pads (`main.c`, `usb_serial_jtag_ll_phy_enable_pad(false)` at
`app_main` start): before that, a PC's CDC driver toggling DTR/RTS on the
serial-JTAG during the ~3 s boot window reset the chip, and a software
reboot with a PC attached looped forever (bench 2026-08-23).
Enter download mode with the UART0 adapter's DTR/RTS first if the chip is
running (`esptool -p <UART0> --before default-reset --after no-reset chip-id`
parks it there even though its own transfer fails). When the dongle is
powered by the WiCAN (no PC on the USB), UART0 flashing works normally.

## 4. Firmware changes (ESPNetLink, `refactor`)

New component `components/wifi_modem/` — see its README for the API,
settings, footprint and tests. Composition hooks in `main/main.c`:

```c
init_step("wifi_modem_init", wifi_modem_init);                 // after gps_register_settings
init_step("wifi_modem_register_http", wifi_modem_register_http); // after gps_register_http
/* after settings_manager_start + log_manager_start + main_legacy_import_run: */
wifi_modem_provision(&reboot) → restart_tracker_restart(CONFIG_APPLY) once when it changed something
init_step("wifi_modem_apply_usb_route", wifi_modem_apply_usb_route); // BEFORE any USB stack
start_component("wifi_modem", wifi_modem_start);                 // after wifi_manager_start
if (wifi_modem_usb_data_enabled()) start usb_dev_ethernet; else log + skip
```

`main/CMakeLists.txt` adds `wifi_modem`; `sdkconfig.defaults` pins
`CONFIG_LWIP_IP_FORWARD=y` + `CONFIG_LWIP_IPV4_NAPT=y` (NAPT silently
drops out of a fresh config without IP_FORWARD — the on-target test app
caught exactly that on its first run). Docs: route map (`components/HTTP_API.md`),
CLI ownership table (`cmdline_manager/README.md`),
`COMPLIANCE_STATUS.md`, CI host-test list.

Also: `lte_upstream_pppos.ncm_share` defaults to `false` and
`usb_dev_ethernet` offers DHCP options 3 (router) and 6 (DNS) on the USB
link only when sharing is on (`usb_dev_ethernet_set_router_offer()`,
wired by main) — with the IDF server's default mask a PC's default route
moved onto the Cat-M1 link and its resolver waited on a proxy with no
upstream. Also fixed: the USB DHCP server is restarted after a bus reset
/ re-plug (it was stopped and never restarted, so a host re-enumerating
after the runtime restore fell to 169.254.x). `main.c` detaches the USB-Serial-JTAG from the
pads at `app_main` start (see §3).

Also fixed on the way: `refactor` HEAD (`e4c306d`) did not build —
`imu_manager`/`vpn_manager` CMakeLists listed their `*_events.c` files
although `event_manager` is not part of the ESPNetLink composition (the
files' own comments say they are excluded). Restored the SRCS lines.

Differences from the sandbox MVP: `usb_mode` is `auto|ncm|console`
(`auto|ncm|power_only` until 2026-10-12; default `auto`: the host cuts the
data lines at runtime, the zero-touch WiCAN flow; no pairing state on the
dongle), `GET /api/wifi_modem/
credentials` (USB-only) + `POST /api/wifi_modem/usb_data`, the AP password
is provisioned per device, `init/start/stop` lifecycle +
`wifi_modem_settings_config()` getters, CLI in its own file with argtable
options (`-k`, `-u`, `-a`, `-d`), DNS proxy stoppable and quiet on a busy
port, `gpio_hold_dis` before driving the mux, host tests + a pytest test
app.

## 5. Known gaps (phase 2 / product items)

- **Home policy** — at home the dongle stays fully on. Planned:
  `home_policy = lte_off | power_off | keep`, driven by the two transitions
  the WiCAN link task already logs; `power_off` needs a public VBUS control
  in the WiCAN's `usb_host_manager`.
- **BLE rule** — `interface_manager.sta_ble_handover` suspends STA while a
  BLE client is connected → uplink lost; needs an exemption while on the
  dongle.
- **Web UI** — no WiFi-modem page yet; status/switch are API + CLI.
- **WiCAN auto-pair state machine** — USB detect → `credentials` →
  `usb_data off` → store/verify → join when needed; VBUS cycle as the
  recovery path. The dongle side is complete and bench-tested with a PC
  playing the WiCAN.
- Runtime LTE on/off API, WiFi TX-power cap during acquisition, mDNS
  discovery, QR pairing; data-usage and VPN re-handshake behaviour on
  uplink changes are not measured yet.
