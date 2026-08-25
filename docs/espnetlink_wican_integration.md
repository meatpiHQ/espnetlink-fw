# ESPNetLink ↔ WiCAN zero-touch pairing — what the WiCAN side needs

*2026-08-23. The ESPNetLink side is complete and bench-verified
(`refactor`, `components/wifi_modem/`, firmware `v1.22-39-ge4c306d-dirty`
or later). This document is the contract the WiCAN firmware implements
against, plus the concrete work list for `meatpi-components`
(`espnetlink_link`, `usb_host_manager`) and `wican-fw`.*

## 1. The flow in one picture

```
 dongle power-on ──► USB data ON (NCM + CDC enumerate, ~3 s)
                          │
 WiCAN USB host sees 303A:4007 ─► GET http://192.168.7.1/api/info   (device_type=espnetlink)
                          │
                          ├─► GET /api/wifi_modem/credentials       {ssid,password,device_id}  (USB only)
                          │        store / verify in the fallback slot + espnetlink_link settings
                          │
                          └─► POST /api/wifi_modem/usb_data {"enabled":false}
                                   200 now, lines cut 500 ms later ─► USB device drops (expected)
                                                                      cable = 5 V only, GPS undisturbed
 later, when the uplink is needed ─► join AP ESPNetLink_<id6> ─► internet via NAPT, GET /api/gps
```

Budget: dongle boot → cut in **≤ 10 s**. No pairing state on the dongle,
nothing to un-pair, no reboot on either side after the first pairing.

**2026-08-24 addendum — the boot-cut hint (`wifi_modem.boot_cut`).** The
flow above is now only the FIRST-pairing / key-re-read path. A
host-requested cut arms a device-state hint; every later boot in
`usb_mode=auto` cuts the data lines at ~3 s, before the USB device stack
starts — the WiCAN sees no USB device at all (its machine stays `idle`),
joins the AP as usual, and **GPS acquires from power-on with a quiet
bus** (no desense window, nothing to kill). Self-healing: if no station
joins the AP within 60 s the dongle restores the lines and clears the
hint (a PC gets tethering back one boot late; a WiCAN needing a key
re-read gets its USB channel after the same 60 s — VBUS-cycle recovery
is therefore ~75 s worst case). Any runtime `usb_data=1` restore or a
factory reset clears the hint. Correspondingly the WiCAN **no longer
VBUS-power-cycles at host start** (its connector mux provides the DWC2
connect edge), so a WiCAN reboot never reboots the dongle or costs a fix.

**2026-08-25 addendum — runtime USB Ethernet class (NCM / RNDIS).** The
USB device class is now selected at RUNTIME from the new
`usb_dev_ethernet` settings descriptor (v1, one field: `class` =
`ncm`|`rndis`, default `ncm`; Kconfig `USBNET_CLASS` only sets that
default now, and CDC-ECM was dropped). Both classes compile in; the
choice is consumed once at `usb_dev_ethernet_start()` (reboot-to-apply).
A WiCAN host in `mode=usb_ncm` / `mode=usb_rndis` makes the dongle match
during its ensure-share pass: `GET /api/settings/usb_dev_ethernet` ->
`class` differs -> PUT + `POST /api/settings/submit` (ONE dongle reboot,
shared with the `ncm_share` flip when both change). Firmware without the
descriptor answers 404 and the WiCAN stays on the dongle's built-in
class. Everything else (VID/PID 303A:4007, 192.168.7.1, the composite
CDC-ACM console, the cut contract and the boot-cut hint) is
class-independent.

## 2. Dongle contract (what the WiCAN can rely on)

### 2.1 USB

| Item | Value |
|---|---|
| Composite device (OTG) | **VID `0x303A` PID `0x4007`**, manufacturer `MeatPi Electronics`, product `ESPNetLink`; interfaces: CDC-NCM (0–1, data alt 1) + CDC-ACM console (2–3) |
| Dongle address on the link | `192.168.7.1/24`; the host gets `192.168.7.2/24` by DHCP — **address + netmask only, no router and no DNS** unless `ncm_share` is on. The WiCAN reaches the dongle by its subnet route; its own default route and resolvers are untouched. The DHCP server restarts on every bus reset/re-plug (fixed 2026-08-23). |
| Time to enumerate after power-on | ~3 s (the firmware starts the OTG stack at ~3.4 s) |
| Boot window gotcha | The S3's **USB-Serial-JTAG** (VID `0x303A` PID **`0x1001`**, a CDC-ACM) is on the pads only while the ROM/bootloader run (< 1.5 s after power-on) — the app detaches it at `app_main` start, then the OTG stack takes the PHY at ~3.4 s. Still: **never open / toggle DTR-RTS on 303A:1001** (its line sequence resets the chip); ignore that PID and wait for `0x4007`. |
| After the cut | The dongle's DWC2 core raises no disconnect event (no VBUS sense) — it idles "configured" until the next bus reset. From the host side it is simply an unplug. |
| Restore | `POST /api/wifi_modem/usb_data {"enabled":true}` over the AP, or the dongle's UART console (`wifi_modem -d 1`), or a VBUS power-cycle. A PC re-enumerated in ~12 s; a WiCAN host should see it within its normal enumeration time. |
| NCM internet sharing | **Off by default** (`lte_upstream_pppos.ncm_share=false`): the USB DHCP offers no default router, so the link is a control channel only (192.168.7.1 reachable by subnet route) and a PC's internet never moves onto the LTE link; internet for the WiCAN comes over the AP. |

### 2.2 HTTP over USB (192.168.7.1) — all JSON

| Route | Answer |
|---|---|
| `GET /api/info` | `{"device_type":"espnetlink","model":"ESPNetLink","fw_version":"…","device_id":"206ef1894a5d","mac":"20:6E:F1:89:4A:5C","api_level":6}` — identify with `device_type`, key the pairing on `device_id` |
| `GET /api/wifi_modem/credentials` | `{"ssid":"ESPNetLink_894A5D","password":"<12 chars>","device_id":"206ef1894a5d","ap_started":true}` — **200 only when the request arrived over USB** (local socket address is neither the AP nor STA address); **403 over WiFi**. The password is the dongle's per-device random key (provisioned on its first boot; `0/O/1/l/I` never appear). |
| `POST /api/wifi_modem/usb_data` body `{"enabled":false}` | `{"usb_data":false,"applies_in_ms":500}` — the cut happens 500 ms after the response. `409 {"error":…}` when the dongle is in `usb_mode=ncm` (owner locked USB on — treat as "PC tethering mode", do not retry). `400` on a malformed body. |
| `GET /api/wifi_modem` | health: `usb_mode`, `usb_data` (live state of the lines), `napt`, `ap{ssid,ip,started,clients}`, `lte{valid,attached,connected,rssi_dbm,operator,network_type}`, `gps{valid,fix,satellites,age_ms}`, `uptime_s`, `device_id` — same document over USB and over the AP |
| `GET /api/gps` | full fix (`latitude`, `longitude`, `altitude_m`, `speed_kmph`, `course_deg`, `satellites`, `hdop`, `fix_quality`, `age_ms`, …) — unchanged, this is what `espnetlink_link` already parses |

### 2.3 The AP

| Item | Value |
|---|---|
| SSID | `ESPNetLink_<last 6 hex of device_id, upper>` e.g. `ESPNetLink_894A5D` — stable per device; channel 6 by default |
| Security | WPA2-PSK, the per-device key from `credentials` |
| Addressing | dongle `192.168.80.1/24`, DHCP to clients, DHCP-offered DNS = `192.168.80.1` (UDP/53 proxy to the PPP-learned resolver, 8.8.8.8 until LTE is up) |
| Internet | lwIP NAPT into the LTE link (Cat-M1: ~10–14 KB/s sustained); up ~40 s after dongle boot |
| ICMP | the AP address answers ping |
| First request after joining | occasionally returns an empty body while the client's link settles — retry once after ~2 s |
| `GET /api/wifi_modem/credentials` over the AP | 403 by design |

## 3. WiCAN state machine (`espnetlink_link`)

```
IDLE ──usb attach 303A:4007, NCM up──► IDENTIFY ──/api/info ok, device_type=espnetlink──► READ_KEY
  ▲                                         │ not an ESPNetLink / timeout 5 s                │ GET credentials
  │                                         ▼                                                ▼
  │                                       IDLE                                   STORE ── key/ssid/device_id unchanged ──► CUT
  │                                                                                  │ changed/new ─► persist fallback slot +
  │                                                                                  │   espnetlink settings (WiCAN reboot-to-apply)
  │                                                                                  ▼
  │                                                                                 CUT ── POST usb_data false ─► 200 ──► WAIT_DROP
  │                                                                                  │ 409 (ncm)            │ device gone ≤ 3 s
  │                                                                                  ▼                      ▼
  │                                                                            TETHER_MODE (leave USB on)  ON_AP ◄──── existing link task:
  │                                                                                                         join when needed, poll /api/gps
  └──────────────────── dongle re-enumerates (next power-on or VBUS cycle) ◄────────────────────────────────┘
```

Rules:

- **IDENTIFY** only on `0x303A:0x4007`. Ignore `0x303A:0x1001` (boot-window serial-JTAG) — do not bind CDC-ACM to it.
- **READ_KEY** every time the dongle enumerates; it's cheap and makes a swapped/reset dongle self-healing. Compare `device_id` + `ssid` + `password` with what is stored.
- **STORE** writes the WiCAN `wifi_manager` fallback slot (first free or the slot already holding this `ssid`, `trusted=true`) and `espnetlink_link.{ssid, device_id}` through `settings_manager` — the existing `espnetlink_link_pair()` does this. Those are reboot-to-apply on the WiCAN: a **new or changed** key means the WiCAN itself reboots once after the cut (the dongle does not). Unchanged key → no reboot, straight to CUT.
- **CUT** → **WAIT_DROP**: the NCM device must disappear within ~3 s. If it doesn't: retry the POST once; if still there after 10 s, cycle VBUS (`usb_host_manager` power control — today's `wusb power off/on`, GPIO10) and start over. Count and log.
- **TETHER_MODE** (409): the owner set `usb_mode=ncm` on the dongle. Leave the USB link alone; the stored key (if any) still works for the AP.
- **ON_AP** is the current `espnetlink_link` behaviour (home network preferred, dongle as fallback, `sta_roam_interval_s`, GPS + health polls).
- **Recovery when the AP join fails with the stored key** (dongle factory-reset, key rotated): cycle VBUS → the dongle re-enumerates → READ_KEY. Rate-limit to once per few minutes.
- Everything above runs on the WiCAN's own tasks; the dongle needs nothing timed from the WiCAN except the cut.

## 4. Work list

### 4.1 `meatpi-components/espnetlink_link` (exists — extend)

| File | Change |
|---|---|
| `espnetlink_link_usb.c` (new) | the state machine of §3: USB attach/detach hook → IDENTIFY → READ_KEY → STORE → CUT → WAIT_DROP; HTTP via `esp_http_client` to `192.168.7.1` (the NCM netif's subnet routes it, no default-route games); timers; retry/VBUS counters |
| `espnetlink_link_core.c/.h` | pure helpers (host-tested): credentials-document parse, "changed?" comparison, state transition table, URL build — keep the JSON/HTTP out of the state logic |
| `espnetlink_link_settings.c` | add `auto_pair` (bool, true), `device_id` (12 hex, ""), `cut_retries` (1–5, 2); keep `ssid`, `host`, `gps_poll_s`, `health_poll_s`, `cli`; bump schema version + `on_migrate` from v1 |
| `espnetlink_link.c` | on `ON_AP`, unchanged; expose `espnetlink_link_usb_attached()` / pairing state to the status document |
| `espnetlink_link_http.c` | `GET /api/espnetlink` gains `pair_state`, `usb_attached`, `device_id`, counters; `POST /api/espnetlink/pair` stays (manual path); add `POST /api/espnetlink/repair` = VBUS cycle + re-read |
| `espnetlink_link_cli.c` | `espnetlink` prints the pairing state; `espnetlink repair` |
| `host_test/` | state machine table, credentials parse, compare/changed logic, 409/timeout branches |
| `README.md`, `HTTP_API.md` route map, `cmdline_manager/README.md` | per the standard |

### 4.2 `meatpi-components/usb_host_manager`

| Change |
|---|
| Public VBUS control: `usb_host_manager_set_vbus(bool)` (GPIO10 `USB_OTG_PWR_EN`, active high) — today only reachable as the `wusb power` dev command |
| Attach/detach notification with VID/PID (callback or event) so `espnetlink_link` doesn't poll; include the NCM "link up + IP" moment |
| Do **not** bind the CDC-ACM class to `0x303A:0x1001` |

### 4.3 `wican-fw`

| Change |
|---|
| `main_glue`: wire the attach callback and the VBUS control into `espnetlink_link` (one glue, like `main_glue_wire_gps()` today) |
| Web UI: a "Dongle" card — pair state, SSID, device id, uplink (home/dongle/none), GPS age, `Re-pair` button (= `/api/espnetlink/repair`) |
| `wusb power` stays as the dev command |

### 4.4 Not needed

No changes to the WiCAN's USB NCM driver, to `wifi_manager`, or to the GPS sink path — the dongle speaks the same `/api/gps` and the same AP as before. No BLE, no QR, no label.

## 5. Bench test plan (this rig)

1. WiCAN on COM1175 (2 Mbaud console), dongle on COM1176 (UART0 115200), dongle USB-C into the WiCAN's host port, DIP in the run position. Dongle already stores `usb_mode=auto` and a provisioned key.
2. Power the WiCAN. Expected WiCAN log: attach 303A:4007 → `espnetlink: identified 206ef1894a5d` → `credentials ok` → `stored (slot N)` → `usb_data off` → `usb gone` (≤ 3 s) → (first time only) WiCAN reboot → `uplink=espnetlink` once the STA policy picks the dongle. Dongle UART0: `AP credentials handed to the USB host` → `host requested usb_data=0` → `USB data CUT at the mux` (within 10 s of its boot).
3. `wcat http://example.com/` on the WiCAN → 200 through the dongle. `espnetlink` → `paired=1 uplink=espnetlink gps: valid=1`.
4. Cold-boot loop: `wusb power off` / `on` ×10 — every cycle must end cut within 10 s, no WiCAN reboot (key unchanged), GPS fix age stays fresh.
5. Dongle factory reset (`POST /api/settings/factory_reset` over the AP before step 4 ends) → new key → next cycle: `credentials changed` → store → WiCAN reboot once → uplink back.
6. Owner lock: set `usb_mode=ncm` on the dongle → WiCAN must log `409 tether mode` and leave USB on; set back to `auto`.
7. Negative: a plain USB-serial device (CH343) on the port → no identify attempt beyond VID/PID filtering.

Until 4.1–4.3 land, the manual path works unchanged: `espnetlink pair <ssid> <password>` with the key from the dongle's `wifi_modem -k` (or `GET /api/wifi_modem/credentials` from a PC on the dongle's NCM).

## 6. Acceptance

- Fresh WiCAN + fresh dongle: internet via the dongle with zero user input; dongle boot → cut ≤ 10 s; one WiCAN reboot total.
- Replaced dongle or replaced WiCAN: same, no user input.
- 10 consecutive power cycles: 10 cuts, 0 reboots, 0 VBUS recoveries needed.
- Cold GPS TTFF with the WiCAN attached within the bench envelope of the power-only runs (45–121 s, median ~65 s aided).
- `GET /api/wifi_modem/credentials` never answers over WiFi (403).
