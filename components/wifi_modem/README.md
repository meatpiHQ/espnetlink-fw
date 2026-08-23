# wifi_modem — WiFi-modem mode (feature)

## Summary

Lets a WiCAN Pro (or any WiFi client) use the ESPNetLink as a **WiFi-LTE
modem + GPS source** with the USB-C cable carrying **5 V only**. The
WiCAN's USB host transmitter desenses the GNSS front-end while a USB-NCM
link is active (a cold fix never completes — bench evidence on the `gps-test-sandbox`
branch, `bench2_out/full_system/CH343_ROUTE_RESULTS.md`, 2026-08-22); cutting the USB data lines at the mux
and serving everything over the ESPNetLink's own AP removes the interferer
with no hardware change. The component owns:

- the `wifi_modem` settings (`usb_mode`, `ap_share`, the AP-password
  provisioning marker, `cli`) — reboot-to-apply;
- the **USB route**: at boot, `power_only` drives `USB_SEL_1/2 = 1/1`
  before any USB device stack could enumerate (`main` then skips
  `usb_dev_ethernet_start()`); at runtime (`auto`), the host on the cable
  cuts/restores the data lines through `POST /api/wifi_modem/usb_data` —
  an electrical unplug at the mux, no reboot, no stack teardown;
- the **pairing channel**: `GET /api/wifi_modem/credentials` hands the AP
  SSID/password to a request that arrived over USB (never over WiFi);
- **AP sharing**: lwIP NAPT on the AP netif into the default route (the PPP
  uplink), DHCP offering the AP address as resolver, and a UDP/53 proxy
  bound to the AP address forwarding to the PPP-learned DNS (clients that
  joined before the link came up keep resolving);
- **per-device AP password provisioning** (see below);
- `GET /api/wifi_modem` and the `wifi_modem` console command.

Sits in the services layer: below `main`, above `wifi_manager`,
`settings_manager`, `lte_upstream_pppos` (status only), `gps` (status only).

## API

| Call | Behavior |
|---|---|
| `wifi_modem_init()` | Log + settings descriptors. Between `settings_manager_init()` and `_start()`. |
| `wifi_modem_provision(&reboot)` | One-time AP password provisioning (after the settings apply pass). `reboot` true ⇒ main reboots once (`restart_tracker`, CONFIG_APPLY). |
| `wifi_modem_apply_usb_route()` | Drive the mux for the configured `usb_mode`. After the apply pass, BEFORE any USB stack start. |
| `wifi_modem_usb_data_enabled()` | Boot decision: `usb_mode != power_only` (or unconfigured ⇒ legacy behaviour). Gates `usb_dev_ethernet_start()` in main. |
| `wifi_modem_usb_data_set(bool)` | Runtime USB data switch: `false` drives the mux to 1/1 (electrical unplug — the device stack idles as with no host), `true` releases the pins to the DIP route (host re-enumerates). `ESP_ERR_NOT_SUPPORTED` in `ncm`. Idempotent. |
| `wifi_modem_usb_data_routed()` | Current state of the data lines. |
| `wifi_modem_register_http()` | `GET /api/wifi_modem`, `GET /api/wifi_modem/credentials`, `POST /api/wifi_modem/usb_data` (before `http_server_manager_start()`). |
| `wifi_modem_start()` | After `wifi_manager_start()`: arms NAPT + DNS offer + proxy. `ESP_ERR_INVALID_STATE` when unconfigured; ESP_OK/no-op when `ap_share` is off or the WiFi mode has no AP. |
| `wifi_modem_stop()` | NAPT off, proxy down, IP event handler gone. |
| `wifi_modem_settings_config()` / `_is_configured()` | Boot-applied config (§4.1 getters). |
| `wifi_modem_napt_active()`, `wifi_modem_upstream_dns()` | Status for the HTTP/CLI views. |
| `wifi_modem_settings_persist_str/bool()` | Persist one key (reboot to apply). |
| `wifi_modem_register_cli()` | Called internally on the settings apply behind `cli`. |

## Dependencies

All private (`PRIV_REQUIRES`): `settings_manager`, `log_manager`,
`wifi_manager` (AP status, mode), `hw_config` (USB_SEL pins),
`cmdline_manager`, `http_server_manager`, `gps` + `lte_upstream_pppos` +
`dev_status_manager` (read-only status for the health document),
`esp_driver_gpio`, `esp_netif`, `esp_event`, `lwip` (NAPT, sockets),
`esp_wifi`, `esp_timer`, `esp_hw_support` (`esp_random`), `console`,
`espressif__cjson`. Requires `CONFIG_LWIP_IP_FORWARD=y` + `CONFIG_LWIP_IPV4_NAPT=y`
(`sdkconfig.defaults`; NAPT depends on IP forwarding and Kconfig drops it
silently otherwise — the component logs `CONFIG_LWIP_IPV4_NAPT is off` and
AP clients get no internet).

Init order: `wifi_manager_init()` must be registered before this component
only in the sense that provisioning reads/writes the `wifi_manager`
settings object through `settings_manager` — no direct dependency on its
apply order. `wifi_modem_start()` needs `wifi_manager_start()` first (the
AP netif must exist).

## Settings (`"wifi_modem"`, version 1, field table)

| Key | Type | Default | Meaning |
|---|---|---|---|
| `usb_mode` | enum `auto`,`ncm`,`power_only` | `auto` | `auto`: boot with USB data on (NCM + CDC, today's PC behaviour) and let the host cut it at runtime — the WiCAN flow. `ncm`: USB data locked on (runtime cut refused with 409). `power_only`: cut at boot (charger-powered, AP only). |
| `ap_share` | bool | `true` | NAPT the AP netif into the LTE uplink + DNS proxy. |
| `ap_password_provisioned` | bool | `false` | Set once by the provisioning step. Never cleared by firmware; a factory reset starts over. |
| `cli` | bool | `true` | Register the `wifi_modem` console command at boot. |

**Why `auto` is the default.** The dongle stays stateless about who is on
the cable: it always boots with USB data on, so a PC gets tethering as
before and a WiCAN gets a point-to-point channel to read the AP key and
then cut the data lines — every boot, no pairing state, no button, no
reboot. A device that is power-cycled in front of a new WiCAN just gets
read again.

## AP password provisioning

wifi_manager's schema default is `12345678`; shipping that on a device
whose AP is its main interface would let anyone join. On the first boot of
this firmware, `wifi_modem_provision()`:

1. reads `wifi_manager.ap_password`;
2. if it is the stock value, generates a random
   `WIFI_MODEM_AP_PASSWORD_LEN` (12) character password from an
   unambiguous alphabet (no `0/O/1/l/I` — it gets typed from a label) with
   `esp_random()`, persists it into the `wifi_manager` settings and asks
   main for one reboot (reboot-to-apply, like the legacy-config import);
   a custom password is left alone;
3. persists `ap_password_provisioned = true` so this never runs again — an
   owner who later chooses the stock password deliberately keeps it.

The password is never logged (§10) and never appears in an HTTP document.
Read it on a console with **`wifi_modem -k`** (UART0 console when
`cmdline_manager.uart` is on; the USB CDC console in `ncm` mode) — that is
also how production reads it for the label. The web UI can set a new one
(`PUT /api/settings/wifi_manager` with a non-empty `ap_password`).

## HTTP API (route map: `components/HTTP_API.md`)

`GET /api/wifi_modem` — one read-only health document for the WiCAN
poller (no secrets):

```json
{"usb_mode":"power_only","ap_share":true,"napt":true,
 "ap":{"ssid":"ESPNetLink_894A5D","ip":"192.168.80.1","started":true,"clients":1},
 "lte":{"valid":true,"attached":true,"connected":true,"rssi_dbm":-51,
        "operator":"ALDI Mobile","network_type":"eMTC"},
 "gps":{"valid":true,"fix":true,"satellites":23,"age_ms":321},
 "uptime_s":162,"device_id":"206ef1894a5d"}
```

`usb_data` is the live state of the data lines. `lte` carries only
`valid:false` until the monitor has polled once; `rssi_dbm`/`operator`/
`network_type` appear when known. The full fix is `GET /api/gps`; settings
are `GET/PUT /api/settings/wifi_modem`.

`GET /api/wifi_modem/credentials` → `{"ssid","password","device_id",
"ap_started"}`. **Served only when the request did not arrive on a WiFi
interface** (the local socket address is neither the AP nor the STA
address) — i.e. over the USB link, which is point-to-point; otherwise
`403`. This is the WiCAN's pairing channel.

`POST /api/wifi_modem/usb_data` body `{"enabled":false|true}` →
`{"usb_data":bool,"applies_in_ms":500}`. The switch is applied 500 ms
after the response so the answer gets out over the very link being cut.
`409` when `usb_mode=ncm`. `enabled:false` = the cable becomes 5 V only
(the host sees an unplug); `enabled:true` = the host re-enumerates the
device. Bench note (2026-08-23): the DWC2 device core raises no
disconnect event when the lines are muxed away (no VBUS sensing), so the
stack just idles in its configured state until the host's next bus reset
— harmless; a PC re-enumerated ~12 s after the restore, and repeated
cut/restore cycles were clean.

## CLI (`wifi_modem`, ownership table: `cmdline_manager/README.md`)

| Invocation | Output |
|---|---|
| `wifi_modem` | `usb_mode=power_only ap_share=1 napt=1 ap_ip=192.168.80.1 clients=1 upstream_dns=101.168.244.101 mux SEL1=1 SEL2=1` |
| `wifi_modem -k` | `ap_ssid=ESPNetLink_894A5D ap_password=<12 chars>` |
| `wifi_modem -u auto\|ncm\|power_only` | persists `usb_mode` (reboot to apply) |
| `wifi_modem -a 0\|1` | persists `ap_share` (reboot to apply) |
| `wifi_modem -d 0\|1` | runtime USB data cut / restore (what the WiCAN does over HTTP) |

## WiCAN pairing (product flow, zero-touch)

Ship state: `usb_mode=auto`, per-device AP key already provisioned. Every
time the dongle boots on a WiCAN's USB port:

1. The dongle enumerates as NCM + CDC (USB data on). The WiCAN sees an
   ESPNetLink (`GET http://192.168.7.1/api/info` → `device_type=espnetlink`).
2. WiCAN: `GET /api/wifi_modem/credentials` over USB → stores/verifies
   `{ssid, password, device_id}` in its fallback slot.
3. WiCAN: `POST /api/wifi_modem/usb_data {"enabled":false}` → 500 ms later
   the data lines are cut; the cable is 5 V only, GPS is undisturbed. The
   WiCAN sees the USB device drop (expected) and joins the AP when it
   needs the uplink.

No pairing state on the dongle, nothing to un-pair: a new WiCAN reads the
key on the next power-on (the WiCAN can force one by cycling VBUS). A PC
on the cable never sends step 3; it keeps the link as a control channel
(web UI at 192.168.7.1) — the USB DHCP offers no default router unless
`lte_upstream_pppos.ncm_share` is on, so the PC's internet never drifts
onto the LTE link. Manual override: `wifi_modem -u ncm|power_only` /
settings PUT + reboot.

## Memory footprint (estimated — replace with measured before release)

| Where | What | Size |
|---|---|---|
| PSRAM `.bss` | share task stack (1 × 4096 `StackType_t` = 16 KiB, exits after arming) | 16 KiB |
| PSRAM `.bss` | DNS proxy task stack (1 × 4096 `StackType_t`) + 512 B packet buffer | 16.5 KiB |
| Internal `.bss` | 2 TCBs + ~40 B of state | ~0.8 KiB |
| Heap | one cJSON document per `/api/wifi_modem` request (PSRAM hooks), two UDP sockets | < 2 KiB transient |

Neither task writes flash (persistence is settings_manager's, executed in
the caller's context), so PSRAM stacks are compliant with §2. The
provisioning step runs in `app_main`'s context (internal stack).

## Tests

- `host_test/` — 14 Unity tests on the linux target: pure helpers
  (usb_mode mapping, runtime-switch policy, credentials gate,
  stock-password check, generator alphabet/bounds, resolver pick, IPv4
  format), the settings descriptor against the real
  settings_manager + storage stub (defaults, enum rejection,
  reboot-to-apply, `cli` gate), and the provisioning state machine
  (stock → generated + marker + reboot; custom → marker only; never twice;
  deliberate stock after the marker respected). In CI
  (`.github/workflows/host-tests.yml`) and `tools/run_host_tests.py wifi_modem`.
- `test_apps/` — pytest-embedded app on the main partition table +
  sdkconfig: boot pass, provisioning on real storage, NAPT arming on the
  real AP netif, stop, reboot-to-apply, and the mux drive/readback on the
  real `USB_SEL` pins, plus the runtime cut/restore readbacks and the
  `ncm` lock. Wipes the settings partition first. Device-run 2026-08-23
  (all markers in order); its first run caught the missing
  `CONFIG_LWIP_IP_FORWARD` in `sdkconfig.defaults`.
- Zero-touch flow bench (2026-08-23, PC as the WiCAN over NCM): `/api/info`
  → `credentials` 200 with the key → bad body 400 → `usb_data false` 200 →
  link gone; over the AP: `credentials` 403, `usb_data true` 200 → PC
  re-enumerated in ~12 s; second cut cycle clean. Soak (same day, after
  the USB DHCP-restart fix): 5 cut/restore cycles, AP answered every poll
  after each cut, NCM back with a fresh lease in 4 s each time. One
  unexplained AP loss was seen once earlier that day between test steps
  (not reproduced in a 47 s post-cut poll nor in the soak) — keep an eye
  on it during the WiCAN bench.
- Bench acceptance (2026-08-22, rpi001 `wtest1` as the WiCAN stand-in):
  join with the provisioned key → `GET /api/wifi_modem` 200 first try →
  HTTP 200 (0.7 s) / HTTPS 200 (0.5 s) through LTE → DNS via the proxy →
  50 KB ranged download (~10 KB/s, Cat-M1) → ICMP to the AP answers.
  Procedure + numbers: `docs/espnetlink_wifi_mode_howto.md`.
