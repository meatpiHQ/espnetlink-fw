# wifi_manager — HTTP API reference

> **Implemented (2026-07-03)**: `wifi_manager_http.c`, registered via `wifi_manager_register_http()` (build-verified; live over-RF check = HIL S10). Unlike core components, `wifi_manager`
> is a **feature component** and registers these routes itself with
> `http_server_manager` at init (§9.1). Conventions:
> `components/HTTP_API.md` §1. Configuration is strictly via the generic
> `/api/settings/wifi_manager` — there are no bespoke config routes here.

## GET /api/wifi/status

Live connection state (wraps the component's status getters; cheap).

**Response 200**
```json
{
  "enabled": true,
  "sta_connected": true,
  "ip": "10.42.0.62",
  "ap_started": true,
  "clients": 1,
  "ap_ip": "192.168.80.1",
  "dns": ["10.42.0.1", "1.1.1.1"]
}
```
- `ip` is `""` while disconnected.
- `ap_ip` (2026-07-08) is the AP's live gateway address — reflects the
  configurable `ap_ip` setting; omitted when the AP interface has no
  address (AP off).
- `dns` entries read `"N/A"` when unavailable.
- The broader picture (overlap warning, time synced, …) lives in
  `GET /api/status` — this route is the WiFi panel's detail view.

## Configuration — `PUT /api/settings/wifi_manager` (v6)

No bespoke config routes; everything is configured through the generic
settings endpoint.

STA addressing (v6, 2026-07-11):

| Key | Type | Default | Notes |
|---|---|---|---|
| `sta_ip_mode` + `fallback1..5_ip_mode` | enum `dhcp`/`static` | `dhcp` | PER-NETWORK: each network carries its own addressing (applied per connect attempt). |
| `sta_static_ip` + `fallback1..5_static_ip` | string | `""` | REQUIRED when that network is static; dotted IPv4, host octet 1..254 (400 otherwise). |
| `sta_static_netmask` + `fallback1..5_static_netmask` | string | `255.255.255.0` | Must be a contiguous mask (400 otherwise). |
| `sta_static_gw` + `fallback1..5_static_gw` | string | `""` | Optional; validated as a host address when set. |
| `sta_dns` | string | `""` | Custom DNS override for BOTH ip modes. Empty = automatic (DHCP-provided, or the gateway when static). Re-asserted on every got-ip in DHCP mode. |

The AP LAN knobs (v4, 2026-07-08):

| Key | Type | Default | Notes |
|---|---|---|---|
| `ap_ip` | string | `192.168.80.1` | AP device/gateway IPv4; a `/24` is assumed and the DHCP pool follows it. Validated as a dotted quad with host octet 1..254 (rejects network/broadcast). Blank falls back to the legacy default. |
| `ap_hidden` | bool | `false` | Suppress SSID in beacons (clients must know the name). |
| `ap_bandwidth` | enum `ht20`/`ht40` | `ht20` | 40 MHz raises throughput at the cost of spectrum/robustness. |
| `ap_auth` | enum `auto`/`open`/`wpa2`/`wpa2wpa3`/`wpa3` | `auto` | `auto` = the historic rule (WPA2 with a password, OPEN without). WPA3/mixed enable PMF automatically. Any encrypted mode is rejected (400) when there is no `ap_password`. |

The WiFi memory profile (v5, 2026-07-08) — trades internal RAM for WiFi
throughput by sizing the driver buffers at `esp_wifi_init` (the counts
are `wifi_init_config_t` fields, so this is a **runtime** setting, no
recompile; reboot to apply):

| Key | Type | Default | Notes |
|---|---|---|---|
| `wifi_ram_profile` | enum `full`/`lean`/`custom` | `full` | `full` = IDF-default buffers. `lean` frees **~14.4 KB internal** (static RX 10→6, static TX 8→4, cache TX 32→16) at **no measured throughput cost** — the device's real traffic is app/TLS-capped (~900 KB/s) below WiFi's buffer-limited ceiling (bench A/B 2026-07-08). Use `lean` to run WiFi + BLE together (the internal-RAM cliff). `custom` = the three knobs below. |
| `wifi_static_rx` | int 2..25 | `10` | custom only — static RX buffers (~1.6 KB internal DMA each). |
| `wifi_static_tx` | int 1..64 | `8` | custom only — static TX buffers (~1.6 KB internal DMA each). |
| `wifi_cache_tx` | int 0..128 | `32` | custom only — cache TX buffers. Ranges match the IDF Kconfig bounds, so no combination fails `esp_wifi_init`; `rx_ba_win` and the dynamic (PSRAM) pools are left at their defaults. |

Existing AP keys (`ap_ssid`, `ap_password`, `ap_channel`,
`ap_max_connections`, `ap_auto_disable`) are unchanged. All of the above
is reboot-to-apply like every setting.

## GET /api/wifi/scan

Blocking scan (≈2 s — the UI shows a spinner; in AP-only mode the radio
briefly switches to APSTA and back). Response is
`wifi_manager_scan_networks()` verbatim.

**Response 200**
```json
{
  "networks": [
    {
      "ssid": "HomeAP",
      "rssi": -52,
      "channel": 6,
      "auth_mode": "WPA2_PSK",
      "bssid": "aa:bb:cc:dd:ee:ff"
    }
  ]
}
```
`auth_mode` ∈ `OPEN, WEP, WPA_PSK, WPA2_PSK, WPA_WPA2_PSK, WPA3_PSK,
WPA2_WPA3_PSK, UNKNOWN`.

**Errors**: `503 {"error":"scan unavailable"}` (mode `off` or start refused).
Concurrent scans are serialized internally; a second request simply waits.

## Settings slice (via the generic surface)

`GET/PUT /api/settings/wifi_manager` + `/schema` — see
`components/settings_manager/HTTP_API.md`. Reminder: `sta_password`,
`ap_password`, `fallbackN_password` are redacted to `""` in GETs; sending
`""` back keeps the stored secret.
