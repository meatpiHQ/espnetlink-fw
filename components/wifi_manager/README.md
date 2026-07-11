# wifi_manager

## Summary

Feature component owning the WiFi radio. Rewrite of the legacy WiCAN WiFi manager
against Coding Standard rev 2.1: same field-proven connection logic — STA with up
to 5 prioritized fallback networks, scan-based candidate selection, a per-SSID
auth-failure ban list (3 fails → 10 min; a ban means "prefer anything else",
never "stop trying" — when the banned network is the only option it is still
retried once per minute: the same SSID name can carry a different password at
another location and the device must reconnect promptly back home, meatpi
2026-07-08), blind sequential fallback when nothing visible matches (hidden
SSIDs + dense-airspace scan truncation), roam-to-preferred (while connected to
a fallback, periodically re-scan and migrate to a higher-priority network —
"home beats the car hotspot"), strongest-BSSID association when one SSID has
several APs (WIFI_ALL_CHANNEL_SCAN + WIFI_CONNECT_AP_BY_SIGNAL), a supervised
reconnect task that pauses while AP clients are attached (escalating
retry backoff since 2026-07-10: the first two attempts keep the 5 s
cadence for a fast candidate walk, then 10/20 s doubling to a 30 s cap
so a parked device doesn't scan every 5 s forever — resets on got-ip;
pure `wm_backoff_skip_loops`, host-tested), AP with static IP `192.168.80.1/24` + DHCP server (the legacy WiCAN address — apps hardcode it; fixed 2026-07-04 after the rewrite briefly used 192.168.0.10, which collides with common home LANs),
APSTA with AP-auto-disable and AP-channel-follows-STA, subnet-overlap warning,
backup-DNS fallback to 1.1.1.1 — but **all configuration is a settings_manager
descriptor and applies only at boot** (reboot-to-apply, §4.2). The legacy runtime
APIs (`set_mode`, `set_sta_config`, `enable`/`disable`, `deinit`, …) are gone by
design; that removed roughly a third of the old code (mode-change/teardown paths).

Recovery when STA can't connect: the device does NOT auto-open its AP
(a deauth attacker could force it open, meatpi 2026-07-08). The user
forces AP mode with a hardware button long-press (legacy behavior; owned
by the future button/input manager).

## API

| Function | One-liner |
|---|---|
| `wifi_manager_init()` | Allocate state; register the `"wifi_manager"` settings descriptor. |
| `wifi_manager_start()` | Radio up per boot-applied settings; `ESP_ERR_INVALID_STATE` if unconfigured; ESP_OK with radio down when mode is `off`. |
| `wifi_manager_stop()` | Stop reconnect task + radio. |
| `wifi_manager_is_enabled/is_sta_connected/is_ap_started()` | Status flags (event-bit backed). |
| `wifi_manager_get_sta_ip(buf,len)` | Current STA IPv4 ("" if none) — caller buffer, no shared statics. |
| `wifi_manager_get_ap_station_count()` | Associated AP clients. |
| `wifi_manager_get_sta_dns(...)` | Main/backup DNS as strings ("N/A" fallback). |
| `wifi_manager_get_event_group()` | Event group with `WIFI_MANAGER_BIT_*` for waiters. |
| `wifi_manager_scan_networks()` | Blocking scan → malloc'd JSON (caller frees). |
| `wifi_manager_set_callbacks(cbs)` | STA up/down + AP client join/leave hooks (event-loop context). |

## Dependencies

- `settings_manager` (private) — configuration descriptor; **init order:**
  `settings_manager_init()` → `wifi_manager_init()` → `settings_manager_start()`
  (runs `on_apply`) → `wifi_manager_start()`.
- `esp_wifi`, `esp_netif`, `esp_event` (private); `espressif/cjson` (managed,
  private) for scan JSON.
- `main` must run `nvs_flash_init()` before `wifi_manager_start()` (esp_wifi
  requirement). WiFi credential storage is RAM-only (`WIFI_STORAGE_RAM`) — the
  settings partition is the single source of truth.
- `log_manager` (private) — registers `{"wifi_manager", INFO}` at init for
  per-TAG runtime levels (§9.2). Reconnect attempts and candidate selection
  log at DEBUG (§10 hot-path rule) — raise the level at runtime to see them.
- `dev_status_manager` (private) — publishes `DEV_STATUS_BIT_STA_CONNECTED`
  (got-IP / disconnect), `DEV_STATUS_BIT_AP_ENABLED` (AP start/stop) and
  `DEV_STATUS_BIT_STA_AP_OVERLAP` (subnet clash, cleared with either side)
  into the ONE device-status group; all three cleared on `stop()`. Safe if
  dev_status_manager was never inited (its setters are no-ops then).

## Settings (`"wifi_manager"`, version 6)

`cli` (bool, default true): register this component's console command(s) with cmdline_manager on the settings boot apply (reboot-to-apply). Ownership: the component registers its own commands — main wires nothing (2026-07-05).

| Key | Type | Default | Notes |
|---|---|---|---|
| `mode` | enum `off/sta/ap/apsta` | `apsta` | |
| `sta_ssid` / `sta_password` | string ≤32 / ≤64 | `""` | empty ssid = no STA network |
| `fallback1..5_ssid` / `_password` | string | `""` | priority order; flat keys (schema subset has no arrays) |
| `hostname` | string ≤32 | `""` | |
| `sta_auto_reconnect` | bool | `true` | |
| `sta_max_retry` | int −1..1000 | `-1` | −1 = infinite; limit hits trigger a 1-min cooldown |
| `sta_ip_mode` / `fallback1..5_ip_mode` | enum `dhcp/static` | `dhcp` | v6: PER-NETWORK STA addressing — each network (primary + every fallback) carries its own choice, since they live on different LANs. Applied per CONNECT ATTEMPT (`apply_sta_network`): DHCP client stopped + `esp_netif_set_ip_info` for a static candidate, restarted (stale address cleared) for a DHCP one; got-ip still fires so the reconnect/roam machinery is unchanged. Bench-verified 2026-07-11 (static reachable from the LAN, DHCP restore clean) |
| `sta_static_ip` / `fallback1..5_static_ip` | string (IPv4) | `""` | v6: REQUIRED when that network is static (validated: dotted quad, host octet 1..254) |
| `sta_static_netmask` / `fallback1..5_static_netmask` | string (IPv4) | `255.255.255.0` | v6: must be a contiguous mask (`wm_netmask_valid`, host-tested) |
| `sta_static_gw` / `fallback1..5_static_gw` | string (IPv4) | `""` | v6: optional (isolated-LAN use); validated as a host address when set |
| `sta_dns` | string (IPv4) | `""` | v6: GLOBAL custom DNS override for BOTH ip modes (Pi-hole/1.1.1.1 case). Empty = automatic: the DHCP-provided server, or the connected network's gateway when static. In DHCP mode the override is re-asserted on every got-ip (the lease overwrites MAIN DNS); the 1.1.1.1 backup-DNS resilience still applies |
| `ap_ssid` | string ≤32 | `""` | empty → derived `WiCAN_xxxxxx` from MAC |
| `ap_password` | string 8..64 | `12345678` | key material; auth mode set by `ap_auth` |
| `ap_auth` | enum auto/open/wpa2/wpa2wpa3/wpa3 | `auto` | v4: `auto` = WPA2 with a password / OPEN without (historic). WPA3 + mixed enable PMF; an encrypted mode with no password is rejected |
| `ap_ip` | string (IPv4) | `192.168.80.1` | v4: AP gateway address; `/24` assumed, DHCP pool follows it. Validated dotted-quad, host octet 1..254. Live value echoed in `/api/wifi/status` `ap_ip`. The legacy default dodges the common 192.168.0/1.x home LANs |
| `ap_channel` | int 1..13 | `6` | |
| `ap_bandwidth` | enum ht20/ht40 | `ht20` | v4: 40 MHz for throughput vs 20 MHz for robustness |
| `ap_max_connections` | int 1..10 | `4` | |
| `ap_hidden` | bool | `false` | v4: don't beacon the SSID |
| `ap_auto_disable` | bool | `false` | drop AP once STA has an IP; back on STA loss |
| `sta_roam_interval_s` | int 0..86400 | `300` | v2: while connected to a FALLBACK, re-scan this often and migrate when a higher-priority network is visible; 0 = never roam |
| `wifi_ram_profile` | enum full/lean/custom | `full` | v5: trades internal RAM for WiFi throughput at boot. `full` = IDF default buffers. `lean` frees **~14.4 KB internal** (static RX 10→6, static TX 8→4, cache TX 32→16) at **no measured throughput cost** for the device's real traffic (MQTT/HTTP are app-capped below WiFi's ceiling; bench 2026-07-08). `custom` = the three knobs below. Runtime — the counts are `wifi_init_config_t` fields applied at `esp_wifi_init`; reboot to take effect. Use `lean` to run WiFi + BLE together (the internal-RAM cliff). |
| `wifi_static_rx` | int 2..25 | `10` | custom only: static RX buffers (each ~1.6 KB internal DMA) |
| `wifi_static_tx` | int 1..64 | `8` | custom only: static TX buffers (each ~1.6 KB internal DMA) |
| `wifi_cache_tx` | int 0..128 | `32` | custom only: cache TX buffers. Ranges match the IDF Kconfig bounds so no combination is invalid (rx_ba_win + dynamic pools are left at defaults / PSRAM) |
| `sta_trusted` / `fallback1..5_trusted` | bool | `true` | v3 network-trust lockdown: when the CURRENT STA network is untrusted, ALL inbound admin surfaces (every /api route, the UI catch-all, every /ws channel) that arrive via the STA address return 403 / refuse the handshake. The device's own AP + the USB link are always non-STA and keep full admin (the recovery path). Outbound clients — autopid HTTP posts, MQTT — don't traverse httpd and are unaffected. Enforced by http_server_manager's per-request gate (a trampoline wrapping every route incl. WS pre-handshake); wired by main to `wifi_manager_http_request_allowed()`. Fail-closed if the request can't be classified. Use case: configure on your home network, keep the device muted on shared/office WiFi. |
| `power_save` | enum `none/min/max` | `none` | |

`on_validate` (cross-field): rejects `sta_password` set with empty
`sta_ssid`; static mode without a valid `sta_static_ip`; malformed
netmask/gateway/DNS addresses.

## HTTP API (requirement — full endpoint reference: `HTTP_API.md` in this directory; conventions: `components/HTTP_API.md`)

As a **feature component** it registers its own routes with
`http_server_manager` at init (unlike core components, which go through the
`api_http` glue): `GET /api/wifi/status` (wraps the status getters) and
`GET /api/wifi/scan` (the scan JSON, blocking ≈2 s). Configuration strictly
via the generic `/api/settings/wifi_manager` — no bespoke config routes.
Passwords are redacted in settings GETs (HTTP_API.md §1).

## Memory footprint (estimated — replace with measured before release)

| Where | What | Size |
|---|---|---|
| PSRAM `.bss` | config + status + select state + scan scratch + callbacks | ~6 KiB |
| PSRAM `.bss` | reconnect task stack (1 × 16 KiB `StackType_t[4096]`) | 16 KiB |
| Internal `.bss` | event group + mutex + TCB | ~0.5 KiB |
| Heap | esp_wifi/lwip internals (IDF-owned) | per IDF config |

The reconnect task performs no flash writes (persistence is settings_manager's),
so a PSRAM stack is compliant with §2.

## Tests

- **Host (`host_test/`):** the pure selection/ban module — priority order,
  fallback pick, ban after threshold, ban expiry, success clears, all-banned
  override, sequential rotation.
- **On-target (`test_apps/`):** composition boot (settings apply → start), AP
  comes up with derived SSID, scan returns JSON, invalid settings rejected via
  `settings_manager_set`, no-apply-at-runtime. Builds against the main partition
  table (rev 2.1 §7). A full STA-association test needs a live AP —
  hardware-in-the-loop follow-up per §7 (README with bench setup) when added.

## CLI

`wifi_manager_register_cli()` (main, CLI builds) registers the `wifi [scan]` command with cmdline_manager (`wifi_manager_cli.c`).
