# MeatPi Component Standard — ESPNetLink compliance status

Tracked against `MEATPI_COMPONENT_STANDARD.md` rev 1.0. Updated 2026-07-12
(refactor branch). ✅ done · ◐ partial · ✗ open.

**2026-07-12: migrated to ESP-IDF v6.0.2** (same as the WiCAN v6 reference):
cJSON = `espressif/cjson` managed component, gpio/uart = split
`esp_driver_*` components, mbedTLS 4 / TF-PSA-Crypto (PSA hash in the CDC
OTA path), CI container = `espressif/idf:v6.0.2`. Verified same day:
**19/19 host suites** (locally in the CI image), **8/8 on-target test_apps**
on hardware, WireGuard **`VPN TARGET PASS`** over real LTE (which flushed
out and fixed a real data-plane bug — see
`vpn_manager/BUG_WG_NETIF_ADDR.md`), Tailscale **`TS TARGET PASS`** over
real LTE vs self-hosted Headscale.

## Core platform (§"Core platform components", §4, §6, §6b, §9)

| Item | Status | Notes |
|---|---|---|
| settings_manager (dedicated `settings` partition, CRC, atomic, dedup, reboot-to-apply) | ✅ | Ported from WiCAN v6; verified on device (PUT dedup, pending_reboot, apply-at-boot) |
| log_manager (vprintf hook, per-TAG table, sinks, PSRAM noinit ring) | ✅ | Ported; pipeline + ring live. Product boot default: level=info, console+ring sinks on |
| cmdline_manager (registry + dispatcher) | ◐ | Ported and started (UART console default OFF — ESPNetLink deviation). Legacy esp> console still owns the product commands; migration below |
| http_server_manager | ✅ | Ported; :80 live. WS pre-handshake gate active since the IDF v6 move (it is `#if >= 6.0.0`-gated in the component) |
| /api/settings HTTP API (§6) | ✅ | `api_http` port (settings surface). GET/PUT/schema/list/submit/backup/factory_reset verified on device over RNDIS |
| wifi_manager + dev_status_manager | ✅ | AP `ESPNetLink_<tail>` @192.168.80.1 verified; STA/fallbacks available via settings. dev_status events glue = no-op (no event_manager here) |
| restart_tracker (PSRAM-noinit reboot forensics; the ONE sanctioned reboot) | ✅ | Ported 2026-07-12; every reboot path routed through it (settings submit, factory reset, OTA apply, CLI reboot/wake, /api/restart). Device-verified: unplanned→user_request→config_apply→ota_apply records across real reboots |
| api_http system/diag/ota areas (/api/status, /api/status/tasks, /api/info, /api/restart*, /api/logs/*, /api/ota/*) | ✅ | Verbatim WiCAN ports; all device-verified over RNDIS. /api/info = HA identity route (device_type espnetlink) |
| ota_manager + multipart_upload (HTTP firmware update) | ✅ | POST /api/ota/upload (raw + multipart) + legacy /upload/ota.bin alias; end-to-end verified: curl upload → ota partition flip → tracked ota_apply reboot |
| mdns_manager (`_meatpi._tcp`, TXT mac/device_id/device_type=espnetlink, hostname espnetlink_<id>) | ✅ | Advertising verified in device log. Caveat: esp-mdns binds the WiFi netifs only — the raw-lwip RNDIS link carries no mDNS (HA discovers over WiFi; RNDIS/manual-IP always works) |
| ha_webhooks (HA telemetry: /api/webhook + poster) | ✅ | ESPNetLink payload: status{identity, LTE fields, wifi_mode, vpn_*} + gps{} (never diffed). Bench-verified against a local fake HA: 201 registration, 5 s pushes, diff mode with identity overlay, gzip (415B vs 915B), 204 handling |
| http_client_manager | ✅ | Ported; cert_manager sets unsupported (bundle-only TLS), download-to-file = ESP_ERR_NOT_SUPPORTED (no consumer) |
| web_ui (browser settings/status/system UI at `/`) | ✅ | WiCAN v6 layout, ESPNetLink branding; nav pruned to Status/Settings/System/About; status page reads /api/lte + /api/gps + /api/vpn + /api/imu; gzip asset serving verified |
| vpn_manager (WireGuard + Tailscale) + vendored esp_wireguard/wireguard_lwip/microlink | ✅ | **Both tunnels bench-PASSED over real LTE (2026-07-12):** WireGuard `VPN TARGET PASS` (vpn_bench_target.py vs the bench wg server — found+fixed the netif-address data-plane bug, BUG_WG_NETIF_ADDR.md; WiCAN carries the same bug) and Tailscale `TS TARGET PASS` (ts_bench_target.py vs self-hosted Headscale: register → 100.64.0.1 → peer → ICMP 0% + HTTP 200 through the tunnel). microlink is byte-parity with WiCAN v6 again (the interim mbedtls-3.6 bridges were removed with the v6 move) |
| imu_manager + i2c_bus (replaces the legacy imu component) | ✅ | SMD/WoM detectors, /api/imu (bench: gravity on Z, die temp), motion dev-status bit, `imu` CLI; ESPNetLink extension imu_manager_wom.c keeps `sleep 4` wake-on-motion working |
| filesystem (FULL WiCAN core, replaces the shim) | ✅ | /data mount on the `storage` littlefs partition (same flash bytes, new VFS prefix), atomic writes, streams/async pipe, path validation, `fs` CLI + {cli} settings, /api/fs file manager. Device-verified: list/info/upload/delete + legacy config.json.imported visible at /data |
| cert_manager (TLS certificate sets) | ✅ | /data/certs storage (WiCAN-verbatim), /api/certs surface device-verified (upload PEM → list → delete); http_client_manager cert_set TLS path re-enabled (was bundle-only), download() restored |
| HTTP route docs (components/HTTP_API.md central map) | ✅ | Central map current (settings/system/diag/ota/webhook/wifi/lte/gps/vpn/imu + UI) |

## §2 Memory

| Item | Status | Notes |
|---|---|---|
| PSRAM enabled (octal, bss/stack/noinit external, memtest off) | ✅ | Verified on device (8 MB pool) |
| Task stacks static in PSRAM (xTaskCreateStatic + EXT_RAM_BSS_ATTR) | ✅ | All long-lived singletons converted + verified (gps_parser, lte_monitor/sup, usbnet rx/evt/dns, main lte task). Deliberately internal/dynamic: usb_cli console (OTA flash writes, §2 corollary), agnss tasks (NVS flash ops), transient per-run tasks (speedtest, deep_sleep, lte_tsync — respawnable) |
| cJSON heap → PSRAM hooks in main | ✅ | WiCAN pattern; verified running |

## §3 / §10 Component rules

| Item | Status | Notes |
|---|---|---|
| No panic paths (ESP_ERROR_CHECK) in components/main | ✅ | modem_manager ×7, usb_dev_bridge ×1, main ×2 converted to log-and-degrade. Remaining: none in active code (dead `usb_dev_ethernet.c` NCM file excluded from build) |
| main = composition root (init_step/start_component) | ◐ | New core composed per standard order; legacy inits kept as-is; USB_HOST/DEV dead modes still in main.c |
| TAG == component dir (§10) | ✅ | Normalized across all components (gps submodule agnss → "gps") |
| Per-component log_descriptor_t registration | ◐ | Ported components register; legacy components rely on default level until their init migration |
| Lifecycle init/start/stop everywhere, no _deinit | ✗ | Legacy components keep ad-hoc lifecycles until migration |

## §4/§5 Settings migration (legacy config_manager → settings_manager)

| Item | Status | Notes |
|---|---|---|
| config_manager retired; keys per component | ✅ | Descriptors: `lte_upstream_pppos` (enabled/apn/apn_user/apn_password/apn_pin/ncm_share/cli), `gps` (enabled/passthru/uart0_out/agnss_enabled/agnss_cache_s/cli), `system` (device_name). All call sites migrated (gps.c, agnss.c, agnss_console.c, usb_cli_console.c, main lte task). Component dir left in tree for `git rm` review |
| Legacy /storage/config.json one-time import | ✅ | VERIFIED 2026-07-12 with a true field-upgrade simulation (old build → plant values → OTA new → values carried, one-time reboot). Bench-hit fixed en route: legacy file wraps keys in a `"config"` object — the first importer looked at the root and silently imported nothing |
| `config` CLI byte-parity over settings_manager | ✅ | main_cli.c key-mapped view; verified on device: list / list -j / -l / get / set / reset formats identical; set is §4.2 reboot-to-apply (matches legacy read-at-boot semantics) |
| CLI↔HTTP unification | ✅ | Verified: `config set NCM_SHARE false` → GET /api/settings/lte_upstream_pppos shows ncm_share:false + pending_reboot:true; reboot applies (boot log share_usb=no) |
| AGNSS NVS cache (namespace "gnss") | ✅ n/a | Runtime cache, not settings — stays in NVS per standard |

## §6b CLI

| Item | Status | Notes |
|---|---|---|
| Byte-identical commands/responses (`esp> `, OK/ERROR trailers) | ✅ | Verified after every move (ver/echo/config/gps/agnss/system/lte sweeps) |
| Commands register via cmdline_manager, owned per component, `cli` bool gate | ✅ | gps/agnss/lte register on their settings apply; imu + usb_cli_console got minimal {cli} descriptors; echo+ota live in usb_cli_console_cli.c/usb_cli_ota.c; composites (ver/debug/system/reboot/sleep/wake/ping/speedtest/config) moved to main/main_cli*.c; usb_cli_commands.c deleted |
| cmdline_manager ownership table | ✅ | components/cmdline_manager/README.md (espnetlink table) |
| Logs leaking into CDC responses | ✅ | log_manager console sink now writes /dev/console (UART0) directly instead of the CDC-redirected stdout |
| New: `ota` command | ✅ | usb_cli_ota.c + tools/cdc_ota.py; documented in README §2.1 |

## §1 Style / §8 packaging

| Item | Status | Notes |
|---|---|---|
| ≤700-line files | ✅* | All split by responsibility (verbatim byte-sliced moves, verified on device): gps.c 613 + gps_cli.c; modem_manager.c 550 + _probe.c 642; lte_upstream_pppos.c 696 + _cli.c 619 + _monitor.c 515 + _at.c 320. *Documented exception: usb_cli_console.c stays at 858 — the console task, line editor, GPS-stream escape state and TX ring share 9 statics with no clean responsibility seam (§1: "never split mid-logic just to hit the number") |
| Allman everywhere | ✅ | usb_cdc_host_manager.cpp + usb_dev_bridge.c reformatted with the new repo-root .clang-format (Allman, 4-space, 100 col) |
| README.md per component | ✅ | All 11 legacy/product components + all ported components |
| idf_component.yml per component | ✅ | Every component declares its own managed deps (main's manifest retained) |
| Stale WiCAN headers (modem_manager, usb_cdc_host_manager) | ✅ | Now say ESPNetLink |
| Dead code | ✅ | usb_dev_descriptors stub files + usb_dev_ethernet.c (never-built NCM variant) deleted |

## §7 Tests

| Item | Status | Notes |
|---|---|---|
| Host unit tests + CI | ✅* | `.github/workflows/host-tests.yml` builds + runs all **19** host_test suites on the IDF `linux` target (**espressif/idf:v6.0.2** container) on every push/PR — all 19 verified PASSING locally in the same image (2026-07-12, re-run after the v6 migration). Suites `exit(UNITY_END())` so a failing test fails the job. *First CI run pending a push |
| pytest test_apps on main sdkconfig/partitions | ✅ | 8 test_apps: 5 ported cores + restart_tracker + filesystem + usb_dev_ethernet smokes. **All 8 PASS on hardware on IDF v6 (2026-07-12**, marker-verified over the UART; wifi suite needs a virgin settings partition — see its pytest note). All inherit repo-root sdkconfig.defaults + minimal overlay |

### Per-component coverage map

"Anything that can be host-tested must be" — the pure logic in every
component is either under host test or the component has none. Hardware-bound
behavior is bench-verified today; on-target pytest apps are the backlog.

| Component | host_test | What it covers / why not |
|---|---|---|
| settings_manager | ✅ | Registry, codec, schema, backup (ported with component) |
| log_manager | ✅ | Level table / pipeline logic (ported) |
| cmdline_manager | ✅ | Registry + dispatcher (ported) |
| http_server_manager | ✅ | Host-testable core (ported) |
| dev_status_manager | ✅ | dev_status_manager_fmt.c formatting (ported) |
| wifi_manager | ✅ | wifi_manager_select.c AP/STA selection (ported) |
| gps | ✅ | Settings descriptor: defaults, agnss_cache_s range, reboot-to-apply, §6b cli gate. NMEA/AGNSS engine is UART/NVS-bound → bench + backlog |
| lte_upstream_pppos | ✅ | Settings descriptor: defaults (incl. **ncm_share=true product default**), apn maxLength, reboot-to-apply, cli gate. PPP/CMUX engine is modem-bound → bench + backlog |
| modem_manager | ✅ | mm_parse_ifc/mm_parse_ipr AT-response parsers (extracted to modem_manager_parse.c). Power/probe ladder is UART-bound → bench + backlog |
| api_http | ✅ | api_http_util.c: secret redact/unredact, /api/settings URI parse, level map. Handlers are esp_http_server-bound, verified on device over RNDIS |
| imu_manager (replaces imu) | ✅ | WiCAN suite: policy state machine + settings migration (host) + on-device /api/imu + `imu` CLI bench-verified. WoM deep-sleep extension is hardware-bound |
| i2c_bus | — | Thin bus owner (one init + add_device wrapper); exercised by imu_manager on every boot |
| restart_tracker | ✅ | WiCAN suite: pure core (CRC envelope, ring, planned intent, MSPI-guard) — 8 host tests + test_app |
| ota_manager | ✅ | WiCAN suite: pure session state machine (flash ops injected). HTTP path device-verified end-to-end |
| mdns_manager | ✅ | WiCAN suite (hostname builders adapted to espnetlink_<id>) |
| http_client_manager | ✅ | WiCAN suite: pure auth/url/base64 helpers |
| vpn_manager | ✅ | WiCAN suite (pure config/state logic). Both tunnel types bench-PASSED over real LTE 2026-07-12 (`VPN TARGET PASS` + `TS TARGET PASS`) |
| ha_webhooks / multipart_upload / web_ui | — | Transport/UI glue; bench-verified live (fake-HA pushes incl. gzip + diff mode; multipart + raw OTA uploads; UI serving). esp_wireguard/wireguard_lwip/microlink are vendored engines (upstream provenance documented) |
| usb_cli_console | ✅ | {cli} descriptor gate. Console core/OTA path is USB-bound — exercised on every bench OTA/CLI session |
| filesystem | ✅ | WiCAN suites: 10 pure path-routing host tests + the full 13-marker on-target test_app (mount/atomic/list/validation/remount) |
| cert_manager | ✅ | WiCAN suite: 6 pure policy host tests. HTTP surface bench-verified |
| hw_config | — | Header-only pin map; nothing executable |
| usb_dev_ethernet | — | NEW test_apps smoke (stack up + 5 s stability); full RNDIS behavior is host-side — bench-verified daily (DHCP lease, NAPT, OTA-over-RNDIS) |
| usb_cdc_host_manager, usb_dev_bridge | — | Legacy alternate modes (not active in product boot); HIL backlog if revived |
| main units (config CLI key map, legacy import) | — | Verified end-to-end on device (byte-parity sweeps + old→new upgrade simulation); host-testing main/* is future work |

## Port deviations (2026-07-12 WiCAN feature-set port)

- `ha_webhooks` payload: `autopid_data`/`config` sections replaced by `gps`
  (never diffed — the HA tracker needs lat+lon together); `status` carries
  `lte_*` fields; poster gates on LTE-PPP-or-WiFi (the LTE uplink has no
  dev-status bit).
- `vpn_manager`: rtc_manager absent → clock validity = `time(NULL)` sanity
  floor (LTE NITZ feeds settimeofday); event_manager glue no-op'd.
- `microlink` (Tailscale engine): byte-parity with the WiCAN v6 copy (the
  interim mbedtls-3.6 bridges were removed with the IDF v6 move; see its
  PROVENANCE.md, ESPNetLink port section).
- `http_client_manager`: cert_manager sets unsupported (esp-tls bundle
  only); `download()` = ESP_ERR_NOT_SUPPORTED.
- `imu_manager`: event_manager glue excluded; ESPNetLink gained
  `imu_manager_wom.c` (`sleep 4` wake-on-motion arm — kept from legacy imu).
- Ported `*_cli.c` internal `OK` trailers stripped — on this product the
  CDC console owns the OK/ERROR trailer (legacy parity contract).
- mDNS: `_meatpi._tcp` only (no `_wican._tcp`); esp-mdns rides the WiFi
  netifs — the raw-lwip RNDIS link carries no mDNS by design.
- Identity Kconfig (`CONFIG_WICAN_DEVICE_TYPE`= espnetlink etc.) keeps the
  WICAN_* symbol names for cross-component source compatibility.

## Known deviations (product addendum candidates)

- `cmdline_manager` `uart` setting defaults **false** (WiCAN: true) — UART0
  carries boot logs + optional GPS NMEA routing.
- `dev_status_manager` events glue is a no-op (no event_manager composed).
- `api_http` delayed reboot uses esp_restart() (restart_tracker not ported yet).
- AGNSS: this LC76G firmware build carries **EPOC, not EASY** (Quectel
  AGNSS app note: one build has one of the two; PAIR491 → `$PAIR001,491,3`
  = unsupported). agnss.c probes EASY → falls through to EPOC (PAIR508) —
  bench module reports `EPOC enabled=1 constellation=0x1` (GPS-only
  prediction). PAIR511 nav-data saves are shared by both. Future option:
  Host EPO over LTE (the module requests it via `$PAIR010,0,<gnss>`;
  needs a Quectel `vendorinfo` download credential).
- Boot log default changed from silent (`esp_log_level_set("*", NONE)`) to
  INFO via log_manager settings — `debug 0/1` CLI still toggles globally, and
  `PUT /api/settings/log_manager {"level":"none"}` restores silence persistently.
- **Open parity issue:** stdout is globally redirected to the CDC console, so
  INFO logs emitted while a command runs interleave into CLI responses.
  lte_upstream_pppos's 15 s monitor loop was demoted to DEBUG (§10), which
  removes the common case; the systemic fix (log_manager console sink writes
  the UART directly, not stdout) belongs to the CLI-migration phase.

## Verified on hardware (2026-07-11 bench session)

- Build/flash loop: CLI build + CDC OTA (`tools/cdc_ota.py`, ~10 s for 1.5 MB)
  + UART chunk-flash recovery.
- PSRAM 8 MB octal up; settings partition formatted + defaults persisted.
- WiFi AP `ESPNetLink_894A5D` @192.168.80.1; `/api/wifi/status` + full
  `/api/settings` cycle (PUT dedup, no-op submit, reboot-to-apply,
  pending_reboot) over RNDIS.
- RNDIS DHCP (per-netif dhcps) leases to Windows alongside the AP's server.
- LTE end-to-end with SIM: BG95 registered (eMTC, ALDI Mobile), PPP up in
  ~2 s warm, `ping 8.8.8.8` 4/4 at 56–97 ms — with `NCM_SHARE=false`
  (bench policy: sharing stays off unless explicitly under test).
- Regression fixed en route: esp_modem DTE "UART driver malloc error" — the
  32K/16K INTERNAL UART rings no longer fit beside esp_wifi; now 16K/8K.
