# MeatPi Component Standard — ESPNetLink compliance status

Tracked against `MEATPI_COMPONENT_STANDARD.md` rev 1.0. Updated 2026-07-11
(refactor branch). ✅ done · ◐ partial · ✗ open.

## Core platform (§"Core platform components", §4, §6, §6b, §9)

| Item | Status | Notes |
|---|---|---|
| settings_manager (dedicated `settings` partition, CRC, atomic, dedup, reboot-to-apply) | ✅ | Ported from WiCAN v6; verified on device (PUT dedup, pending_reboot, apply-at-boot) |
| log_manager (vprintf hook, per-TAG table, sinks, PSRAM noinit ring) | ✅ | Ported; pipeline + ring live. Product boot default: level=info, console+ring sinks on |
| cmdline_manager (registry + dispatcher) | ◐ | Ported and started (UART console default OFF — ESPNetLink deviation). Legacy esp> console still owns the product commands; migration below |
| http_server_manager | ✅ | Ported; :80 live. IDF 5.5 note: WS pre-handshake gate is v6-only (frames still gated) |
| /api/settings HTTP API (§6) | ✅ | `api_http` port (settings surface). GET/PUT/schema/list/submit/backup/factory_reset verified on device over RNDIS |
| wifi_manager + dev_status_manager | ✅ | AP `ESPNetLink_<tail>` @192.168.80.1 verified; STA/fallbacks available via settings. dev_status events glue = no-op (no event_manager here) |
| HTTP route docs (components/HTTP_API.md central map) | ✗ | Ported components carry their per-component HTTP_API.md; central map not written yet |

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
| Host unit tests + CI | ✅* | `.github/workflows/host-tests.yml` builds + runs all 7 host_test suites (6 ported + new gps) on the IDF `linux` target (espressif/idf:v5.5.3 container) on every push/PR. *First green run pending a push (local smoke needs Docker Desktop started — daemon was down) |
| pytest test_apps on main sdkconfig/partitions | ✅ | All 6 test_apps repointed: inherit repo-root sdkconfig.defaults (which now pins the espnetlink partition table + 16MB flash) + minimal test-only overlay |
| Product-component test exemplar | ✅ | components/gps/host_test: gps settings descriptor against the REAL settings_manager registry + storage stub — schema defaults, range validation, reboot-to-apply semantics, §6b cli-gate (CLI register spies) |

## Known deviations (product addendum candidates)

- `cmdline_manager` `uart` setting defaults **false** (WiCAN: true) — UART0
  carries boot logs + optional GPS NMEA routing.
- `dev_status_manager` events glue is a no-op (no event_manager composed).
- `api_http` delayed reboot uses esp_restart() (restart_tracker not ported yet).
- IDF v5.5: bundled `json` for cJSON; WS pre-handshake gating unavailable.
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
