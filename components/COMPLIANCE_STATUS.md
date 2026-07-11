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
| Task stacks static in PSRAM (xTaskCreateStatic + EXT_RAM_BSS_ATTR) | ✗ | 25+ legacy dynamic internal-RAM tasks to convert (ported core components already comply) |
| cJSON heap → PSRAM hooks in main | ✗ | WiCAN main pattern; adopt with main cleanup |

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
| config_manager retired; keys per component | ✗ | Mapping fixed: lte_upstream_pppos ← APN/APN_USER/APN_PASS/APN_PIN/LTE_ENABLED/NCM_SHARE; gps ← GPS_PASSTHRU/GPS_UART0_OUT/AGNSS_ENABLED/AGNSS_CACHE_S; DEVICE_NAME/GPS_ENABLED unused (drop) |
| Legacy /storage/config.json one-time import | ✗ | Field devices exist — import → settings_manager_set per component → rename marker → single reboot |
| `config` CLI byte-parity over settings_manager | ✗ | Contract captured (list/-l [-j], get KEY, set KEY VAL, reset; exact formats in config_manager_console.c) |
| AGNSS NVS cache (namespace "gnss") | ✅ n/a | Runtime cache, not settings — stays in NVS per standard |

## §6b CLI

| Item | Status | Notes |
|---|---|---|
| Byte-identical commands/responses (`esp> `, OK/ERROR trailers) | ✅ | Preserved — no behavior change shipped |
| Commands move to `<comp>_cli.c` + cmdline_manager_register, `cli` bool gate | ✗ | Currently central in usb_cli_console (+ per-comp console_register fns). Ported components' own CLIs (wifi, dev_status) already comply |
| cmdline_manager ownership table | ✗ | With the CLI migration |
| New: `ota` command | ✅ | usb_cli_ota.c + tools/cdc_ota.py (documented in PRODUCT.md; add to README CLI section) |

## §1 Style / §8 packaging

| Item | Status | Notes |
|---|---|---|
| ≤700-line files | ✗ | lte_upstream_pppos.c 2009, modem_manager.c 1133, gps.c 971, usb_cli_commands.c 910, usb_cli_console.c ~870, config_manager.c 800 |
| Allman everywhere | ◐ | Mixed in usb_dev_ethernet.c (dead), usb_cdc_host_manager.cpp, usb_dev_bridge.c |
| README.md per component | ◐ | Ported components ship theirs; 12 legacy espnetlink components have none |
| idf_component.yml / Kconfig per component | ◐ | Ported ✅; legacy ✗ |
| Stale WiCAN GPL headers (modem_manager, usb_cdc_host_manager) | ✗ | Replace with product headers |
| Dead code: usb_dev_descriptors (empty stub), usb_dev_ethernet.c (NCM variant) | ✗ | Delete or revive decision needed |

## §7 Tests

| Item | Status | Notes |
|---|---|---|
| Host unit tests | ◐ | Ported core components include host_test/ suites (settings fields/schema/registry/codec, log ring, http matcher, wifi select, api_http utils are host-testable). Not yet wired to CI; not run on this Windows host |
| pytest test_apps on main sdkconfig/partitions | ✗ | Ported test_apps still reference WiCAN's sdkconfig defaults — repoint per §7 |
| Legacy component tests | ✗ | Only config_manager has any |

## Known deviations (product addendum candidates)

- `cmdline_manager` `uart` setting defaults **false** (WiCAN: true) — UART0
  carries boot logs + optional GPS NMEA routing.
- `dev_status_manager` events glue is a no-op (no event_manager composed).
- `api_http` delayed reboot uses esp_restart() (restart_tracker not ported yet).
- IDF v5.5: bundled `json` for cJSON; WS pre-handshake gating unavailable.
- Boot log default changed from silent (`esp_log_level_set("*", NONE)`) to
  INFO via log_manager settings — `debug 0/1` CLI still toggles globally, and
  `PUT /api/settings/log_manager {"level":"none"}` restores silence persistently.
