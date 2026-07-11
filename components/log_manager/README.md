# log_manager (Debug Log Manager, Coding Standard §9)

## Summary

Core component owning the log pipeline. It installs itself with
`esp_log_set_vprintf()`, so every component and IDF internals keep using plain
`ESP_LOGx` — no wrapper macros, no call-site migration. Lines flow through a
fixed-depth PSRAM queue into a dedicated log task, which fans them out to
**sinks**. The producer path is wait-free: a full queue drops the *oldest*
line and counts it (`dropped N messages` is emitted when pressure clears);
a slow sink can never stall a CAN task. No logging from ISRs (ISR calls are
detected and dropped).

**Routing is fully pluggable.** A sink is `{name, write(line,len)}`; its
`write` runs in the log task only. Built in:

| Sink | What it does |
|---|---|
| `console` | UART stdout, live from `init()` (§9.3) |
| `ring` | PSRAM `.noinit` ring (default 16 KiB) that **survives warm resets** — same magic/version/CRC + `esp_cache_msync` pattern as `restart_tracker`. After a crash/reboot, `log_manager_ring_read()` returns the last lines *including pre-reset output*, with **zero flash wear**. A `---- boot ----` separator is written every boot. |

TCP / UDP / WebSocket / file sinks are **external registrants** — the manager
deliberately has no network or filesystem dependency (Architecture §3). The
recipe:

```c
static esp_err_t udp_write(const char *line, size_t len) { /* sendto(...) */ }
static const log_sink_t UDP_SINK = { "udp", udp_write };
log_manager_add_sink(&UDP_SINK);   // from the owning feature component
```

**Flash-wear rule for a future file sink:** never write per-line. Accumulate
into a RAM buffer and flush on a coarse period (≥ 60 s) or on buffer-full,
through an internal-stack writer task (§2 corollary — the log task itself
runs on a PSRAM stack and performs no flash writes). For crash forensics the
PSRAM ring already covers the common case without touching flash at all.

## API

| Function | One-liner |
|---|---|
| `log_manager_init()` | Hook + console + ring adopt. **First init in main** (§9.6); synchronous routing until start. |
| `log_manager_start()` / `_stop()` | Spawn/stop the log task (queued routing). |
| `log_manager_register_settings()` | Register the `"log_manager"` settings descriptor — call after `settings_manager_init()`. |
| `log_manager_register(desc)` | Per-component `{TAG, default_level}` (§9.2). |
| `log_manager_set_level(name, level)` | Runtime, **ephemeral** level knob (§9.4). |
| `log_manager_add_sink(sink)` / `log_manager_sink_set_enabled(name, on)` | Pluggable routing. |
| `log_manager_sink_get(index, *name, *enabled)` | Enumerate sinks (for `/api/logs/status`); `ESP_ERR_NOT_FOUND` past the end. |
| `log_manager_dropped_count()` | Backpressure drops since boot. |
| `log_manager_ring_read(buf, len, *n)` / `log_manager_ring_clear()` | Crash-ring access (chronological, spans reboots). |

## Dependencies

- `log` (public: header exposes `esp_log_level_t`); `settings_manager` +
  `esp_mm` private.
- Requires `CONFIG_SPIRAM_ALLOW_NOINIT_SEG_EXTERNAL_MEMORY=y` and
  `CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y` (main sdkconfig has both).
- Boot order: `log_manager_init()` first; `log_manager_register_settings()`
  in the registration phase; `log_manager_start()` after
  `settings_manager_start()`.

## Settings (`"log_manager"`, version 1)

| Key | Type | Default | Notes |
|---|---|---|---|
| `level` | enum `none/error/warn/info/debug/verbose` | `info` | global boot level (`esp_log_level_set("*", …)`); capped by `CONFIG_LOG_MAXIMUM_LEVEL` |
| `console_enabled` | bool | `true` | boot state of the console sink |
| `ring_enabled` | bool | `true` | boot state of the PSRAM ring sink |

Runtime changes via `set_level`/`sink_set_enabled` reset to these on reboot.

## HTTP API (requirement — full endpoint reference: `HTTP_API.md` in this directory; conventions: `components/HTTP_API.md`)

Via the `api_http` glue (never a direct dependency on the HTTP server):
`GET/DELETE /api/logs/ring` (crash-ring dump as text/plain, chunked / clear),
`GET /api/logs/status` (drop counter + sink states), `PUT /api/logs/level`
and `PUT /api/logs/sink` (the ephemeral runtime knobs, §9.4). Persisted
defaults stay on `/api/settings/log_manager`.

## Kconfig

`LOG_MANAGER_LINE_MAX` (256), `LOG_MANAGER_QUEUE_DEPTH` (64),
`LOG_MANAGER_RING_SIZE` (16384).

## Memory footprint (estimated — measure before release)

| Where | What | Size |
|---|---|---|
| PSRAM `.bss` | queue (64×258 B) + registries + log-task stack (16 KiB) | ~33 KiB |
| PSRAM `.noinit` | crash ring + header | ~16.1 KiB |
| Internal | spinlock, TCB, flags | ~400 B |
| Producer cost | one 256 B line buffer on the caller's stack | per call |

## Tests

- **Host (`host_test/`, 9 tests):** the pure ring core — garbage detection,
  reset, roundtrip, wrap-keeps-newest, oversized append, newest-window reads,
  header tamper, size mismatch, tuning-guard clobber harmless.
- **On-target (`test_apps/`):** ESP_LOGx → custom sink capture, runtime level
  gate, sink disable, drop-oldest under a priority-5 burst, and ring survival
  across a **real esp_restart()** (two-phase). Main partition table per
  rev 2.1.
