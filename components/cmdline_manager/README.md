# cmdline_manager

The WiCAN command line: ONE esp_console-backed command **registry** and
dispatcher, reachable over every transport **without any transport code
in this component** — and owning **no commands except `help`**
(rewrite of the legacy `cmdline`; registry shape follows IDF's
`examples/system/console` per-domain `register_*()` convention).

## Who owns which command

Every component registers its own commands with
`cmdline_manager_register()` from a `<comp>_cli.c` /
`<comp>_register_cli()` that the component itself calls on its settings
boot apply, gated by its own **`cli` setting** (bool, default true,
reboot-to-apply) — main wires nothing (meatpi 2026-07-05; the
`*_register_http()` ownership rule taken one step further). Components
without other settings (dev_status_manager, filesystem,
external_storage, restart_tracker) carry a minimal `{cli}` descriptor
registered via `<comp>_register_settings()` right after
settings_manager_init (they init before it — the
log_manager_register_settings pattern). Turning `cli` off removes that
component's commands from every surface (UART, WS, BLE) at the next
boot:

Interface rule: **legacy argtable3 options behave legacy-exact**
(outputs byte-alike, `OK` trailers); bare invocation gives a v6 status
summary (legacy bare was an error — nothing depended on it); v6
additions are options too.

| Command | Owner |
|---|---|
| `help [cmd]` | cmdline_manager (the registry's own; legacy format) |
| `version`, `status` | dev_status_manager |
| `wifi -s/-i/--scan` | wifi_manager (`-s`/`-i` legacy-exact; `--scan` = v6 JSON scan) |
| `rtc -s/-r/-i` | rtc_manager (sync/read legacy-exact; `-i` = responding probe — the RX8130 has no ID register) |
| `imu -i/-r` | imu_manager (`-i` legacy-exact; `-r` + bare = v6 activity/accel/temp) |
| `led -i/-c <r g b>/-b/-x` | led_manager (legacy set-color drives an ALERT-priority indication — the arbiter owns the LED; `-x/--clear` releases it) |
| `battery` | battery_monitor (new in v6) |
| `sdcard -i/-t` | external_storage (`-i` card name/type/capacity/sector/speed from its own sdmmc data; `-t` write/read-back self-test) |
| `fs` | filesystem (per-backend usage: /data, /sd) |
| `restart_tracker -l/-a/-p/-n/--panic` | restart_tracker (`--panic` = deliberate test panic; v6 flags print as hex) |
| `system -v/-r/-i/-m/-t` | **main_cli.c** — a composite (battery voltage + restart_tracker reboot + dev_status info/memory/tasks); only main knows all of them (`-t/--tasks` is the v6 addition) |
| `debug -e <0\|1>` or `debug <tag> <level>` | **main_cli.c** — log_manager sits BELOW cmdline_manager; registering there would be a dependency cycle (same reason /api/logs lives in api_http). `-e` legacy-alike but EPHEMERAL (persist via `/api/settings/log_manager`) |
| `factoryreset [-c/--confirm]` | **main_cli.c** — REAL since 2026-07-05: legacy two-step confirm flow (60 s window) -> `settings_manager_factory_reset()` (settings partition only; /data + SD untouched) -> reboot via restart_tracker `FACTORY_RESET` |
| `autopid [-l]` | autopid (self-registered on settings apply, §6b; bare = status/tables/poll counters, `-l` adds per-parameter latest values + age) |
| `eth`, `conn`, `espnetlink`, `wusb`, `ping` | **main_cli.c** pending stubs — no v6 backer yet; only the composition root knows what's missing |

`help` output follows the legacy format (`Available commands:` +
`cmd - help` + indented `Usage:` hints + `Tip:`/`OK`; `help <cmd>` for
details).

## How each transport reaches the CLI

| Transport | Path |
|---|---|
| TCP / UDP | configure a bridge: `br_x = cli <-> obd0/udp0/...` (socket_manager server) |
| WebSocket | configure a bridge: `br_x = cli <-> ws_cli`, then `ws://<ip>/ws/cli` |
| BLE | main glues the CLI IN/OUT characteristics to `cmdline_manager_exec_line_async()` — lines QUEUE to the dispatcher task (4-deep; full = "busy" + prompt back), because they arrive on the NimBLE host task, which must never run a command inline (a long command wedged the BLE stack — fixed 2026-07-10, `ble_cli_wedge_test.py`) |
| UART0 | built-in **linenoise** console (settings `uart`, default on): line editing, arrow-key history (RAM, 32), tab completion + hints from the registry, dumb-terminal fallback via probe. Manual loop per the IDF advanced console example — NOT the REPL component, whose internal loop would bypass the dispatcher lock. |

Input bytes from the endpoint face are assembled into lines (`\r`/`\n`
terminate, ≤255 chars, oversize discarded whole), each line runs through
the one registry, and the response routes back to the invoking transport
only. One command at a time device-wide (a mutex serializes every
transport). Transport responses end with the legacy `wican> ` prompt;
on UART linenoise draws it.

Command handlers print with `cmdline_printf()` (formatted, ≤256 bytes
per call) or `cmdline_write()` (raw, any length — scan JSON, task
lists).

## Caller contract (§2 corollary — learned the hard way)

Handlers run arbitrary component code, so **every task that calls
`cmdline_manager_exec_line()` needs an INTERNAL-RAM stack** with a few
KB of headroom: `fs` walks LittleFS on internal flash, which panics from
a PSRAM-stack task (cache-off assert — hit live). The dispatcher (6 KB)
and console (8 KB) tasks are internal for exactly this reason. Tasks
that can't satisfy the contract (or must never block — the NimBLE host
task) use `cmdline_manager_exec_line_async()` instead: the line +
sink queue to the dispatcher (a FreeRTOS QueueSet lets it serve both
the endpoint face and async lines), which runs the command on its own
internal stack.

## Settings (`cmdline_manager`, reboot-to-apply)

| Field | Default | Meaning |
|---|---|---|
| `enabled` | `true` | master switch (dispatcher + all transports) |
| `uart` | `true` | the UART0 linenoise console |

## Files

- `cmdline_manager.c` — lifecycle, registry + `help`, dispatcher, sink routing, endpoint trio
- `cmdline_manager_console.c` — the UART0 linenoise console
- `cmdline_manager_line.c` — PURE byte-stream→line assembler (host-tested)
- `host_test/` — assembler tests (6; runs on the IDF `linux` target)

## Memory (§2/§12)

Registry mirror + input-queue storage in PSRAM; both task stacks
INTERNAL (see caller contract); FreeRTOS objects internal. UART driver
(256 B RX buffer) installed only when the console is enabled.

Live verification (2026-07-04): UART0 linenoise console (echo/editing/
help format) and the WebSocket console via `br_cli = cli <-raw-> ws_cli`
from rpi001 — `tools/testbench/cli_ws_test.py` → `CLI WS PASS`.
BLE async path (2026-07-10): `tools/testbench/ble_cli_wedge_test.py`
(UB500 on rpi001, main firmware, WiFi+BLE both on) — GATT reads stay at
52 ms median while 4 s of queued commands execute; 8-line flood all
answered (busy fallback exercised); link never drops.
