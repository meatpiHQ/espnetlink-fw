# led_manager — RGB status LED owner + indication arbitration (service)

Owns the AW2023 3-channel LED controller (I2C 0x45 on `i2c_bus`) and the
ARBITRATION over it: the LED is one shared resource many components want,
so clients set an INDICATION at a fixed priority instead of poking colors.
Highest occupied priority owns the LED; clearing falls back to the next
one down; the IDLE indication (settings, default solid blue) can't be
vacated, so the LED is never undefined.

The designed-for case (meatpi 2026-07-04): idle = solid blue; during an
OTA update the session sets **fast-blinking red at PRIO_CRITICAL** (via
main's `ota_manager_set_event_cb` glue) — anything lower set meanwhile is
stored, not shown, and reappears when the update ends.

## API

| Call | Behavior |
|---|---|
| `led_manager_init()` | Settings + log descriptors. No bus traffic. |
| `led_manager_start()` | AW2023 bring-up, shows the idle indication. `ESP_ERR_INVALID_STATE` when unconfigured. |
| `led_manager_stop()` | LED off; the arbitration table survives. |
| `led_manager_set(prio, state)` | Set/replace the indication at `prio` (`{mode: off/solid/blink_slow/blink_fast, r,g,b}`). |
| `led_manager_clear(prio)` | Release; next lower occupied indication shows. |
| `led_manager_active(*prio, *state)` | What's showing (status/tests). |

Priorities: `IDLE < STATUS < ALERT < CRITICAL` — small and semantic; a new
use case picks by urgency, no registration needed.

## Decided semantics

- Blink is the AW2023's **hardware pattern engine** (T1..T4 timers): zero
  CPU after the register writes. Fast ≈ 130 ms on/off, slow ≈ 510 ms.
- A mixed color blinks as one: the same pattern timing is written to every
  lit channel.
- `enabled=false` in settings → start() succeeds but stays dark; set/clear
  still book-keep (so enabling later shows the right state after reboot).
- All calls serialize on one mutex — the multi-register updates must be
  atomic (the i2c_master driver only locks single transactions).

## Settings (`"led_manager"`, version 1, field table)

`cli` (bool, default true): register this component's console command(s) with cmdline_manager on the settings boot apply (reboot-to-apply). Ownership: the component registers its own commands — main wires nothing (2026-07-05).

`enabled` (bool, true) · `idle_mode` (off|solid|blink_slow, solid) ·
`idle_r`/`idle_g`/`idle_b` (0..255, default 0,204,255 — brand cyan,
the startup color; meatpi 2026-07-19. Was 0,0,60 dim blue).

## Files

- `led_manager.c` — lifecycle, settings, mutex-serialized API.
- `led_manager_policy.c` — PURE arbiter (host-tested, 6 tests).
- `led_manager_aw2023.c` — chip layer (port of the field-proven legacy
  `led.c` onto i2c_master; same bring-up values).
- `led_manager_events.c` — event_manager glue (`led.indicate`/`led.clear`).
  **ESPNetLink port:** kept on disk, excluded from the build (no
  event_manager composed here); the call site in `led_manager.c` is a
  no-op, same as imu_manager/vpn_manager.

## Memory (estimated)

Arbiter table in PSRAM `.bss` (~100 B); mutex internal; no task.

## CLI

`led_manager_register_cli()` (main, CLI builds) registers the `led` command with cmdline_manager (`led_manager_cli.c`).

## HTTP API (endpoint reference — conventions: `components/HTTP_API.md` §6e2)

`/api/led` (2026-07-05): GET = the arbiter's current winner
(priority/mode/rgb); PUT `{r,g,b[,mode]}` = set the **ALERT** indication
(the user slot — same as `led -c`); DELETE = release it. REST drives only
ALERT: STATUS/CRITICAL stay firmware-internal so the ladder stays honest.
Idle config via `/api/settings/led_manager`.
