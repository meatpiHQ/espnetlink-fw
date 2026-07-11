# imu_manager — motion sensor owner (feature)

Owns the ICM-42670-P IMU (I2C 0x68 on `i2c_bus`) and runs **both** of its
motion detectors side by side. The detector task **polls** `INT_STATUS2`
every 200 ms (meatpi's call — safer than an ISR and nothing here is
time-critical): the status bits **latch until read**, so no event is
missed and worst-case detection latency is one tick. The chip's INT1 pin
(wired to GPIO3 on WiCAN Pro) stays configured but unlistened — flip to
interrupt mode only if latency ever matters.

- **SMD** (Significant Motion Detection, DMP/APEX — needs SUSTAINED
  motion) drives the ACTIVITY state: ACTIVE (+ `DEV_STATUS_BIT_MOTION`)
  until quiet for `stationary_s` → STATIONARY. "The vehicle is moving."
- **WoM** (Wake on Motion, per-sample threshold) catches single bumps —
  door open/close, a knock, vibration. It does NOT touch the activity
  state; it is published as an EVENT. Reference is the **previous
  sample** (differential), not legacy's initial sample — an initial
  reference latches forever after any orientation change (parked on a
  hill, remounted).

## Events (subscribe and react)

Any component attaches a queue and reacts — send a CAN frame on a door
slam, an MQTT alert on motion, wake logic, whatever the product defines:

```c
QueueHandle_t q = xQueueCreate(8, sizeof(imu_manager_event_t));
imu_manager_subscribe(q);
// items: {type: WOM|SMD|ACTIVE|STATIONARY, wom_axes, uptime_ms}
```

Semantics: fan-out to every subscriber (≤4) with a zero-timeout send — a
full queue drops that subscriber's copy (counted + warned), the detector
task never blocks. WOM publishes are throttled to one per 500 ms so
driving vibration can't flood anyone; ACTIVE/STATIONARY transitions are
events too, so consumers don't poll.

## Driver note (vendored)

The chip driver is the **vendored espressif/icm42670 2.1.0** —
`icm42670/` with `PROVENANCE.md`: the managed component's mandatory
`sensor_hub` dependency doesn't compile on IDF v6, so the (otherwise
untouched) driver lives here minus the hub glue. Both detectors are
implemented in `imu_manager.c` via the driver's raw register access (it
has no WoM/APEX API). SMD recipe per DS-000451 v1.0: accel LP 50 Hz →
DMP SRAM reset → `DMP_INIT_EN` (self-clears, polled) →
`SMD_SENSITIVITY_SEL` in MREG1 `APEX_CONFIG9` → `SMD_ENABLE` +
`DMP_ODR=50 Hz` in `APEX_CONFIG1`. WoM: per-axis thresholds in MREG1,
`WOM_CONFIG` = enabled/previous-ref/OR/first-event. INT1 routing is kept
(status latching matches the verified config) but the pin is unlistened;
the polling task reads `INT_STATUS2` (read-clear) to tell the detectors
apart.

## API

| Call | Behavior |
|---|---|
| `imu_manager_init()` | Settings + log descriptors. No bus traffic. |
| `imu_manager_start()` | Chip probe (WHO_AM_I), WoM config, INT GPIO ISR, activity task. `ESP_ERR_INVALID_STATE` when unconfigured. |
| `imu_manager_activity()` | STATIONARY / ACTIVE / UNKNOWN (debounced). |
| `imu_manager_subscribe(q)` / `unsubscribe(q)` | Attach/detach an event queue (see Events above). |
| `imu_manager_register_http()` | `GET /api/imu` → `{activity, accel{x,y,z} g, temp °C}` — HTTP_API.md §6e; main wires it. |
| `imu_manager_read_accel(x,y,z)` | g units (accel always on, LP mode). |
| `imu_manager_read_temp(t)` | Die temperature °C. |
| `imu_manager_device_id(*id)` | WHO_AM_I (0x67) — diagnostics. |

## Decided semantics

- Accel runs permanently in **low-power mode at 50 Hz** feeding both
  detectors (`ACCEL_ODR` must be ≥ `DMP_ODR`); reads work anytime.
- **Gyro stays OFF in v1**: nothing consumes it and it dominates the
  chip's power budget. Add a read API + lazy power-up when a consumer
  exists.
- Detector task polls `INT_STATUS2` every 200 ms (read-clears the
  latched bits): the PURE state machine + WOM gate
  (`imu_manager_policy.c`, injected clock, wrap-safe) decide
  transitions/publishes; transitions log at INFO, raw events at DEBUG.
- Activity is **SMD-only** by design: loading the trunk (bumps) is not
  "the vehicle is moving". Consumers that care about bumps subscribe to
  WOM events.
- `smd_sensitivity` 0..4 (datasheet): low = more detections + more false
  positives; high = robust but "reduces detection rate, especially for
  transport use cases" — hence default 0 for a car device.

## Settings (`"imu_manager"`, version 3, field table)

`cli` (bool, default true): register this component's console command(s) with cmdline_manager on the settings boot apply (reboot-to-apply). Ownership: the component registers its own commands — main wires nothing (2026-07-05).

`enabled` (bool, true) · `smd` (bool, true) · `wom` (bool, true) ·
`smd_sensitivity` (0..4, default 0) · `wom_threshold` (1..255, default 8
≈ 31 mg) · `stationary_s` (1..3600, default 3).

History: v1 = WoM only, v2 = SMD only, v3 = both. Both migrations are
pass-throughs (`imu_manager_settings.c`, pure, host-tested) — every
historical key is valid again; validation fills the new keys from schema
defaults. v1→v2 and v2→v3 both verified LIVE on the bench device.

## Files

- `imu_manager.c` — lifecycle, dual-detector bring-up, ISR/task, event
  fan-out, settings.
- `imu_manager_policy.c` — PURE activity state machine + WOM publish
  gate (host-tested, incl. clock wrap).
- `imu_manager_settings.c` — PURE migrations (host-tested).
- `icm42670/` — vendored driver (Apache-2.0, see PROVENANCE.md).

## Memory

Activity task 3 KB PSRAM stack; driver handle ~100 B heap. (estimated)

## CLI

`imu_manager_register_cli()` (main, CLI builds) registers the `imu` command with cmdline_manager (`imu_manager_cli.c`).
