# dev_status_manager — HTTP API reference

> **Implemented (2026-07-03)** by the `api_http` glue (on-target suite green).
> Conventions: `components/HTTP_API.md` §1.

## GET /api/status

One poll for the dashboard header: every named status bit, uptime, running
image identity, and reboot counters (the latter come from `restart_tracker`).

**Response 200**
```json
{
  "bits": {
    "awake": true,
    "sleep": false,
    "sta_connected": true,
    "mqtt_connected": false,
    "ble_connected": false,
    "sdcard_mounted": false,
    "ble_enabled": false,
    "sta_enabled": true,
    "ap_enabled": true,
    "autopid_enabled": false,
    "home_mode": false,
    "drive_mode": false,
    "smartconnect": false,
    "sta_ap_overlap": false,
    "time_synced": true,
    "vpn_enabled": false,
    "wake_voltage_ok": true,
    "eth_connected": false,
    "autopid_idle": false,
    "motion": false,
    "sta_suspended": false,
    "ap_suspended": false,
    "ble_suspended": false
  },
  "network_connected": true,
  "uptime": "1d 02:03:04",
  "version": "v4.49p_beta-06-46",
  "partition": "ota_0",
  "boot_count": 17,
  "unexpected_resets": 1
}
```

- `bits` keys are exactly `dev_status_manager_bit_name()` strings — the UI can
  render unknown-to-it bits generically.
- `sta_suspended` / `ap_suspended` / `ble_suspended` (added 2026-07-05) are
  `interface_manager`'s arbitration state: the interface is CONFIGURED on but
  the wireless policy is holding it down (e.g. a BLE client suspends STA for
  the drive). Distinguish from `*_enabled`, which tracks the running state.
- `network_connected` is the `DEV_STATUS_NETWORK_CONNECTED_MASK` convenience
  (STA **or** ETH).
- Poll-friendly: cheap snapshot, no side effects. Suggested UI cadence 1–2 s.
- **`memory`** (added 2026-07-04, Architecture §12b): both heaps —

```json
"memory": {
  "internal": { "total": 275000, "free": 164531, "min_free": 158200,
                "largest_block": 90112 },
  "psram":    { "total": 8272000, "free": 8107828, "min_free": 8010000,
                "largest_block": 7995392 }
}
```

  `largest_block` vs `free` is the fragmentation signal; `internal` is the
  scarce heap (the radio stacks live there). A memory gauge belongs on the
  dashboard.
- **`temp_c`** (added 2026-07-08): die temperature in °C (ESP32-S3
  internal sensor, 0.1 ° resolution; omitted if the sensor errors).

## GET /api/info

Device identity for the control API (added 2026-07-11 — device-contract
v2 ask #1: the HA integration verifies it is talking to the *configured*
device before sending control commands, and pre-fills add-by-IP flows).
Same keys and casing as the webhook push `status` section.

**Response 200**
```json
{
  "device_type": "wican_pro",
  "model": "WiCAN Pro",
  "hw_version": "WiCAN-PRO",
  "fw_version": "6.0.0",
  "device_id": "14c19f44e349",
  "mac": "AA:BB:CC:DD:EE:FF",
  "api_level": 6
}
```

- `device_type` / `model`: Kconfig `WICAN_DEVICE_TYPE` /
  `WICAN_MODEL_NAME` (per-product, defaults `wican_pro` / "WiCAN Pro");
  `hw_version`: `WICAN_HW_VERSION`.
- `mac` is the STA MAC, colon-separated uppercase — identical to the
  mDNS TXT `mac` (HA's stable unique ID).
- `api_level` bumps only when the `/api/*` surface changes incompatibly.

## GET /api/status/tasks

Task monitor (2026-07-08, JSON — was text/plain): every FreeRTOS task
with state, core affinity, priority, stack high-water and the cumulative
runtime counter. Busiest first.

**Response 200**
```json
{
  "cores": 2,
  "total_us": 23785179,
  "tasks": [
    { "name": "IDLE1", "state": "R", "core": 1, "prio": 0,
      "stack_hw": 776, "runtime_us": 21699114 },
    { "name": "httpd", "state": "B", "core": -1, "prio": 5,
      "stack_hw": 2148, "runtime_us": 913245 }
  ]
}
```

- `state`: `X` running, `R` ready, `B` blocked, `S` suspended, `D`
  deleted-pending-cleanup. `core`: `-1` = unpinned.
- `stack_hw` = stack bytes the task has NEVER used — small values mean
  it is close to overflowing (the httpd-4KB lesson).
- **CPU% is a client-side delta**: poll twice, then per task
  `Δruntime_us / (Δtotal_us × cores) × 100`. `total_us` is PER-CORE
  scheduler time; the IDLE0/IDLE1 deltas give the idle share (system
  load = 100 − idle%). Counters are u64 microseconds — no wrap handling
  needed. The web UI System Monitor page (`#/monitor`) and `system -t`
  (1 s window) both do exactly this.

**Errors**: `501 {"error":"CONFIG_FREERTOS_USE_TRACE_FACILITY is off"}`
(the main config has it on).

**No mutating routes.** Bits are runtime state owned by the components that
publish them; there is deliberately no `POST /api/status`.
