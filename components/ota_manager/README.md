# ota_manager — the firmware update manager (core service)

Owns the device's OWN firmware update: one byte-stream session
(`begin` → `write`× → `end`/`abort`) over the `ota_0`/`ota_1` partitions
(esp_ota). **Transport-agnostic by design**: transports feed the session —
HTTP multipart upload from the web UI today (`api_http`,
`POST /api/ota/upload`), raw HTTP body for tooling, and TCP/MQTT/anything
later through the same four calls, with no changes here.

Naming: "OTA" = this device's application image. The OBD chip's firmware
update is a different mechanism (`obd_chip_firmware_update`).

## API

| Call | Behavior |
|---|---|
| `ota_manager_init()` | Log descriptor; no flash access. |
| `ota_manager_start()` | Confirms the running image when rollback is enabled (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`, off today); logs the running partition. |
| `ota_manager_begin(total_or_0)` | Opens THE session against the next OTA partition (`0` = size unknown, e.g. multipart). `ESP_ERR_INVALID_STATE` while another session is receiving; begin from FAILED = retry, from READY = re-upload. |
| `ota_manager_write(data,len)` | Stream image bytes, any chunking. Overrun of an announced size / backend failure → FAILED + flash session released. |
| `ota_manager_end()` | Validates (esp_ota magic + digest), sets the boot partition → READY. The TRANSPORT then reboots via `restart_tracker_restart(OTA_APPLY, <source>)`. |
| `ota_manager_abort()` | Drop the session; the running firmware is untouched. |
| `ota_manager_status(*out)` | state / received / total / error / target partition (feeds `GET /api/ota/status`). |

A failed or aborted session can never hurt the running firmware — the
inactive partition is scratch space until `end()` flips the boot pointer,
and `esp_ota_end` rejects images that don't validate.

## Dependencies

`app_update` (esp_ota), `log_manager` — private. No transport dependencies
(the whole point). The reboot is the transport's job through
`restart_tracker` — this component never restarts anything.

## Memory footprint

Static: session struct + mutex (<200 B). esp_ota's own write buffering is
IDF-internal; the HTTP transport's 4 KB rx buffer is request-scoped heap
(§12b). No steady-state allocations.

## Tests

- **Host (`host_test/`, 9 tests)**: the pure session state machine against
  a recorder backend — happy path, write-before-begin, double-begin,
  announced-size overrun, short image, empty image, backend-failure
  latching (+ no double-abort after validation failure), retry-from-FAILED,
  abort-resets. **Green on rpi001 2026-07-04.**
- **Live (2026-07-04, the real thing)**: 1,510,992-byte image uploaded over
  Wi-Fi to the composed firmware via BOTH transports — multipart
  (`curl -F firmware=@wican-fw.bin`, the HTML-UI path) and raw body
  (`curl --data-binary`) — **~8.3 s each (~180 KB/s incl. flash writes)**;
  partition flipped ota_0→ota_1→ota_0, device self-rebooted via
  restart_tracker, history records `planned ota_apply web_ui`, memory
  healthy after. Serial-flash recovery is safe: `flash_args` includes
  `ota_data_initial.bin`, resetting boot to `ota_0`.
