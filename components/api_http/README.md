# api_http

**Summary** — Glue layer implementing the device's `/api/settings` HTTP surface (MeatPi standard §6) on top of `http_server_manager`: per-component get/put/schema, list, submit-with-conditional-reboot, whole-device backup/restore, and factory reset. Core components never touch HTTP — this glue depends DOWN on them. Routes register buffered at init and go live when `http_server_manager_start()` runs. The central route map lives in `components/HTTP_API.md`; any route change updates that file in the same commit (§6).

## API

- `esp_err_t api_http_init(void)` — register the `/api/settings` routes (buffered) + log descriptor; call after `http_server_manager_init()`.
- `esp_err_t api_http_start(void)` / `esp_err_t api_http_stop(void)` — lifecycle uniformity (§3); trivial, the component is passive after init.

Routes served (per-endpoint reference: `components/HTTP_API.md`):
`GET /api/settings` (list) · `GET|PUT /api/settings/<name>` · `GET /api/settings/<name>/schema` · `POST /api/settings/submit` · `GET|POST /api/settings/backup` · `POST /api/settings/factory_reset`.

Conventions: JSON in/out; GET never mutates; secrets redacted to `""` on reads and `""` on PUT = keep stored; PUT is full-object replace; submit reboots (delayed ~1 s so the response flushes) only when a PUT in the batch actually changed something. ESPNetLink port note: no `restart_tracker` yet, so the delayed reboot uses `esp_restart()` directly.

## Dependencies

All `PRIV_REQUIRES` (public header exposes only `esp_err_t`):

- `http_server_manager` — buffered route registration + server lifecycle
- `settings_manager` — get/set/schema/list + factory reset
- `dev_status_manager` — backup metadata (device id)
- `log_manager`, `esp_http_server`, `json` (cJSON)
- Init order: `http_server_manager_init()` → `api_http_init()` → … → `http_server_manager_start()`.

## Settings

None (no persisted settings — it is the HTTP transport for everyone else's).

## Memory footprint (estimated)

**All numbers estimated.**

- Static (`.bss`/`.data`): ~72 B status-line scratch + flags; no large static buffers.
- Task stacks: `api_reboot` 3072, prio 5, dynamic internal — transient, spawned only to delay `esp_restart()` after a submit/restore/factory-reset response.
- Heap: request bodies allocated per call from PSRAM (`MALLOC_CAP_SPIRAM`): settings PUT ≤8 KB, backup restore ≤32 KB. Peak heap: not yet measured.
