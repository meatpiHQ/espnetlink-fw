# filesystem

**Summary** — Mounts the general-purpose LittleFS user filesystem (partition label `storage`) at `/storage` and exposes a minimal open helper. This is the WiCAN shared-core surface consumed by `http_server_manager` for serving files; `main` calls `filesystem_init()` once at boot. Settings storage is deliberately NOT here — `settings_manager` owns its own dedicated partition (standard §4.4), so a corrupted user filesystem can never take down settings.

## API

Lifecycle:

- `esp_err_t filesystem_init(void)` — register/mount LittleFS at `FS_BASE_PATH` (`format_if_mount_failed`, `grow_on_mount`); logs total/used KB.

Other:

- `FILE *filesystem_open(const char *path, const char *mode)` — open a file through the component (NULL when the path can't be opened). The component owns the mounts; callers never touch VFS registration.
- `FS_BASE_PATH` — `"/storage"` mount-point macro.

No `_start`/`_stop`: passive after init.

## Dependencies

- `joltwallet__littlefs` (managed component) — LittleFS VFS driver.
- Init order: `filesystem_init()` before any `filesystem_open()`; `main` runs it after the core platform init.

## Settings

None (no persisted settings).

## Memory footprint (estimated)

**All numbers estimated.**

- Static (`.bss`/`.data`): none beyond the TAG string; no large static buffers.
- Task stacks: none (no tasks; runs in callers' context).
- Heap: LittleFS mount allocates its cache/metadata from internal heap (sized by the littlefs component config). Peak heap: not yet measured.
