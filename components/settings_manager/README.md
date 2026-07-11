# settings_manager

## Summary

Core component that owns settings persistence and validation for the whole
firmware. Components never touch the filesystem: each registers a descriptor
(schema + hooks). **Reboot-to-apply** (Coding Standard rev 2 §4.2): `set()`
validates and persists only; `on_apply` runs once per component at boot,
single-threaded, in `settings_manager_start()`'s context, before any
component's `_start()`. There is no live-update path and components carry no
reload code. The manager owns LittleFS on a dedicated partition, validates
every change against the component's JSON Schema, and persists atomically
(temp-file + rename, CRC-32 envelope, write dedup). It sits at the core layer;
services and transports (HTTP, CLI) depend down onto it, never the reverse.
See the repository `ARCHITECTURE.md` for the full picture.

## API

`include/settings_manager.h`:

- `esp_err_t settings_manager_init(void)` — mount FS, prepare registry. No apply.
- `esp_err_t settings_manager_start(void)` — the §4.3 boot pass for every component: load → CRC check → `on_migrate` (if stored version is older) → validate → `on_apply`, falling back to defaults (persisted + logged) on any failure. If a component rejects even defaults it is left unconfigured and marked degraded; boot always continues.
- `esp_err_t settings_manager_stop(void)` — release and unmount.
- `esp_err_t settings_manager_factory_reset(void)` — delete EVERY stored settings file; the running config is untouched and the caller MUST reboot (via `restart_tracker`, reason `FACTORY_RESET`) so the next boot applies pure factory defaults. Scope: the settings partition only — `/data` (certs, files), the SD card and NVS are NOT touched. Consumer: the CLI `factoryreset` command (legacy two-step confirm flow, 60 s window).
- `esp_err_t settings_manager_register(const settings_descriptor_t *desc)` — register a component (between init and start). Rejects unparseable schemas and invalid names (`[a-z0-9_]`, ≤ 32 chars) at the call site.
- `esp_err_t settings_manager_get(const char *name, cJSON **out)` — copy of current settings; caller frees.
- `esp_err_t settings_manager_set(const char *name, const cJSON *in, char *err, size_t err_len, bool *changed)` — **full-object replace**: validate (schema, including `required`, after filling omitted optional keys from schema defaults) → `on_validate` → persist. **Never applies**; the change takes effect at next boot. Identical content skips the write and reports `changed=false`, which the transport uses to skip the post-submit reboot. `err` (optional) receives the failure reason, private to this call.
- `bool settings_manager_is_degraded(const char *name)` — true if the boot fallback fired (§4.3 steps 4–5). Transports add `"degraded":true` to GET responses from this; the manager never injects synthetic keys into settings objects.
- `esp_err_t settings_manager_get_schema(const char *name, cJSON **out)` — copy of the parsed schema; caller frees (never dangles across `stop`).
- `esp_err_t settings_manager_list(cJSON **out)` — array of `{name, version}`; caller frees.
- `const char *settings_manager_last_error(void)` — last failure reason. **Single-threaded convenience only**: the buffer is shared across callers, so concurrent transports must use `set`'s `err` out-param instead.
- `bool settings_manager_is_pending_reboot(const char *name)` — persisted (pending) object differs from what on_apply ran with at boot (exact compare vs the boot snapshot; saving the boot values back clears it). Transports surface it as `"pending_reboot"` — the UI's "restart to apply" indicator (2026-07-05).
- `esp_err_t settings_manager_export(cJSON **out)` — every component as `{"<name>":{"version":N,"data":{…}}}` for backup/transfer (2026-07-05). Pending values, **passwords verbatim** (redaction is the transport's call); caller frees.
- `esp_err_t settings_manager_restore(name, version, in, dry_run, err, err_len, changed)` — restore one component from a backup: `on_migrate` when the backup is older (exactly the boot-pass path) → fill defaults → validate → persist via the `set` pipeline (dedup, reboot-to-apply). Newer-than-firmware versions are rejected (`ESP_ERR_INVALID_VERSION`). `dry_run` stops before persisting so transports can make a whole-document restore all-or-nothing (`POST /api/settings/backup` does).

`settings_descriptor_t` fields: `name`, `version`, `fields`/`field_count` (the
field-table schema — the component authoring format, standard §4.1 rev 2.5;
the generated JSON Schema is the source of truth for shape, types, ranges, and
defaults via per-property `default` keywords), `schema` (raw JSON Schema
string — reserved for this component's own tests; exactly one of
`schema`/`fields` per descriptor), `defaults_json` (optional **whole-object
override** of the schema-derived defaults; validated as a JSON object at
registration; NULL for most components), `on_apply` (required; boot only),
`on_validate` (optional), `on_migrate` (optional; called once with the stored
version when it is older than `version` — must handle any historical version;
the result is re-validated and persisted).

> `get()` returns the **pending** (post-reboot) values after a `set()` — that is
> what a UI wants to display after a save.

## Dependencies

- `json` (cJSON) — **public** (`REQUIRES`); the header exposes `cJSON` types.
- `littlefs` (`joltwallet/littlefs`, via `idf_component.yml`) — **private**
  (`PRIV_REQUIRES`); only the `.c` files use it.

Init order: `settings_manager` is a root component. It depends on nothing above
it. `main` calls `settings_manager_init()` first, then each component's `init`
(which registers a descriptor), then `settings_manager_start()`.

**Partition requirement:** the target's `partitions.csv` must define a
`littlefs`-subtype data partition labelled `settings` (the manager mounts it at
`/settings`, files under `/settings/cfg/<name>.json`).

## Constraints & limitations (read before writing a descriptor)

- **`on_apply` runs only at boot**, single-threaded in `settings_manager_start()`,
  before any component's `_start()` — no threading contract needed; the
  component's own tasks do not exist yet. **`on_validate` runs at `set()` time**
  with the manager's lock held: keep it fast and take no locks another task can
  hold while calling into the manager (ABBA deadlock).
- **Envelopes are produced and consumed by this codec only.** The CRC is computed
  over the cJSON re-serialization of `data`; a settings file generated by another
  tool with different JSON formatting will fail the CRC check even if honestly
  checksummed. Provision settings through the API, not by writing files.
- **The schema validator does not recurse into nested objects.** `type:"object"`
  checks that the value is an object; inner properties of nested objects are not
  validated. Keep settings flat, or validate nesting in `on_validate`.
- **Defaults are trusted, not validated.** A `default` that violates its own
  schema will be applied at boot but rejected on the next `set` of the same value.
  Author defaults inside their constraints.
- **Migrations are the component's job.** When the stored version is older than
  the descriptor's, `on_migrate(from_version, settings)` is called once and must
  handle any version the component has ever shipped; the result is re-validated
  and persisted at the new version. Absent or failing → defaults (config lost),
  so ship the hook with every breaking schema change (and a unit test proving
  every historical version migrates — Coding Standard §7).
- **Without PSRAM** (`CONFIG_SPIRAM` off) `EXT_RAM_BSS_ATTR` is empty and the
  registry (~1.2 KB) lands in internal `.bss`. This is correct fallback behaviour,
  not a bug.

## Settings

This component stores no settings of its own — it is the store. What it enforces
for everyone else:

- **On-disk format:** one JSON file per component, wrapped in an integrity
  envelope `{"crc32":N,"version":N,"data":{...}}`. CRC mismatch, parse failure,
  failed migration, or schema violation on load → fall back to defaults (persist
  them, log). If `on_apply` then rejects even the defaults, the component is left
  unconfigured, `settings_manager_is_degraded()` reports true, and boot continues
  — one broken component never bricks the device (§4.3).
- **Atomic writes:** temp file, `fsync`, then `rename`, so a power loss mid-write
  leaves the previous good file intact (the fsync forces data to media before the
  swap — `fflush` alone only empties stdio buffers).
- **Schema keywords** validated: `type` (`string`/`integer`/`number`/`boolean`/
  `object`/`array`), `minimum`, `maximum`, `minLength`, `maxLength`, `enum`,
  `required`, and the WiCAN extension `format:"file"` (the referenced filename
  must exist on the FS, else the change is rejected before apply/persist).
  Arrays are **bounded**: `maxItems` (1..16) is mandatory, `minItems` optional,
  and `items` describes every element — a scalar schema or ONE level of
  `type:"object"` with its own `properties`/`required` (validation errors carry
  the element index, e.g. `servers[1].port: above maximum`). Added 2026-07-03
  for socket_manager/bridge_manager.
- **Field-table authoring (the component format — standard §4.1 rev 2.5).**
  A descriptor sets `fields`/`field_count` to a `settings_field_t` table —
  one row per key via the `SETTINGS_STR` / `SETTINGS_STR_LEN` /
  `SETTINGS_STR_ENUM` / `SETTINGS_STR_FILE` / `SETTINGS_INT` / `SETTINGS_BOOL`
  macros (`_REQ` variants mark required keys). Bounded arrays of objects use
  `SETTINGS_ARRAY(key, max_items, ITEM_FIELDS, default)` (item rows are scalar
  only — one nesting level, mirroring the validator) or
  `SETTINGS_ARRAY_ANY(key, max_items, default)` for free-form object items
  (enforce their shape in `on_validate`). Array defaults are written as plain
  JSON through `SETTINGS_JSON([...])` — the preprocessor stringizes it (each
  JSON string must stay one C token; adjacent-literal concatenation happens
  after stringization and corrupts the JSON). `register()` generates the
  equivalent JSON Schema (rejecting malformed tables loudly — including
  unbounded/oversized arrays and nested arrays) and everything downstream —
  validation, defaults, `get_schema` — runs on that schema unchanged. Item-row
  defaults surface in the generated items schema as UI "add row" prefill
  hints. See `wifi_manager_settings.c` (flat) and `socket_manager_settings.c`
  (arrays) for real tables.
- **Defaults live in the schema.** A property's `default` keyword seeds that key
  when it is absent — on first boot (no file), when a non-breaking schema
  addition introduces a key, and for optional keys omitted from a `set()`. The
  descriptor's `defaults_json`, if non-NULL, is a **whole-object override** that
  replaces the schema-derived defaults entirely (validated at registration).
- **No redundant writes.** `set` skips the flash write when the resulting object
  is identical to what is already persisted (and reports `changed=false` so the
  transport also skips the reboot). A write still happens when values actually
  change. `start` writes only to repair — persisting defaults after a
  missing/corrupt/invalid file, a migration result, or filled-in schema
  additions; a clean boot on valid stored data costs no flash wear.

## HTTP API (requirement — full endpoint reference: `HTTP_API.md` in this directory; conventions: `components/HTTP_API.md`)

The generic settings surface every component's config UI uses, implemented by
the `api_http` glue (this component never depends on transports):
`GET /api/settings` (list), `GET/PUT /api/settings/<name>` (full-object
replace, degraded flag), `GET /api/settings/<name>/schema` (self-describing
forms), `POST /api/settings/submit` (batch end → reboot via
`restart_tracker_restart(CONFIG_APPLY, CONFIG_SERVER)` only when something
changed). Password-typed fields are redacted in GET responses.

## Memory footprint

No task of its own (passive; driven by callers), so no task stacks. All numbers
**estimated** — replace with **measured** before release using the commands
below.

| Region                         | Where         | Size (est.) | Notes                                  |
|--------------------------------|---------------|-------------|----------------------------------------|
| Registry `s_registry[32]`      | PSRAM `.bss`  | ~1.2 KB     | `EXT_RAM_BSS_ATTR`; 32 × ~36 B entry   |
| `s_last_error[128]` + mutex    | internal `.bss` | ~0.25 KB  | small, lock-held; off PSRAM            |
| FS write scratch               | internal heap | = file size, transient | `// internal: cache-off during FS write` |
| Per-op cJSON (get/set/start)   | heap          | a few KB transient | freed after each call           |
| LittleFS cache/buffers         | internal      | per littlefs config | owned by the littlefs component |

Measure (per Coding Standard §3):

- Static: `idf.py size-components` (look for `settings_manager`).
- Heap deltas: `heap_caps_get_free_size(MALLOC_CAP_INTERNAL)` and
  `MALLOC_CAP_SPIRAM` around `init`/`start`.

## Testing

- **Host (linux target):** `host_test/` — Unity suites for the schema validator,
  the envelope/CRC codec, AND the registry/lifecycle logic (reboot-to-apply,
  defaults override, migration, boot fallback, write dedup) — the registry runs
  against an in-memory storage stub (`test_storage_stub.c`). No board needed.
  ```
  cd host_test
  idf.py --preview set-target linux
  idf.py build
  ./build/settings_manager_host_test.elf
  ```
- **On-target (pytest-embedded):** `test_apps/` — drives validation accept/reject,
  persistence across restart, and CRC rollback-on-corruption against real
  LittleFS. Run with `pytest --target esp32 test_apps/pytest_settings_manager.py`
  (QEMU where possible, hardware otherwise). No external instruments required, so
  no `test_apps/README.md` is needed.

## Files

| File                            | Responsibility                              | Host-testable |
|---------------------------------|---------------------------------------------|:-------------:|
| `settings_manager.c`            | registry, lifecycle, get/set/list           |  ✓ (vs stub)  |
| `settings_manager_schema.c`     | JSON Schema validator                       |       ✓       |
| `settings_manager_codec.c`      | envelope + CRC-32                           |       ✓       |
| `settings_manager_storage.c`    | LittleFS mount + atomic file IO             |               |
| `include/settings_manager.h`    | public API                                  |               |
| `settings_manager_private.h`    | internal shared declarations                |               |
