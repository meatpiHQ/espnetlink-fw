# MeatPi Firmware Component Coding Standard — rev 1.0

Applies to all firmware components in every MeatPi product. Written for both human and AI contributors. Keep it boring and consistent.

**Provenance & versioning.** Rev 1.0 of this document is the WiCAN Firmware Component Coding Standard rev 2.5, made product-agnostic. This document versions independently from any product's standard; a product repo may carry a short addendum for product-specific rules, but where the two conflict without an explicit addendum entry, this document wins. Rules that came from hard-won field failures are marked as such — do not relax them without equivalent evidence.

**Targets:** Espressif chips under **ESP-IDF, one version pinned per product repo** — the exact version the product builds and tests with, recorded in the repo README and CI (WiCAN pins v6.0.2). Chip families vary per product: on chips **with PSRAM** (e.g. ESP32-S3 with octal PSRAM) the §2 PSRAM rules apply in full; on chips **without PSRAM** (e.g. ESP32-C3) they degrade to internal-RAM-only; everything else applies unchanged. IDF v6 gotcha that bites every product: the bundled `json` component was removed — depend on the **`espressif/cjson` managed component** (public `REQUIRES` only when a public header exposes `cJSON`). The canonical minimal config is the repo-root **`sdkconfig.defaults`** (extracted with `idf.py save-defconfig`); per-chip overlays (`sdkconfig.<chip>`) sit next to it; test apps inherit from it per §7.

**Core platform components.** The standard assumes each product composes the shared MeatPi core: `log_manager` (§9), `settings_manager` (§4–§5), `cmdline_manager` (§6b), and an HTTP server manager exposing the settings API (§6). These components are product-agnostic by design — reuse them across products, don't rewrite them per device.

**Product profile.** Each product repo declares its hardware/toolchain facts in one place (repo README or a `PRODUCT.md`): MCU + PSRAM/flash sizes, the pinned ESP-IDF version, the canonical `sdkconfig.defaults`, and the partition table path. This standard refers to those facts generically; nothing below hard-codes a product.

## 1. Language & Style

- **C (C11)**, ESP-IDF. C++ only where a dependency forces it.
- **Allman brace style**, no exceptions:

```c
esp_err_t comp_do_thing(int arg)
{
    if (arg < 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}
```

- 4-space indent, no tabs. Lines ≤ 100 cols.
- Naming: `snake_case` for functions/vars, `UPPER_SNAKE` for macros/constants, component prefix on all public symbols (e.g. `batt_mon_*` for a `battery_monitor` component). Prefixes must be unique across the whole firmware — check before picking one.
- Header guards: `#pragma once`. Include order in `.c` files: own header first, then C stdlib, then IDF/FreeRTOS, then other components, then private headers. Own-header-first catches missing includes in the public header.
- One component = one directory. Public API in `include/<component>.h`; everything else is `static` or private.
- **≤ 700 lines per file.** Over that, split by responsibility (e.g. `comp.c`, `comp_settings.c`, `comp_api.c`) — never split mid-logic just to hit the number.

## 2. Memory

On PSRAM-less chips this section degrades to internal-RAM-only: skip the PSRAM placement rules, keep everything else (DMA, ISR, flash-cache, and stack rules still apply).

- **Default to PSRAM for everything** — task stacks, buffers, queues, large structs, component state. Use internal RAM only when a hard constraint below forces it.
- **Prefer static allocation in PSRAM** over dynamic. Place static/global data in PSRAM `.bss` with `EXT_RAM_BSS_ATTR` (or `EXT_RAM_NOINIT_ATTR` for noinit). Allocate task stacks as static buffers in PSRAM and create with `xTaskCreateStatic`:

```c
static StaticTask_t s_tcb;                            // TCB: internal RAM
static StackType_t  s_stack[8192] EXT_RAM_BSS_ATTR;   // stack: PSRAM
xTaskCreateStatic(comp_task, "comp", 8192, NULL, 5, s_stack, &s_tcb);
```

  Required Kconfig (PSRAM parts): `CONFIG_SPIRAM=y`, `CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY=y` (for `EXT_RAM_BSS_ATTR`), `CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y` (for PSRAM task stacks), and `CONFIG_SPIRAM_ALLOW_NOINIT_SEG_EXTERNAL_MEMORY=y` if `EXT_RAM_NOINIT_ATTR` is used.

- **Must stay in internal RAM** — these are the only acceptable reasons to leave PSRAM:
  - **DMA buffers** (TWAI/CAN, SPI, I2S, USB, …). Static: `DMA_ATTR`. Dynamic: `MALLOC_CAP_DMA`.
  - **Anything reached with the cache disabled** — data/code touched by an IRAM ISR (`ESP_INTR_FLAG_IRAM`), or accessed during flash/OTA/LittleFS writes. PSRAM is unreachable when cache is off, so this is a crash, not a perf issue.
  - **A task whose path hits the above** (IRAM ISR handler, cache-disabling work) → its stack stays internal too. **Corollary: a task running on a PSRAM stack must never perform flash writes directly** — hand the work to a task with an internal stack. (This is why the Settings Manager's writer runs on an internal stack, §4.)
  - **Hot, latency-sensitive paths** where PSRAM access cost is measured and matters.
- When you use internal RAM, say why in a one-word comment: `// internal: DMA`.

**PSRAM `.noinit` (reboot-surviving state).** `EXT_RAM_NOINIT_ATTR` data survives
warm resets (esp_restart / panic / watchdog) but is random after a power cycle,
so it MUST sit behind a magic/version/CRC envelope with `esp_cache_msync`
write-back after every mutation (reference implementation: WiCAN's
`restart_tracker`). Two ESP32-S3 hardware gotchas, both found the hard way:

- **`CONFIG_SPIRAM_MEMTEST` must be off** — the boot memtest writes ALL of
  PSRAM and wipes `.noinit`.
- **The first 64 bytes of `.ext_ram_noinit` are clobbered every boot** by the
  ESP32-S3 MSPI PSRAM timing tuning (test pattern at PSRAM physical address 0,
  `MSPI_TIMING_TEST_DATA_LEN`). Every noinit structure MUST therefore begin
  with a sacrificial `uint8_t mspi_tuning_guard[64]` excluded from its CRC,
  and multi-part data (header + buffer) must be ONE object so the buffer can't
  independently link at address 0.

## 3. Component API

- Every component exposes a single public header declaring its API for main firmware and other components.
- Lifecycle is explicit and uniform:

```c
esp_err_t <comp>_init(void);     // register, allocate, no side effects on bus
esp_err_t <comp>_start(void);    // begin operation
esp_err_t <comp>_stop(void);
```

- **No `_deinit`.** Components are init-once for the life of the firmware; `stop()` is the only teardown. `_deinit`/`_free` functions add untested paths and invite use-after-free. Exception: host test builds may provide a test-only reset so the Unity harness can re-init between cases — keep it out of the public header, guarded by the host-test build flag.
- Return `esp_err_t`. No silent failures, no `abort()` in library code.
- **No panic paths anywhere in production code.** `ESP_ERROR_CHECK`,
  `ESP_ERROR_CHECK_WITHOUT_ABORT`-then-`abort()`, `assert()` on an `esp_err_t`,
  or any construct that panics on a recoverable failure is forbidden in
  components **and in `main`**. A deterministic init failure under
  `ESP_ERROR_CHECK` is a **permanent boot loop** — the one thing a field
  device must never do. Instead: log at ERROR and continue degraded. The
  composition root uses log-and-continue helpers (`init_step` /
  `start_component` in `main.c`); a component whose init failed simply stays
  unconfigured and its `_start()` refuses per §4.3 step 5, while the rest of
  the device keeps booting (serial console, and usually the AP + OTA recovery
  path, stay alive). `ESP_ERROR_CHECK` remains acceptable in **test apps and
  host tests only** — there, failing loudly is the point. (`assert()` on
  programmer invariants that cannot depend on hardware/input state — never on
  return codes — is tolerated but discouraged.)
- A component left unconfigured at boot (§4, defaults also rejected) must refuse `_start()` with `ESP_ERR_INVALID_STATE` rather than run half-configured.
- No cross-component access to internals — talk through public APIs only.

**Dependencies.** Components may depend on other components:

- Declare every dependency in `CMakeLists.txt`. Use `PRIV_REQUIRES` when only the `.c` files use it; use `REQUIRES` only when the public header exposes the dependency's types. Favour `PRIV_REQUIRES` to keep the graph shallow.
- Depend on a component's **public header only**, never its internals.
- **No circular dependencies.** If two components need each other, that's a design smell — extract the shared part into a lower component, or invert it with registration/callbacks (the way components register with the Settings Manager and Log Manager instead of those depending on them).
- **Dependencies flow one direction:** core (log manager, settings manager, storage, HAL) ← services ← app. A lower layer never depends on a higher one.
- A component assumes its dependencies are already `*_init()`'d; it does **not** init or start them itself. `main` composes the system and brings components up in dependency order: `log_manager` first (§9.6), then `settings_manager`, then everything else.
- Keep pure logic free of heavy deps so it stays host-testable (§7); inject or stub the dependency where practical.

**Every component ships a `README.md`** at its root (`components/<comp>/README.md`) containing:

- **Summary** — one short paragraph: what it does, where it sits.
- **API** — the public functions/types from the header, one line each. Keep in sync when the header changes.
- **Dependencies** — which components it requires, and any required init order.
- **Settings** — schema keys and defaults, if it has any, plus the current schema `version`.
- **Memory footprint** — internal RAM and PSRAM, split into static (`.bss`/`.data`), task stacks (count × size, and where), and peak heap. Mark each number **estimated** or **measured**.

Measure, don't guess where you can: `idf.py size-components` for static, `uxTaskGetStackHighWaterMark()` for real stack use, and `heap_caps_get_free_size(MALLOC_CAP_INTERNAL)` / `MALLOC_CAP_SPIRAM` deltas around init for heap. Estimates are fine to start — replace with measured before release.

## 4. Settings

Don't give each component raw `store()` / `load()` callbacks. That scatters persistence logic, duplicates storage code, and makes schema validation inconsistent.

**Instead: the Settings Manager owns persistence. Components register a descriptor. Settings apply only at boot — changing settings means persist, then reboot.**

### 4.1 Descriptor and API

```c
typedef struct
{
    const char *name;           // component id; settings key + on-disk filename
    uint32_t    version;        // schema version, for migrations

    const settings_field_t *fields; // field-table schema: the ONLY schema
    size_t field_count;             // authoring format for components

    const char *defaults_json;  // OPTIONAL whole-object override of the
                                // schema-derived defaults; NULL for most components

    // Called ONCE, at boot, in the Settings Manager's init context,
    // before any component's _start(). Never called at runtime.
    // On failure the manager retries once with defaults (§4.3).
    esp_err_t (*on_apply)(const cJSON *settings);

    // Optional: reject bad input before commit. err buffer for UI/API.
    // Called at set() time and against stored data at boot.
    esp_err_t (*on_validate)(const cJSON *settings, char *err, size_t err_len);

    // Optional: migrate stored settings from an older schema version.
    // Mutates `settings` in place. Called ONCE with the stored version;
    // the component handles ANY from_version it has ever shipped.
    esp_err_t (*on_migrate)(uint32_t from_version, cJSON *settings);
} settings_descriptor_t;

esp_err_t settings_manager_register(const settings_descriptor_t *desc);
esp_err_t settings_manager_get(const char *name, cJSON **out);      // caller frees
esp_err_t settings_manager_set(const char *name, const cJSON *in);  // validate + persist ONLY
```

**Schema authoring: the field table, always.** Components author
their schema as a `settings_field_t` table — one row per key, `SETTINGS_STR` /
`SETTINGS_STR_LEN` / `SETTINGS_STR_ENUM` / `SETTINGS_STR_FILE` /
`SETTINGS_INT` / `SETTINGS_BOOL` macros (`_REQ` variants mark a key required —
meaningful inside array items). The manager generates the equivalent JSON
Schema once at registration and rejects malformed tables
(`ESP_ERR_INVALID_ARG`) at the call site. Validation, defaults, and
`GET /settings/<name>/schema` all run on the generated JSON Schema — the wire
format is unchanged. The descriptor's raw `schema` string member still exists
but is reserved for the Settings Manager's own tests; **components must not
hand-write JSON Schema strings.**

Bounded **arrays of objects** are expressed in the table too: `SETTINGS_ARRAY`
takes a nested item-field table (scalar rows only — one nesting level, matching
the validator; `maxItems` is required, 1..16), `SETTINGS_ARRAY_ANY` takes
free-form object items (pair it with an `on_validate` that enforces the shape).
Array defaults are JSON array literals written **unescaped** through
`SETTINGS_JSON(...)` — the preprocessor stringizes it, so no `\"` soup. One
gotcha: every JSON string inside `SETTINGS_JSON` must stay a single C token
(adjacent-literal concatenation happens after stringization and corrupts the
JSON).

```c
static const settings_field_t SERVER_ITEMS[] =
{
    SETTINGS_STR_REQ     ("name",  1, 15, ""),
    SETTINGS_STR_ENUM_REQ("proto", "tcp,udp", "tcp"),
    SETTINGS_INT_REQ     ("port",  1, 65535, 3333),
    SETTINGS_BOOL        ("enabled", false),
};

static const settings_field_t FIELDS[] =
{
    SETTINGS_STR_ENUM("mode",       "off,sta,ap,apsta", "apsta"),
    SETTINGS_STR_LEN ("ap_password", 8, 64, "12345678"),
    SETTINGS_INT     ("ap_channel",  1, 13, 6),
    SETTINGS_BOOL    ("ap_auto_disable", false),
    SETTINGS_ARRAY   ("servers", 4, SERVER_ITEMS,
        SETTINGS_JSON([
            {"name":"obd0","proto":"tcp","port":35000,"enabled":true}
        ])),
};
```

**File placement: `<comp>_settings.c`.** Every component with
settings keeps ALL of its settings code in a dedicated `<comp>_settings.c`:
the field table(s), the descriptor, `on_apply` / `on_validate` / `on_migrate`,
and the boot-applied config those hooks fill. The file exposes
`<prefix>_settings_register(void)` (called from `<comp>_init()`) plus
`const`-returning getters for the applied config
(`<prefix>_settings_config()` / `<prefix>_settings_is_configured()`;
reference implementations: WiCAN's `wifi_manager` and `socket_manager`).
State the component mutates at runtime stays in the engine files; the
settings file's state is written only by its hooks. This makes every
component's keys and defaults findable in one predictable place.

### 4.2 Apply model: reboot-to-apply

- `settings_manager_set` = schema validate → `on_validate` → persist. **It never calls `on_apply`.** There is no live-reconfiguration path; components contain zero reload code.
- `on_apply` runs **once per component, at boot, single-threaded**, in the Settings Manager's init context, before any `_start()`. No threading contract is needed: it cannot race the component's own tasks because they don't exist yet.
- User flow: the UI collects edits locally; **Submit changes** sends them to the device; the device validates and persists each component's object, responds to the UI, then reboots itself. The transport must delay `esp_restart()` (≈1 s) so the HTTP response flushes — otherwise the UI sees a dropped connection instead of confirmation.
- **No-op submits skip the reboot.** If every submitted object is byte-identical to what's persisted (§4.4 dedup), nothing was written and the device must not reboot.
- Ephemeral runtime knobs (e.g. live log level, §9.4) are **not settings**. A component may expose its own runtime API for them; persisted settings still only apply at boot, and runtime knobs reset to persisted defaults on reboot.

### 4.3 Boot sequence and failure handling

At boot, for each registered component, the manager:

1. Loads the stored object. Missing or corrupt (CRC fail) → build defaults (schema `default` keywords, or `defaults_json` if set), persist them, log a WARNING.
2. If stored `version` < descriptor `version` → call `on_migrate`, re-validate the result against the schema, persist. Migration absent or failed → defaults, log an ERROR.
3. Validate against the schema and `on_validate`. Invalid → defaults, log a WARNING.
4. Call `on_apply`. **If it fails:** persist defaults, call `on_apply(defaults)` once more, and log an ERROR through the Log Manager. This covers data that is schema-valid but unachievable on the hardware (e.g. a bitrate this crystal can't hit).
5. If **defaults also fail apply**, that's a component bug, not bad data: log at ERROR, leave the component unconfigured, and continue boot. The component's `_start()` must then refuse with `ESP_ERR_INVALID_STATE`. One broken component must never brick the device; retry loops and reboot-on-failure are forbidden here — they boot-loop.
6. `GET /settings/<name>` responses include `"degraded": true` when step 4 or 5 occurred, so the UI can surface it after the reboot that triggered it.

### 4.4 Storage guarantees (normative)

The Settings Manager is the only code that touches settings storage. These behaviors are requirements — do not "simplify" them away:

- **Owns the filesystem.** It decides the FS type (LittleFS today), mounts it, and owns the layout. No other component mounts or touches the FS for settings, so the FS can change without touching any component.
- **Dedicated partition.** Settings live on their own partition, independent of the general `filesystem` component, so a corrupted user filesystem can never take down settings.
- **Atomic writes**: write to a temp file, sync, rename over the target. A power cut mid-write must leave either the old object or the new one — never a torn file.
- **Integrity**: every stored object carries a CRC-32 (polynomial `0xEDB88320`); a mismatch at load is treated as missing (§4.3 step 1).
- **Write deduplication**: `set` with content identical to what's persisted returns `ESP_OK` without writing (the `persisted_clean` guard). This protects flash from redundant wear and is what makes the no-op-submit reboot skip (§4.2) work.
- The manager's **writer task runs on an internal-RAM stack** (§2 corollary: flash writes from a PSRAM stack are forbidden).

## 5. Settings format

- Stored as **JSON**, one object per component keyed by `name`.
- The **JSON Schema is the single source of truth** for shape, types, ranges, and defaults (via `default` keywords) — generated from the component's field table at registration (§4.1). `defaults_json` exists only for the rare component that needs a whole-object override; when present it replaces the schema-derived defaults wholesale.
- Bump `version` on any breaking schema change and extend `on_migrate` to handle every `from_version` the component has ever shipped. Non-breaking additions (a new optional key with a schema `default`) don't need a bump — validation fills the gap from the schema.

## 6. Remote settings API

- HTTP / TCP / WebSocket access is **optional per component** but goes **through the Settings Manager**, not bespoke per-component storage code.
- The manager exposes generic endpoints; the transport layer is shared:
  - `GET  /settings/<name>` → current JSON (plus `"degraded": true` per §4.3)
  - `PUT  /settings/<name>` → validate + persist (returns the `on_validate` error message on reject)
  - `GET  /settings/<name>/schema` → schema, so UIs/tools self-describe
- **PUT is full-object replace.** The client sends the complete object; the manager validates it against the whole schema (including `required`). No merge-patch: the UI already holds the full object from `GET`, and full-replace means validation always sees a complete document. Partial updates failing schema validation is the correct outcome.
- After the last successful PUT of a submit batch, the transport responds, then reboots the device (§4.2). If nothing changed, it responds without rebooting.
- A component that needs custom transport behavior implements only its domain handler; it must still route persistence through `settings_manager_set`.
- **Every HTTP route is documented, in the same change that adds it.**
  `components/HTTP_API.md` owns the conventions and the central route map; a
  component adding or changing any `/api/*` (or other reserved-namespace)
  route updates that map **and** its own per-endpoint reference (its
  `HTTP_API.md`, or its `README.md` for a trivial single route) in the same
  commit. The web UI is built against these files — an undocumented route
  does not exist as far as the product is concerned.

## 6b. Console commands (CLI)

Console commands follow the same ownership inversion as HTTP routes and
settings: **`cmdline_manager` owns the registry and the transports;
components own their commands.**

- A component with console commands ships them in `<comp>_cli.c` with a
  `<comp>_register_cli()` that registers via `cmdline_manager_register()`
  (esp_console semantics; `help`/`hint` strings static — they feed the
  legacy-format `help`). Handlers print with `cmdline_printf()` /
  `cmdline_write()` and run in the dispatcher/console context — treat the
  §2 corollary as binding (exec_line callers have INTERNAL stacks; a
  handler may therefore touch flash/LittleFS).
- **The component registers itself**: `<comp>_register_cli()` is called
  from the component's own settings `on_apply`, gated by a **`cli` bool
  field (default true)** in its settings — reboot-to-apply, like every
  setting. `main` wires no per-component CLI calls. Registration failure
  is log-and-continue, never an `on_apply` failure.
- A component with no other settings still gets the knob: a minimal
  `{cli}` field-table descriptor. If its `init` runs before
  `settings_manager_init`, expose `<comp>_register_settings()` for main
  to call right after settings init (the `log_manager_register_settings`
  pattern).
- Components BELOW `cmdline_manager` in the graph (log_manager,
  settings_manager) cannot register — that would be a cycle; their knobs,
  composites spanning multiple components (`system`), and pending stubs
  live in main's `main_cli.c`.
- **Option parity on ports**: when a product replaces an earlier firmware,
  commands that existed there keep their option interfaces byte-alike
  (outputs + `OK` trailers); bare invocation gives a new-firmware summary;
  additions are options.
- Every command lands in `cmdline_manager/README.md`'s ownership table in
  the same change — the CLI analogue of the §6 HTTP-route rule.

## 7. Testing

**Every component ships tests.** Two layers, both required where they apply:

- **Unit (Unity, C):** pure logic — schema validation, settings serialize/deserialize, frame encode/decode, parsers, state machines. Run on the **host (`linux` target)** so CI needs no hardware and they're fast. Live in `components/<comp>/test/` (or `host_test/`).
- **pytest (pytest-embedded):** builds/flashes a test app and asserts on DUT serial via `dut.expect(...)`. Covers on-target/integration behaviour and anything needing a real peripheral, flash, or LittleFS. QEMU where possible, hardware where not. Lives in `components/<comp>/test_apps/` with `pytest_<comp>.py`.

Rules:

- Anything that *can* be host-tested *must* be — don't require a board to test a parser.
- Settings: schema + `on_validate` + `on_migrate` get unit tests (valid accepted, invalid rejected with the expected error, every historical version migrates to current). Atomic store, rollback-on-corruption, and the boot-failure fallback (§4.3 steps 4–5) get pytest/target tests.
- No component logic merges without tests covering it. CI runs host unit tests on every push.

**On-device test apps use the main firmware's configuration.** A pytest test app exists to prove the component works **on the shipped device**, so it must build against the main firmware's partition table and sdkconfig — never a private fork of them:

- The test app inherits the main project's sdkconfig (list the main project's defaults file first in `SDKCONFIG_DEFAULTS`, then at most a small test-only overlay), and its `CONFIG_PARTITION_TABLE_CUSTOM_FILENAME` points at the repo-root partition table the main app builds with. Test apps do **not** carry their own `partitions.csv`.
- If the component needs a Kconfig option or a partition the main firmware doesn't have yet (a new data partition, a `CONFIG_SPIRAM_*` option, a bigger `max_uri_handlers`), **change the main firmware's sdkconfig / partition table first**, then let the test app inherit it. The main config is the single source of truth; a test app must never build with config the shipped firmware doesn't have.
- The test-only overlay may contain only options that are meaningless in production (test console/runner options, log verbosity for `dut.expect`), each with a one-line justification comment. Functional config (memory, partitions, drivers, stack sizes) never goes in the overlay.
- Rationale: a test that passes on its own partition layout or config proves nothing about the real device, and config drift between test apps and firmware is where "works in the test app, crashes in the field" comes from.

**External tools / hardware-in-the-loop.** A test may rely on external tooling — PCAN, Ollie v2, bench PSUs, custom jigs, etc. If it does, that test app **must ship a `README.md`** next to it (`components/<comp>/test_apps/README.md`) covering:

- **Tools + versions** — exact instrument and any firmware/driver version (e.g. PCAN-USB + driver, Ollie v2 + its FW rev).
- **Wiring** — physical connections (CAN-H/CAN-L, power, target pins), bus termination, and bitrate.
- **Host setup** — drivers, packages, and the tool's CLI/config needed to drive or observe the DUT (e.g. `python-can` backend, the Ollie command set).
- **Procedure** — bring-up steps, how to launch the test, and the **pass/fail criteria** (expected frames, timing, serial output).

Rule: if a human or AI can't reproduce the bench from that README alone, it's incomplete. Tests that silently assume a tool is present and configured are not allowed.

## 8. Contribution rules (human + AI)

- Match this document over any habit or training default. If a change conflicts with it, the document wins.
- New component → new directory containing: public header, `CMakeLists.txt`, `idf_component.yml`, `README.md` (§3 contents), lifecycle funcs (`init`/`start`/`stop` — no `_deinit`), settings in `<comp>_settings.c` if it has any (field-table schema + descriptor + hooks, §4.1), log descriptor (§9.2), CLI commands (if any) per §6b (`<comp>_cli.c`, self-registered on settings apply behind the `cli` bool, documented in cmdline_manager's ownership table), `Kconfig` if it has build-time options, host unit tests + a pytest test app built on the main firmware's partition table and sdkconfig (§7).
- No new file over 700 lines. No non-Allman braces. No settings persistence outside the Settings Manager. No hand-written JSON Schema strings — field tables only, in `<comp>_settings.c` (§4.1). No log routing outside the Log Manager. No logging from ISRs.
- No `ESP_ERROR_CHECK`/panic paths in components or `main` (§3) — log and degrade; test apps only. New/changed HTTP routes update `components/HTTP_API.md` + the component's endpoint reference in the same change (§6).
- Default to static allocation in PSRAM (on PSRAM parts). Internal RAM only for the cases in §2, justified by comment. No flash writes from PSRAM-stack tasks.
- Public API or schema change → note it in the component header; if the persisted shape changed, bump `version` **and** extend `on_migrate` **and** add the migration unit test.

## 9. Debug Log Manager

The Log Manager owns log routing the same way the Settings Manager owns persistence: **one component owns the pipeline; everything else registers into it.** No component writes to a socket, file, or ring buffer directly for logging.

### 9.1 Capture: hook, don't migrate

The manager installs itself via `esp_log_set_vprintf()`. Components keep using plain `ESP_LOGx(TAG, ...)` — no new logging API, no call-site migration, and IDF-internal / third-party logs are captured too. Do not invent wrapper macros.

### 9.2 Component registration

```c
typedef struct
{
    const char      *name;           // must equal the component's TAG (§10)
    esp_log_level_t  default_level;  // level at boot before settings apply
} log_descriptor_t;

esp_err_t log_manager_register(const log_descriptor_t *desc);
esp_err_t log_manager_set_level(const char *name, esp_log_level_t level); // runtime, ephemeral
```

Registration builds the per-component level table, so an operator can set `can` to DEBUG while everything else stays at INFO — at runtime, without a reboot. This is the sanctioned exception in §4.2: a runtime knob, not a setting.

### 9.3 Sinks

```c
typedef struct
{
    const char *name;
    esp_err_t (*write)(const char *line, size_t len);  // called from the log task only
} log_sink_t;

esp_err_t log_manager_add_sink(const log_sink_t *sink);
```

- The **console (UART) sink is built in and always available from `log_manager_init()`.**
- Multiple sinks may be active simultaneously (console + TCP + flash ring buffer), each independently enabled/disabled at runtime.
- A sink's `write` runs in the log task's context, never in the producer's — a slow sink can therefore never stall a producer.

### 9.4 Settings vs runtime knobs

The Log Manager registers its **own settings descriptor** (§4) for persisted defaults: per-component boot levels, which sinks auto-enable, sink parameters (TCP port, ring size). Those apply at boot like all settings. `log_manager_set_level()` and runtime sink enable/disable are **ephemeral** — they reset to persisted defaults on reboot. Turning on DEBUG to chase a bug must not require a reboot, and must not survive one.

### 9.5 Pipeline, backpressure, and context rules

- Producers format into a fixed-depth message queue; a dedicated log task drains it to the sinks.
- **Backpressure = drop-oldest, never block.** The producer path must be wait-free: if the queue is full, the oldest entry is dropped and a drop counter increments; the manager emits a `dropped N messages` line when pressure clears. Blocking a CAN task on a stalled TCP client is not acceptable; losing old debug lines is.
- **No logging from ISR context.** `ESP_LOGx` from an ISR is forbidden (it takes locks, and per §2 an IRAM ISR can run with the cache off). An ISR that must report posts a value to a queue; a task logs it.
- **Memory placement:** queue and buffers live in PSRAM per §2. The log task performs no flash writes itself; a flash-backed sink hands writes to an internal-stack writer path (§2 corollary).
- A **flash-backed crash ring buffer** (retrieve last N lines after reboot) is planned but out of scope for v1; when built, its writes follow §4.4-style atomicity.

### 9.6 Boot order

`log_manager_init()` is the **first** component init in `main` — before `settings_manager_init()` — so settings-apply errors (§4.3) have somewhere to go. Until then, output falls through to the raw IDF console (default vprintf); no early-boot buffering is required. The manager starts with compile-time defaults; its own `on_apply` later in boot adjusts levels and sinks to the persisted configuration.

## 10. Logging conventions

- One `TAG` per component, equal to the component/directory name exactly: `static const char *TAG = "settings_manager";`. This is what makes §9.2's per-component level control line up.
- Levels: `ESP_LOGE` only for failures that are also returned/propagated as errors; `ESP_LOGW` for fallback events (defaults applied, migration failed, messages dropped); `ESP_LOGI` for lifecycle milestones (started, connected) — a handful per boot, not per operation; `ESP_LOGD`/`ESP_LOGV` for everything chatty.
- **No logging in hot paths at INFO or above** (per-frame CAN RX, per-request HTTP). Use DEBUG so it filters out in normal operation.
- **Retry/reconnect loops log at DEBUG too.** Anything that repeats on a timer while a condition persists (WiFi reconnect attempts every 5 s, per-request 404s, client aborts) floods the pipeline at INFO/WARN. Log the state *transition* once at INFO ("connected", "disconnected"), the *attempts* at DEBUG.
- **Why this matters mechanically:** producers never block on a sink — the Log Manager queues and drops-oldest (§9.5) — so a burst can't hang the device, but it CAN evict the lines you actually needed and (pre-`log_manager_start()`, when routing is synchronous to the UART) slow early boot. Sinks differ in tolerance: UART at 115200 sustains ~1 kline/min, TCP/UDP more, the PSRAM ring is fast but small. The level rules above are what keep every sink usable.
- **Every component registers a `log_descriptor_t`** in its `init` (§9.2) so per-TAG runtime levels work. Exception: `settings_manager` (the Log Manager depends on it for its own settings descriptor — registering back would be a cycle); `main` registers `{"settings_manager", ESP_LOG_INFO}` during composition.
- Never log secrets (WiFi passwords, tokens, API keys) at any level.
