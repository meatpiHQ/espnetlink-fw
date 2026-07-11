# http_server_manager

## Summary

Service component that owns the firmware's single HTTP server (`httpd_handle_t`):
port, socket configuration, and lifecycle. It is content-agnostic — it knows
nothing about dashboards or settings. Components register **API routes**
(`httpd_uri_t` handlers) and **static asset tables** (`http_asset_t`) into it;
the manager serves assets through one wildcard catch-all installed last, so
specific routes always win. Assets can be embedded blobs (`EMBED_FILES`) or
filesystem files (VFS path — SD or internal), with optional **fetch-on-miss**:
a missing file with a `source_url` is downloaded once via an injected fetcher,
then served locally on every later request. See `ARCHITECTURE.md` §8.

## API

`include/http_server_manager.h`:

- `esp_err_t http_server_manager_init(void)` — prepare config; nothing listens yet.
- `esp_err_t http_server_manager_start(void)` — install buffered routes, catch-all last, listen.
- `esp_err_t http_server_manager_stop(void)` — stop and release.
- `esp_err_t http_server_manager_register_uri(const httpd_uri_t *uri)` — one API route. Buffered before start; installed live after start. The struct must outlive the server.
- `esp_err_t http_server_manager_register_handlers(const httpd_uri_t *uris, size_t count)` — batch form.
- `esp_err_t http_server_manager_register_assets(const http_asset_t *table)` — sentinel-terminated asset table. **Pre-start only** (tables are read lock-free by the catch-all). Earlier tables win on conflicts.
- `esp_err_t http_server_manager_set_asset_fetcher(http_asset_fetch_fn_t fn)` — inject the fetch-on-miss downloader (main wires the download component's ensure-present here). NULL disables; misses then 404 with `source_url` logged.
- `esp_err_t http_server_manager_set_request_gate(http_request_gate_fn_t fn)` — a per-request admission gate `bool fn(int sockfd)`. Every registered route (incl. WS pre-handshake) is installed behind a trampoline that consults the gate first; `false` → 403 (WS: refuse before the 101), `true` → chain to the real handler. Content-agnostic — the manager knows nothing about *why*; main wires it to `wifi_manager_http_request_allowed` for the network-trust lockdown. NULL disables. The gate runs in httpd-task context: keep it fast + non-blocking.
- `httpd_handle_t http_server_manager_handle(void)` — escape hatch for APIs needing the raw handle (async WebSocket sends). Do not register handlers through it.

`http_asset_t` fields: `uri` (exact, or a prefix entry: a path ending in `/` plus
`*`, which serves a whole directory), `content_type` (inferred from extension if
NULL), `data_start`/`data_end` (embedded), `fs_path` (file, or directory for a
prefix entry), `content_encoding` (e.g. `"gzip"` for pre-compressed), and
`source_url` (fetch-on-miss origin).

Resolution rule: `data_start != NULL` → embedded; else `fs_path` → filesystem.
Registration validates each entry has exactly one usable source.

## Request pipeline

```
GET /api/ping        → specific registered route            → component handler
GET /                → catch-all → normalize → "/index.html" → table lookup
GET /web/icons/x.svg → catch-all → prefix entry → fs_path + remainder
GET /cached.js       → catch-all → fs miss + source_url → fetcher() → serve
GET /a/../b          → catch-all → normalize rejects traversal → 400
GET /nope            → catch-all → no table entry → 404
```

Serving details: query strings/fragments are stripped before matching; `/` maps
to `/index.html`; responses carry `Cache-Control: public, max-age=3600` and a
weak `ETag` (size+mtime; size-only for embedded), with `If-None-Match` answered
by `304`. Filesystem files stream in 4 KB chunks.

## Dependencies

- `esp_http_server` — **public** (`REQUIRES`); the header exposes `httpd_uri_t`
  and `httpd_handle_t`.
- `filesystem` — **private** (`PRIV_REQUIRES`); all file serving goes through
  `filesystem_open()`, so asset `fs_path` values are logical filesystem paths
  (`/data/...` internal flash, `/sd/...` SD once external_storage exists). The
  manager mounts nothing and assumes `filesystem_init()` ran first (Coding
  Standard §3). A missing file AND an unavailable backend both surface as
  open-failure → fetch-on-miss/404.
- **The download component is not a compile-time dependency.** Fetch-on-miss is
  an injected callback (`set_asset_fetcher`), wired by `main` at composition
  time. This keeps the graph shallow and this component buildable/testable
  before `download` exists.

Init order: after the network stack and any filesystems it will serve from;
routes/assets register between `init` and `start`; `start` after everything has
registered (the catch-all must be installed last).

## Settings

None. (The generic `/settings/*` routes belong to the `settings_http` glue
component, not to this server — see ARCHITECTURE.md §9.)

## Constraints & limitations (read before registering)

- **Asset tables register pre-start only** and are read lock-free afterwards;
  the table memory must be `static const` (lives forever).
- **Fetch-on-miss blocks the requesting httpd worker** until the download
  completes (simple block-then-serve). Large first-hit assets tie up one worker
  socket; with `lru_purge_enable` other requests still proceed. If this bites,
  the alternative (redirect-to-source while caching in the background) is a
  serve-path-only change.
- **GET only** on the catch-all. Other methods on asset URIs 404 through httpd's
  method matching; API routes declare their own methods.
- **ETag is identity-weak** (size+mtime). Two different files with identical
  size and mtime would collide — acceptable for firmware assets, not
  cryptographic.
- **`HSM_PATH_MAX` is 192** — URI + resolved FS path must fit; over-long
  requests are rejected with 400, over-long resolved paths don't match.
- **Traversal is rejected** by substring (`..` anywhere in the URI). Encoded
  traversal (`%2e%2e`) arrives decoded from httpd and is caught by the same
  check.

## Memory footprint

No task of its own beyond httpd's own workers (owned by esp_http_server). All
numbers **estimated** — replace with **measured** before release.

| Region                          | Where           | Size (est.) | Notes                          |
|---------------------------------|-----------------|-------------|---------------------------------|
| Route buffer `s_uris[48]`       | PSRAM `.bss`    | ~1.5 KB     | `EXT_RAM_BSS_ATTR`             |
| Asset table ptrs `s_tables[16]` | PSRAM `.bss`    | 128 B       | `EXT_RAM_BSS_ATTR`             |
| Chunk buffer (per active file)  | internal heap   | 4 KB transient | `// internal: DMA / cache-off during FS read` |
| httpd stack/control             | per esp_http_server config | ~4–8 KB | owned by IDF httpd          |

Without PSRAM (`CONFIG_SPIRAM` off) `EXT_RAM_BSS_ATTR` is empty and the
registries (~1.6 KB) land in internal `.bss` — correct fallback, not a bug.

Measure: `idf.py size-components`; internal-heap delta around a large file GET.

## Testing

- **Host (linux target):** `host_test/` — Unity suite for the pure match logic:
  normalization (query/fragment strip, root→index, traversal + overlong
  rejection), exact/prefix resolution, table precedence, remainder joining,
  MIME inference.
- **On-target (pytest-embedded):** `test_apps/` — fully self-contained over
  **lwIP loopback** (`CONFIG_LWIP_NETIF_LOOPBACK`): the app starts the server,
  registers an API route + embedded + FS + fetch-on-miss assets with a stub
  fetcher, then asserts against `http://127.0.0.1` using `esp_http_client`.
  Covers route-beats-catch-all, embedded body match, FS prefix serving,
  exactly-one-fetch-then-local, 404, and traversal→400. No external network or
  instruments required, so no `test_apps/README.md` is needed.

## Files

| File                              | Responsibility                            | Host-testable |
|-----------------------------------|-------------------------------------------|:-------------:|
| `http_server_manager.c`           | lifecycle, registries, catch-all install  |               |
| `http_server_manager_match.c`     | URI normalize, table resolve, MIME        |       ✓       |
| `http_server_manager_serve.c`     | embedded/FS serving, fetch-on-miss, ETag  |               |
| `include/http_server_manager.h`   | public API + `http_asset_t`               |               |
| `http_server_manager_private.h`   | internal shared declarations              |               |
