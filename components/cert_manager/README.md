# cert_manager — TLS certificate sets (service)

Rewrite of the legacy cert_manager: named certificate SETS on the
filesystem — `/data/certs/<set>/` holding up to three PEM parts,
`ca.pem` (server verification), `client.crt` + `client.key` (mutual
TLS). Consumers borrow NUL-terminated PSRAM-cached contents by set name
and never touch files: `mqtt_manager`'s `cert_set` setting today, VPN /
HTTPS clients later.

## API

| Call | Behavior |
|---|---|
| `cert_manager_init()` | Log descriptor only. |
| `cert_manager_start()` | Ensure `/data/certs`, scan sets (≤10). |
| `cert_manager_get(set, part, *pem, *len)` | Borrow a part: NUL-terminated PEM in PSRAM, loaded on first use, cached; `len` = strlen+1 (the length TLS configs want). Pointer valid until the set changes. |
| `cert_manager_list(out, *n)` | Set names + which parts are present. |
| `cert_manager_set_usable(set)` | Exists with at least a CA. |
| `cert_manager_register_http()` | `/api/certs` routes (main wires it). |

## HTTP (HTTP_API.md §6g) — NO read-back by design

| Route | Method | Behavior |
|---|---|---|
| `/api/certs` | GET | `{"sets":[{"name":"bench","ca":true,"cert":false,"key":false}]}` |
| `/api/certs/upload?set=X` | POST (multipart) | the legacy HTML-form path: fields `ca` / `client_cert` / `client_key`, any subset, one request; each part PEM-validated |
| `/api/certs/upload?set=X&type=ca\|cert\|key` | POST (raw) | one PEM body ≤8 KB per request |
| `/api/certs?set=X` | DELETE | whole set |

Multipart parts are STAGED in PSRAM during the parse and stored only
after the request body is fully consumed (the proven api_http_fs
pattern) — flash writes from inside the parse loop crashed the device
(heap fault during a concurrent littlefs read; found on the bench,
2026-07-04).

Key material never leaves the device through this surface. v1 trust
note: the generic `/api/fs` browser can still reach `/data/certs` —
same AP-trust model as the rest of the API; revisit with the auth story.

## Decided semantics

- Set names `[a-z0-9_-]{1,24}` (pure validator — traversal-proof by
  construction, host-tested 6/6).
- Files written via `filesystem_write` (atomic temp+rename); every
  upload/delete rescans and **orphans** (never frees) the affected PSRAM
  cache — borrowed pointers (esp-mqtt's TLS config does not copy) must
  stay valid until reboot; freeing would be a use-after-free at the next
  handshake. Bounded leak per config operation — the legacy-proven trade.
- Legacy on-disk layout AND multipart field names preserved
  (`ca.pem`/`client.crt`/`client.key`; form fields
  `ca`/`client_cert`/`client_key`), so migrated devices keep their sets
  and existing UI form posts keep working.

## Consumer example (how mqtt_manager uses it)

`cert_set` setting non-empty + `mqtts://` URL → CA from the set (and
client cert+key when both present → mutual TLS), falling back to
`ca_file`, then the built-in bundle. **Live-verified 2026-07-04, both
modes**: bench CA + mosquitto TLS :8883 on rpi001 — server-auth
(`cert_set 'bench'`, CA only) AND **full mutual TLS** (`cert_set
'mtls'`: one multipart POST carried ca+client_cert+client_key, the
broker enforced `require_certificate true`, device connected —
client cert presented and CA-validated on both ends).

## Files

- `cert_manager.c` — registry, filesystem I/O, PSRAM cache.
- `cert_manager_policy.c` — PURE names/parts/PEM validation (host-tested).
- `cert_manager_http.c` — the routes.

## Memory

≤10 sets × 3 parts × ≤8 KB PSRAM, loaded lazily (typical: one CA ≈
2 KB); registry ~1 KB PSRAM `.bss`; 8 KB PSRAM upload staging. (estimated)
