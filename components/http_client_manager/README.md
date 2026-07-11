# http_client_manager — the HTTP(S) client owner (service)

Rewrite of the legacy `https_client_mgr`: the one way components do
HTTP-client work — synchronous request/response, TLS via cert_manager
sets, auth helpers, and atomic download-to-filesystem. Consumers:
autopid cloud posts, web-asset fetch-on-miss, future integrations.

## Thread-safety model (the point of the "manager")

Every request builds its own `esp_http_client` instance on the CALLER's
task — zero shared mutable state between requests — and a counting
semaphore caps concurrency at 4 so many tasks can't exhaust sockets/TLS
heap (excess callers briefly block). Any task may call any API.

**Caller contract** (each clause was earned on the bench):

- Calls are SYNCHRONOUS (seconds under bad networks): ordinary tasks
  only — never ISRs, timer callbacks, or handler contexts that must not
  block (mqtt/imu/battery callbacks included).
- `https://` runs the mbedTLS handshake on YOUR stack: budget **≥8 KB**
  (a 4 KB caller stack-overflowed and panicked — live-found).
- `download()` is stack-agnostic: ALL media writes run on filesystem's
  internal-stack async-pipe writer task, so PSRAM-stack callers are
  fine. (The first, staged implementation crashed a PSRAM-stack caller
  in the spi_flash cache-off assert — the pipelined design removed the
  hazard instead of guarding it.)

## API

| Call | Behavior |
|---|---|
| `http_client_manager_init()` | Log descriptor + the limiter. |
| `_request(req, *out)` | Full-control request: method, body, content type, auth, extra headers, TLS options, timeout, response cap. HTTP 4xx/5xx = ESP_OK with `status_code` set; transport failure = error. |
| `_get(url, *out)` / `_post(url, body, len, ct, *out)` | Conveniences. |
| `_download(url, save_path, progress_cb)` | **STREAMED, no size limit** beyond free space: the caller receives into a PSRAM buffer and feeds filesystem's ASYNC PIPE (2×32 KB slots + its internal-stack writer task) — network RX and media writes overlap (meatpi's "fast writing while continuously reading"). Atomic via the temp+rename stream — a broken download never leaves a torn file. One at a time. **Measured: 8 MB → /sd in 16.2 s (505 KB/s, WiFi-bound; SD writes fully hidden), md5-identical round trip.** |
| `_free(resp)` | Release the PSRAM response buffer. |
| `_stats(*out)` | requests / failures / bytes_rx. |

Responses are capped (default 16 KB, `max_response` up to 256 KB) —
the legacy read unbounded bodies to heap; this one can't be OOM'd by a
fat endpoint.

## TLS (priority: `cert_set` > `ca_pem` > bundle)

`cert_set` names a cert_manager set: its CA verifies the server, and a
client cert+key pair (when present) is offered = mutual TLS — the same
model as mqtt_manager. **Live-verified 2026-07-04** against the bench
TLS endpoint (socat + bench CA on rpi001:8443): with `cert_set` →
status 200; without → correctly refused (the bundle doesn't trust the
bench CA). `skip_common_name` exists for rare devices with broken certs.

## Auth helpers (pure, host-tested 5/5 incl. RFC vectors)

`bearer_token` → `Authorization: Bearer …`; `basic_username/password` →
`Authorization: Basic base64(u:p)` (own tiny base64, RFC 4648 vectors);
`api_key`(+`api_key_header`, default `x-api-key`); plus pre-formatted
`extra_headers`. The legacy's query-string builders and custom signer
were dropped in v1 — build the URL yourself; ask if a signer consumer
appears.

## Verified live (2026-07-04, driven via the MQTT bench surface)

GET → `{"status":200,"len":33,…}`; 100 KB download → `/data`,
**byte-identical md5** round trip; HTTPS cert_set both ways (above).
Bench fixtures on rpi001: `wican-http-8080` (python http.server) and
`wican-tls-8443` (socat TLS front) systemd units, files in
`~/wican-http`.

## Files

- `http_client_manager.c` — request engine, limiter, TLS, download.
- `http_client_manager_auth.c` — PURE url/base64/auth/header helpers.

## Memory (verified against the main sdkconfig)

Nearly everything is PSRAM: response buffers and the 2×32 KB download
ping-pong buffers are explicit PSRAM; mbedTLS sessions (~35–45 KB
per TLS connection incl. the 16 KB/4 KB record buffers) go external via
`CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC=y`; esp_http_client internals follow
`CONFIG_SPIRAM_USE_MALLOC` (≥128 B → PSRAM). Internal RAM: only lwIP's
per-socket machinery (a few KB) — worst case bounded by the 4-slot
limiter at ~10–15 KB transient, inside the §12b both-radios envelope.
One task of its own: the 4 KB internal-stack download writer (owns the
flash/SD writes); requests run on the CALLER's stack (see the caller
contract above).
