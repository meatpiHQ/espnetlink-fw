# multipart_upload — streaming multipart/form-data for httpd (utility)

Field-proven **legacy code, adopted verbatim** (2026-07-04): the WiCAN v3/v4
upload path (`multipart_upload.c`, callback-driven streaming parse with
chunked-transfer decoding) wrapping the vendored
[iafonov/multipart-parser-c](https://github.com/iafonov/multipart-parser-c)
(MIT, `multipart_parser.{c,h}` — restored from the legacy tree, byte-
identical API). Style is grandfathered like other adopted legacy code
(`ble.c` reference, ELM fixtures); NEW code in this repo follows the
Coding Standard.

## What it does

`multipart_upload_handle(req, handlers, user_ctx, cfg)` streams a
`multipart/form-data` POST through your callbacks with **no full-body
buffering** (default 1 KB rx buffer): `on_part_begin` (name / filename /
content-type — return `true` to receive the part), `on_part_data` (each
chunk; a non-OK return aborts), `on_part_end`, `on_finished`. Handles
`Transfer-Encoding: chunked` bodies and boundary-from-body fallback.

## Consumers

- `api_http` → `POST /api/ota/upload` (firmware image into `ota_manager`).
- Future: web-UI file uploads (vehicle profiles, certificates) — same
  handler set, different sink.

## Memory (per request, heap, request-scoped)

rx buffer (1 KB, 2 KB when chunked) + parser state (~boundary length). No
statics; concurrent uploads are bounded by httpd's worker count.
