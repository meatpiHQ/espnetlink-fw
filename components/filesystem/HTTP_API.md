# filesystem — HTTP API reference

> **Implemented (2026-07-03; FULL file manager 2026-07-04)** by the
> `api_http` glue (`api_http_fs.c`) — live-verified round trip (upload →
> list → byte-identical download → delete → 404).
> Conventions: `components/HTTP_API.md` §1.
> The write surface (upload/delete/mkdir) was un-deferred by meatpi
> 2026-07-04 for the UI file manager — v1 trust model is the AP-mode setup
> UI (no auth yet); revisit rate limits/auth with the auth story. The
> settings partition is NOT reachable through this API (separate partition,
> separate owner — fault isolation holds).

## GET /api/fs/list?path={logical-path}

Enumerate a directory. Exists chiefly so the settings UI can implement the
schema `format:"file"` picker (a file-typed setting shows a dropdown of
candidates).

**Query**: `path` — a logical filesystem path (`/data/...` internal flash,
`/sd/...` SD card). Validated by the component's own `fs_path_resolve` rules
(no `..`, no `//`, known prefix, < 128 chars).

**Response 200**
```json
{
  "path": "/data/web",
  "entries": [
    { "name": "icons",      "dir": true,  "size": 0 },
    { "name": "index.html", "dir": false, "size": 4823 }
  ]
}
```

**Errors**
| Code | Body | When |
|---|---|---|
| 400 | `{"error":"invalid path"}` | fails path validation |
| 404 | `{"error":"not found"}` | directory doesn't exist |
| 503 | `{"error":"backend unavailable"}` | `/sd/...` while no card is mounted |

## GET /api/fs/info?path={prefix}

Capacity of the backend owning the prefix (for a storage gauge in the UI).

**Query**: `path` — `/data` (internal flash) or `/sd` (once external_storage
exists).

**Response 200**
```json
{ "total": 6029312, "used": 32768 }
```
Bytes. **Errors**: 400 invalid path; 503 backend unavailable.

## GET /api/fs/download?path={file}

Download any file (logs, configs, captures) — streamed in 1 KB chunks with
`Content-Type: application/octet-stream` and
`Content-Disposition: attachment; filename="<basename>"` so the browser
saves it. **Errors**: 400 invalid path / 404 not found.

## POST /api/fs/upload?path={target-file}

Upload a file of any type to `path`. Two body forms, auto-detected:
`multipart/form-data` (the HTML `<input type=file>` path — the first file
part is taken) or a raw body. The image is staged in PSRAM (limit **2 MB**
per file) and committed with `filesystem_write` — **atomic** (temp+rename):
a power cut mid-upload never leaves a torn file; missing parent directories
are created.

**Response 200** `{"ok":true,"size":36,"path":"/data/ui/testfile.txt"}`
**Errors**: 400 invalid path/no file part/interrupted; 413 over the 2 MB
limit; 503 backend unavailable.

## DELETE /api/fs/file?path={file-or-empty-dir}

**Response 200** `{"ok":true}`; 404 when missing.

## POST /api/fs/mkdir?path={dir}

Creates the directory and any missing parents. **Response 200**
`{"ok":true}`.

## Flash-wear note

Uploads/deletes are user-initiated (UI clicks) — no polling writer exists
behind this API. Anything automated writing through it must batch and
rate-limit per Coding Standard §2/§9.
