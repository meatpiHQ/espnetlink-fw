# http_server_manager — on-target test app

Self-contained: serves on the lwIP **loopback** interface and asserts against
itself with esp_http_client — no network or instruments. Files come through
the `filesystem` component (`/data`); builds against the **main partition
table** (standard rev 2.1 §7). Run:

```powershell
.\test.ps1 target http_server_manager
```

**Erase flash first** when asserting the fetch-on-miss counter: the cached
file persists on LittleFS by design, so a re-run without erasing serves it
locally (`fetches=0`) — which is correct cache behavior, but not the
first-run expectation below.

## What is covered

Specific API route beating the wildcard catch-all, embedded asset serving
with `/` → `/index.html` mapping, filesystem serving through a `/prefix/*`
asset entry, fetch-on-miss (exactly one fetch via the injected stub, second
request served locally, destination written atomically via
`filesystem_write`), 404 for unknown paths, traversal rejected with 400.

## Expected result — serial markers, in this order (clean flash)

```
PING status=200 body={"pong":true}
INDEX status=200 match=1
FSFILE status=200 body=hello-from-fs
MISS1 status=200 body=fetched-content fetches=1
MISS2 status=200 body=fetched-content fetches=1
NOPE status=404
TRAVERSAL status=400
TEST DONE
```

`Failed to post esp_http_server event` log lines are benign (no default event
loop in this app; tracked in CHECKLIST). Last verified green: 2026-07-02 on
WiCAN Pro.
