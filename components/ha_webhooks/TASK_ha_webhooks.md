# TASK — `ha_webhooks` (Home Assistant integration, v6 port)

> Design + implementation plan. Written BEFORE code (meatpi 2026-07-09).
> Authoritative standard: `components/ARCHITECTURE.md` (rev 18),
> `components/COMPONENT_STANDARD.md` (rev 2.4), digest in
> `AGENT_KB/02_ARCHITECTURE_AND_STANDARD.md`. Reference implementation:
> `components/legacy/autopid/autopid.c::autopid_webhook_task` +
> `components/legacy/ha_webhooks/`.

## 1. What the Home Assistant integration actually does

WiCAN's HA integration is a HACS custom component running on the **HA
side**. The end-to-end flow, in order:

1. **Discovery (mDNS).** HA finds the device via `mdns_manager` — the
   byte-exact legacy contract (hostname `wican_<id>`, the wc_mdns TXT
   records). Already shipped in v6; nothing to do here.
2. **Webhook registration (HA → device).** Once found, the HA integration
   creates a webhook on its side and **POSTs that webhook URL to the
   device** at `POST /api/webhook`. The device stores it. `manual_override`
   lets a user pin a URL so the integration's auto-push can't overwrite it.
   On **PRO**, a `urls[]` array carries up to 2 failover targets (e.g. a
   local `http://homeassistant.local:8123/api/webhook/<id>` plus a Nabu
   Casa `https://…ui.nabu.casa/…` cloud URL).
3. **Telemetry (device → HA).** A poster task, every `interval` seconds
   while autopid is enabled and the network is up, builds a JSON payload
   and **POSTs it to the webhook URL(s)** with failover. HA parses it and
   creates/updates sensors.

The device is the HTTP **client** for telemetry (outbound) and an HTTP
**server** for the registration endpoint (inbound). Two directions, one
component.

## 2. The payload contract (must stay byte-compatible)

The HA HACS integration already exists and parses a fixed shape. v6 MUST
produce the same top-level object so existing installs keep working:

```json
{ "config": { … }, "status": { … }, "autopid_data": { "ParamName": value, … } }
```

- **`autopid_data`** — the live value snapshot `{"ParamName": value}`.
  v6 source: `autopid_snapshot(cJSON**)` (RAM cache — safe from a PSRAM
  stack).
- **`status`** — device status (voltage, connectivity, uptime, version…).
  v6 source: the `/api/status` object (`dev_status_manager`, RAM).
- **`config`** — the autopid PID/filter table so HA can build sensors.
  v6 source: `/data/autopid/config.json`. **This is the trap** — see §5.

**Data mode** (`data_mode` / legacy `webhook_data_mode`): `full` sends
every section in full each cycle; `changed` (legacy default) diffs each
section against the previous snapshot and sends only changed keys, and
**omits empty sections**. The poster keeps three PSRAM "previous" strings
for the diff. This roughly halves airtime on a settled vehicle.

## 3. Component shape (v6)

A **feature** component `ha_webhooks` that owns the HA telemetry link.
It consumes services (`http_client_manager`, `http_server_manager`,
`cert_manager`) and reads from `autopid` + `dev_status_manager`. It does
**not** live inside autopid — autopid just exposes the data; the poster
is a consumer (ownership inversion, Standard §the-one-pattern).

Files (all ≤700 lines, Allman braces):

| File | Role |
|---|---|
| `ha_webhooks.c` | lifecycle (`init`/`start`/`stop`), settings descriptor + `on_apply`, the status struct + accessors |
| `ha_webhooks_post.c` | the poster task (PSRAM stack), payload builder, diff engine, failover POST via `http_client_manager` |
| `ha_webhooks_http.c` | `/api/webhook` GET/POST/DELETE (the HA registration API) |
| `ha_webhooks_cli.c` | `webhook` console command (status/test), self-registered from `on_apply` (§6b) |
| `include/ha_webhooks.h` | public API |
| `ha_webhooks_private.h` | shared internals |
| `TASK_ha_webhooks.md`, `README.md`, `HTTP_API.md` | docs |

## 4. Configuration — settings_manager, not a raw JSON file

Legacy stored `/ha_webhook.json` by hand. v6 uses a **settings
descriptor** (`"ha_webhooks"`), edited via `/api/settings/ha_webhooks`
(reboot-to-apply for structural fields; see §7 for the runtime-push
nuance). Fields:

| Key | Type | Default | Notes |
|---|---|---|---|
| `enabled` | bool | `false` | master switch for the poster |
| `url` | str ≤191 | `""` | primary webhook URL (http/https) |
| `url2` | str ≤191 | `""` | PRO failover URL (empty = none) |
| `interval_s` | int 5..3600 | `60` | post cadence |
| `data_mode` | enum `changed`/`full` | `changed` | diff vs full |
| `manual_override` | bool | `false` | pin the URL against HA auto-push |
| `cert_set` | str ≤31 | `""` | cert_manager set for a private-CA HA; empty = built-in bundle |
| `cli` | bool | `true` | register the `webhook` command |

Runtime **stats are NOT settings** — they live in a mutex-guarded PSRAM
status struct (`status`, `last_post`, `success_count`, `fail_count`,
`retries`, `last_error`, `last_error_time`), exposed by a getter and the
`/api/webhook` GET.

**TLS (meatpi note 2026-07-09):** `http_client_manager` resolves
`cert_set > ca_pem > built-in bundle`. So `cert_set=""` uses the internal
`esp_crt_bundle` (covers Nabu Casa + any public CA); a `cert_set` names a
`cert_manager` CA for a self-hosted HA with a private cert. Raw-IP HTTPS
hosts set `skip_common_name` automatically (CN can't match an IP).

## 5. The §2-corollary problem: the `config` section

The poster runs on a **PSRAM stack** (it's network I/O, PSRAM-first).
`autopid_snapshot` and dev_status are RAM — safe. But the `config`
section means reading `/data/autopid/config.json`, which is **flash
I/O**, and a PSRAM-stack task faults with the cache disabled (§2
corollary — the rule that has caused live crashes).

Options (decision in §9):
- **(A, recommended) autopid caches its config JSON in PSRAM** and exposes
  `autopid_config_json(cJSON**)` (or a `char*`), returning the already-
  loaded config with no flash touch. autopid reads the file at boot/PUT
  anyway; keep a PSRAM copy. Poster stays PSRAM, zero flash.
- (B) Poster task uses an **INTERNAL** stack. Costs scarce internal RAM
  (the cliff) and re-reads a large file each config-send — worse.
- (C) Send `config` only on change, built by a **one-shot internal-stack
  helper** (the `autopid_std` scan-task pattern) triggered rarely. More
  moving parts.

Plan: **(A)**. It's the cleanest and matches the legacy `autopid_get_config()`
(which returned a cached global, never a per-cycle file read).

## 6. The poster task (ha_webhooks_post.c)

```
loop (period 1 s):
  if !enabled or !url: continue
  wait for DEV_STATUS NETWORK_CONNECTED (block, not spin)
  if !autopid_enabled: continue
  if (now - last_attempt) < interval_s: continue
  last_attempt = now              # schedule on ATTEMPT, not success —
                                  # a down endpoint must not be hammered 1/s
  build payload (full or diff of config/status/autopid_data; omit empties)
  for url in [url, url2]:          # failover, first success wins
     resp = http_client_manager_request(POST, json, "application/json",
                                         cert_set, skip_common_name=is_raw_ip)
     if ok(resp): record success; break
  update stats (PSRAM status struct, mutex — NO flash write)
```

- PSRAM stack (§12); reads only RAM sources (§5-A).
- Blocks on the network bit rather than spinning.
- `http_client_manager` already has the concurrency limiter, PSRAM
  response cap, timeout, and TLS resolution — no bespoke HTTP code.
- Stats update is a cache write only (mutex-guarded PSRAM), never a
  settings/flash write (the task can't touch flash).

## 7. `/api/webhook` — the HA registration endpoint

Kept verbatim for HA compatibility (the HACS integration POSTs here):

- `GET /api/webhook` → current config + runtime stats (the `create_webhook_response` shape).
- `POST /api/webhook` `{url, urls?, enabled?, interval?, manual_override?}` →
  validate (http/https, urls[0]==url, ≤2 on PRO), then apply. If
  `manual_override` is already set and the body doesn't include it, the
  external push is **ignored** (user pinned it). 201 on first set, 200 on update.
- `DELETE /api/webhook` → clear + disable, 204.

**Registration nuance (settings interaction).** Settings are normally
reboot-to-apply, but the HA integration expects the webhook URL to take
effect **live** (it registers, then telemetry should start). So the
`/api/webhook` write path updates BOTH the persisted settings (so it
survives reboot) AND the live PSRAM cache the poster reads (so it applies
now) — the same split the legacy used (`ha_webhooks_set_config` = save +
cache; `ha_webhooks_update_cache` = cache only). This is a deliberate,
documented exception to reboot-to-apply, justified by the discovery UX.
The route is a **feature-component route** (`ha_webhooks_register_http`,
§9.1), covered by the network-trust gate like every admin surface;
outbound telemetry is unaffected by the gate (matches the autopid-http-
post-on-untrusted ruling).

## 8. Testing

- **Host (`host_test/`)**: the pure diff engine (build a changed-only
  object from prev/current cJSON) + URL validation + the raw-IP detector.
  No RTOS/flash. Unity.
- **Bench (`tools/testbench/ha_webhook_bench.py`, new)**: a mock HA
  webhook receiver on the Pi (tiny HTTP server) → configure the DUT via
  `POST /api/webhook` pointing at it → assert it receives well-formed
  `{config,status,autopid_data}` at the interval, in both `full` and
  `changed` modes → kill the primary, assert failover to `url2` → assert
  the GET stats reflect success/fail. HTTPS leg with a self-signed cert +
  `cert_set` if time permits, else built-in-bundle against a public echo.
  Wire as a `test.ps1 live` stage.
- **Regression**: the existing suites must stay green (autopid untouched
  except the new cached-config accessor).

## 8b. CONFIRMED against the real integration (rpi002:~/workspace/ha-wican)

Read the live HACS integration — `custom_components/wican/coordinator.py`
`handle_webhook_data()` + `DEVICE_ENDPOINTS.md`. Authoritative facts:

- **Telemetry payload = `{status, autopid_data, config, gps?}`.** The
  coordinator does `self._data.update(data)` (top-level merge) — sections
  stay nested; sensors read them. Order irrelevant.
- **`status` MUST carry `device_id`** — `_validate_device_identity` reads
  `status.get("device_id") or data.get("device_id")` to bind/verify the
  device (mismatch → ConfigEntryError). v6 `/api/status` does NOT include
  `device_id` today → **the poster injects `device_id`** (from
  `dev_status_manager_device_id()`) into the status section. (Absent =
  validation skipped for old firmware, but we want it.)
- **`gps`** is fully optional (lat/lon/accuracy/alt/speed/heading) — no v6
  GPS source yet → **omit** for now.
- Registration: `POST /api/webhook {url, urls?, enabled, interval}` →
  201 first / 200 update / 400 bad-url; `GET` → `{url, enabled, last_post,
  status, retries}`; `DELETE` → 204. Matches §7.
- Status-key ALIGNMENT (follow-up): the integration's sensors were built
  against the LEGACY flat status (`batt_voltage` as "12.5V" string,
  device_id, connectivity). v6 `/api/status` is restructured (nested
  `bits`). First cut sends v6 `/api/status` + injected `device_id`; the
  exact per-sensor key mapping is a coordination item with the integration
  (both are meatpi's) — track in §9.3.

## 9. Open questions for meatpi

1. **Component name**: `ha_webhooks` (matches legacy + HA side) vs
   `ha_integration`. Plan assumes `ha_webhooks`.
2. **`config` source (§5)**: confirm option A (autopid caches its config
   JSON in PSRAM + new accessor). Alternative is a heavier internal-stack
   path.
3. **`status`/`config` section contents**: legacy sent
   `config_server_get_status_json(true)` + `autopid_get_config()`. v6
   equivalents = `/api/status` + `/data/autopid/config.json`. Confirm HA
   expects those exact shapes, or if a curated subset is wanted.
4. **Failover `urls[]`**: keep the PRO-only 2-URL cap (`url`+`url2`), or
   make it universal now? Plan exposes `url`+`url2` for all.
5. **`data_mode` default**: `changed` (legacy, less airtime) vs `full`.
6. **Config-send policy**: every cycle, or once-then-telemetry-only (the
   legacy `initial settings push` optimization)? Plan sends per `data_mode`
   diff (config rarely changes → naturally near-empty in `changed` mode).
7. **GPS block**: legacy had a commented-out `gps` object. Out of scope now?
8. **ABRP + other destinations**: the legacy autopid also posted to ABRP
   and arbitrary destinations. This task is HA-webhook only; the general
   multi-destination story is separate (event_manager `http.post`).
