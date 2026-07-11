# web_ui

The built-in web app. **API-first (§1b): it is just another client** —
every screen works exclusively through the documented `/api` + `/ws`
surfaces; nothing here has a privileged path.

## Preview in a browser (no device needed)

```powershell
cd components\web_ui\web
python -m http.server 8000
# then open:
#   http://localhost:8000/?mock=1              — canned data (js/mock.js)
#   http://localhost:8000/?api=http://<dut-ip> — against a real device
```

`js/mock.js` is DEV-ONLY — never list it in EMBED_FILES.

## The rules that keep this maintainable

The legacy UI died of a 5,584-line `main.js`. Hard rules here:

1. **One file per page** (`js/pages/<tab>.js`), exporting
   `render(viewEl) -> cleanupFn`. Pages are lazy-loaded by the router.
2. **Core stays small** (`js/app.js` ≈ 7 KB): API wrapper, connection
   watchdog, hash router, `h()` DOM builder, toasts. Page logic never
   goes in core; styles never go in pages (everything in `app.css`,
   mobile-first).
3. **Schema-driven wherever the firmware offers it.** The Settings page
   renders every component's form from `GET /api/settings/<name>/schema`;
   the Automation editor builds its dropdowns from
   `/api/events/sources|actions|values`. New firmware features appear in
   the UI without UI changes — forms are data, not code.
4. **Size budget: ≤ 100 KB raw / ≤ 30 KB gzipped total.** Currently
   ~69 KB raw / ~18.5 KB gzipped. No frameworks, no icon fonts
   (`js/icons.js` = ~15 hand-picked inline SVGs).

## App-like behavior (the #1 legacy complaint)

`js/app.js` owns a **connection watchdog** (3 s `/api/status` poll):

- online / offline states drive the topbar pill + a banner — the user
  always sees whether the device is reachable;
- `expectReboot(reason)` switches to an *expected*-outage banner
  ("Applying settings — waiting for the device…") used by settings
  apply, OTA, restore, factory reset — and when the device answers
  again the current page re-renders itself. **Nobody ever refreshes
  the page manually.**

## Tabs (matched to the v6 firmware)

| Tab | Backed by |
|---|---|
| Status | `/api/status`, `/api/battery` (poll) |
| Settings | `/api/settings` + per-component values/schema, `submit`, per-component reset |
| AutoPID | `/api/autopid` (live), `/config` (editor, applies live), `/std_scan[/result]`, `/test` |
| Automation | `/api/events/*` discovery + log, `event_manager` settings |
| Terminal | `/ws/cli` |
| System | `/api/ota/upload`, `/api/settings/backup` (down/up), `factory_reset`, `/api/restart`, `/api/logs` |

Mobile: the sidebar collapses to a drawer under 860 px; tables become
stacked cards under 640 px (`.rtable` — no separate mobile code paths,
this was the legacy CAN-monitor pain).

## Embedding (when we wire the firmware)

`web_ui.c` registers one `http_asset_t` table with
http_server_manager (embedded blobs via `EMBED_FILES`, served by the
catch-all; API routes always win). Pre-gzip at build time and set
`content_encoding = "gzip"` — the manager already supports it. Files to
embed: `index.html`, `app.css`, `js/app.js`, `js/icons.js`,
`js/pages/*.js` (NOT `js/mock.js`).
