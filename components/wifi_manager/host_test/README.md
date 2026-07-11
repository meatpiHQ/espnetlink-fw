# wifi_manager — host unit tests

Pure selection/ban-logic suite (`wifi_manager_select.c` only — no esp_wifi),
IDF `linux` target. Run via `.\test.ps1 host` or manually per
`components/TESTBENCH.md` §4.

## What is covered (11 tests)

| Case | What it proves |
|---|---|
| scan prefers primary | primary wins whenever visible, regardless of scan order |
| falls back in priority order | primary absent → fallback1 beats fallback2 |
| skips banned candidate | a banned primary yields to a visible fallback |
| all banned picks best anyway | never refuse to connect just because everything is banned |
| nothing present returns none | no configured network visible → defer, don't guess |
| ban after threshold and expiry | 3 auth failures arm the ban; it expires after `WM_BAN_DURATION_MS` |
| success clears failures | one good connect resets an SSID's strikes/ban |
| already banned does not extend | failures during a ban don't push the window out |
| sequential rotates and wraps | no-scan path round-robins 0→1→2→0 |
| sequential skips banned | rotation hops over banned entries |
| sequential all banned still returns | rotation degrades to "try one anyway" |

## Expected result

```
11 Tests 0 Failures 0 Ignored
OK
```

Last verified green: 2026-07-02 on rpi001. The radio/event integration of the
same logic (real auth failures, real reconnects) is covered by
`../test_apps_hil` on the bench.
