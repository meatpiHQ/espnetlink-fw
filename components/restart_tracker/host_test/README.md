# restart_tracker — host unit tests

Pure core suite (`restart_tracker_core.c` — no esp_cache/esp_timer), IDF
`linux` target. Run via `.\test.ps1 host`.

## What is covered (7 tests)

| Case | What it proves |
|---|---|
| random memory is invalid and resets | power-on garbage never masquerades as history (magic/version/CRC) |
| boot recording and counters | one record per boot; sequence + boot_count advance; valid state adopted |
| planned restart consumed once | intent recorded into exactly the next boot, then cleared |
| unexpected classification | panic/task-wdt count; sw/poweron/deepsleep don't; a planned boot never counts |
| history ring wraps | 8-entry ring arithmetic; newest sequence at latest index |
| CRC detects tamper | one flipped bit invalidates the state |
| invalid time stored as zero | pre-2024 clock → timestamp 0 + time_valid 0 |

## Expected result

```
7 Tests 0 Failures 0 Ignored
OK
```
