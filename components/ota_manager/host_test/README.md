# ota_manager — host unit suite

Compiles ONLY the pure session state machine (`ota_session.c` — flash
operations injected via a recorder backend, no esp_ota) on the IDF
**linux** target. Runs on the bench Pi via `.\test.ps1 host`.

## What is covered (9 tests)

Happy path with an announced size; write/end before begin rejected;
double-begin rejected while receiving (backend untouched); overrun of the
announced size → FAILED + flash session released; short image at end;
empty image at end; backend write failure latches FAILED + aborts;
validation failure at end does NOT double-abort (esp_ota_end releases the
handle itself); retry from FAILED works and abort resets to IDLE with
exact backend-call accounting.

## Expected result

```
9 Tests 0 Failures 0 Ignored
OK
```

Last verified green: 2026-07-04 on rpi001.
