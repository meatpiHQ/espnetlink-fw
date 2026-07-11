# restart_tracker — on-target test app

Self-contained, **two-phase across a real reboot**. Builds against the main
partition table (standard rev 2.1). Run:

```powershell
.\test.ps1 target restart_tracker
```

## What is covered

PSRAM `.noinit` state surviving a genuine `esp_restart()`; the planned-restart
intent (reason/source/flags) recorded by the *next* boot; boot counter
continuity; `esp_restart()` classified as `software` and **not** counted as
unexpected.

## Expected result — serial markers

Phase 1 (boot right after flashing — reset reason is not "planned"):

```
BOOT count=1 seq=1 reason=<poweron|external|software> planned=0
PHASE1 marking planned restart and rebooting
```

then the device reboots itself, and phase 2 prints:

```
BOOT count=2 seq=2 reason=software planned=1
PHASE2 planned=1 reason=user_request source=console flags=0xC0FFEE
SURVIVED boots=2 unexpected=0 history_kept=1
CLASSIFY reset=software unexpected_count=0
TEST DONE
```

Note `boots` keeps growing on re-runs without a power cycle (state survives
reflashing only if PSRAM content is untouched — a flash + hard reset usually
preserves it; a power cycle resets to `boots=2`). The serial-capture verifier
only requires `TEST DONE` + no crash; the strict sequence lives in
`pytest_restart_tracker.py`. Last verified: 2026-07-03 on WiCAN Pro.
