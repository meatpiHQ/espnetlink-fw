# log_manager — host unit tests

Pure crash-ring core only (`log_manager_ring.c`). Run via `.\test.ps1 host`.

## What is covered (8 tests)

| Case | What it proves |
|---|---|
| garbage header is invalid | power-on PSRAM noise never reads as logs |
| reset then valid | fresh header validates; empty read |
| append + read roundtrip | bytes come back verbatim, chronological |
| wrap keeps newest | oldest bytes fall off, newest survive intact |
| oversized append keeps tail | a write larger than the ring keeps its newest bytes |
| small-buffer read gets newest | readers get the newest window, still chronological |
| header tamper detected | CRC catches a flipped index |
| size mismatch invalid | a Kconfig ring-size change invalidates the old ring safely |

## Expected result

```
8 Tests 0 Failures 0 Ignored
OK
```
