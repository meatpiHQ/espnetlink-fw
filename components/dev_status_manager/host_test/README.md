# dev_status_manager — host unit tests

Pure helpers only (`dev_status_manager_fmt.c`). Run via `.\test.ps1 host`.

## What is covered (5 tests)

| Case | What it proves |
|---|---|
| bit names map known bits | every `DEV_STATUS_BIT_*` resolves to its stable string name |
| bit names unknown | unmapped, zero, and multi-bit values return "unknown" |
| uptime formats H:M:S | `01:02:03`, zero-uptime `00:00:00` |
| uptime formats days | `2d 03:04:05` |
| uptime truncation/errors | NULL/zero-length safe; small buffers truncate NUL-terminated |

## Expected result

```
5 Tests 0 Failures 0 Ignored
OK
```
