# filesystem — host unit tests

Pure path-logic suite (`filesystem_path.c` only — no VFS/LittleFS), IDF
`linux` target. Run via `.\test.ps1 host` or manually per
`components/TESTBENCH.md` §4.

## What is covered (10 tests)

| Case | What it proves |
|---|---|
| accepts valid paths | `/data`, `/sd`, nested files, spaces allowed |
| maps backends | `/data/*` → internal, `/sd/*` → SD |
| rejects unknown prefix | NULL, empty, relative, `/nope`, `/datax`, `/settings` (reserved for settings_manager) |
| rejects traversal and dot | `..` and `.` segments can never escape the namespace |
| rejects empty segment / trailing slash | `//`, `/data/`, `/data/a/` |
| rejects bad chars and overlong | backslash, control chars, ≥128 chars |
| temp name appends suffix | atomic-write sibling is `<path>.tmp` |
| temp name rejects overflow | a path whose `.tmp` sibling wouldn't fit is refused |
| parent derivation | `/data/a/b.txt` → `/data/a`; `/data/b.txt` → `/data` |
| parent of root rejected | `/data` itself has no parent |

## Expected result

```
10 Tests 0 Failures 0 Ignored
OK
```

Last verified green: 2026-07-02 on rpi001. The on-target behaviours (real
mount, atomic replace, persistence) are covered by `../test_apps` instead.
