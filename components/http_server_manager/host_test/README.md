# http_server_manager — host unit tests

Pure match-logic suite (`http_server_manager_match.c` only — no httpd/VFS),
IDF `linux` target. Run via `.\test.ps1 host` or manually per
`components/TESTBENCH.md` §4.

## What is covered (11 tests)

| Case | What it proves |
|---|---|
| normalize strips query and fragment | `/a?x=1#y` resolves as `/a` |
| normalize root maps to index | `/` serves `/index.html` |
| normalize rejects traversal | `..` in a URI → 400, never reaches the FS |
| normalize rejects overlong | URI beyond the path cap is refused |
| exact match: first table wins | registration order is the tie-breaker |
| exact match: fs-path entry | non-embedded entries resolve to their `fs_path` |
| prefix match joins remainder | `/web/*` + `/web/icons/x.svg` → `<fs_path>/icons/x.svg` |
| prefix requires remainder | bare `/web` does not match a `/web/*` entry |
| second table reachable | multiple registered asset tables are all searched |
| no match is NULL | unknown path → catch-all 404 path |
| MIME inference | extension → content-type when the entry has none |

## Expected result

```
11 Tests 0 Failures 0 Ignored
OK
```

Last verified green: 2026-07-02 on rpi001. Serving, ETag/304, fetch-on-miss
and traversal-over-HTTP are covered on-target by `../test_apps`.
