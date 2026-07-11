# settings_manager — host unit tests

Pure-logic suite (IDF `linux` target, no hardware). Run via `.\test.ps1 host`
(all suites, on rpi001) or manually per `components/TESTBENCH.md` §4.

## What is covered (36 tests)

| Area | Cases | What they prove |
|---|---|---|
| Schema validation (`test_schema.c`, 14) | valid object accepted; missing `required` / wrong type / out-of-range int / non-int / enum non-member / overlong string rejected (with error text); number+enum+bool accepted; `format:"file"` present vs missing; unknown keys allowed; defaults collected | the JSON-Schema subset behaves exactly as documented in the component README |
| Registry — rev 2 contract (`test_registry.c`, 13) | `set()` never calls `on_apply`; `changed` flag + write dedup; `defaults_json` whole-object override; schema defaults primary; migrate happy path / migrate failure / invalid migrate result / missing hook → defaults; `on_apply` failure → retry with defaults (degraded); defaults also fail → unconfigured, boot continues; bad `defaults_json` rejected at register; register-after-start rejected; PUT is full-replace not merge | Coding Standard §4.2/§4.3 reboot-to-apply and boot-fallback semantics |
| Codec (`test_codec.c`, 4) | CRC-32 known vector; envelope encode/decode roundtrip; tamper detected; malformed rejected | §4.4 integrity guarantees |
| Field-table schema generator (`test_fields.c`, 5) | generated keywords (enum/minLength/maxLength/max); validator roundtrip accept+reject; defaults collected; duplicate/empty keys rejected; `required` emitted + enforced | rev 2.2 table authoring produces the same schema semantics |

## Expected result

```
36 Tests 0 Failures 0 Ignored
OK
```

The ELF does not exit afterwards (FreeRTOS POSIX port) — the runner kills it
and judges by the summary line. Any `:FAIL` line or a lower test count is a
failure. Last verified green: 2026-07-02 on rpi001.
