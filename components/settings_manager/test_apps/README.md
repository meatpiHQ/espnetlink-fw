# settings_manager — on-target test app

Self-contained (no external gear). Exercises the real `settings` LittleFS
partition from the **main partition table** (standard rev 2.1 §7). Run:

```powershell
.\test.ps1 target settings_manager      # build + flash + capture + verify
```

Erase flash first (`idf.py -p COM7 erase-flash`) when asserting first-boot
behavior — settings persist across reflashes by design.

## What is covered

Registration hardening (bad schema / bad name / bad defaults rejected at the
call site), first-boot defaults applied exactly once, `set` persists without
applying (reboot-to-apply), write dedup (`changed=0`), range rejection with
error text, persistence across reboot, v1→v2 migration (key rename), CRC
corruption → defaults fallback, broken component degrades without affecting
its sibling.

## Expected result — serial markers, in this order

```
REG badschema REJECTED
REG badname REJECTED
REG baddefaults REJECTED
APPLY count=1 channel=6
CURRENT channel=6
SET ch=11 OK changed=1
APPLY-COUNT-AFTER-SET 1
CURRENT channel=11
SET ch=11 OK changed=0
SET ch=99 REJECT: channel: above maximum 13
CURRENT channel=11
APPLY count=2 channel=11
CURRENT channel=11
MIGRATE from=v1
APPLY count=3 channel=11
CURRENT wifi_channel=11
DEGRADED demo=0
CORRUPT done
APPLY count=4 channel=6
CURRENT wifi_channel=6
APPLY count=5 channel=6
DEGRADED broken=1
DEGRADED demo=0
TEST DONE
```

A first mount of a virgin partition logs a littlefs `Corrupted dir pair` error
then formats — expected, not a failure. Crash signatures (`abort()`,
`Guru Meditation`) are failures. `pytest_settings_manager.py` encodes the same
sequence for pytest-embedded. Last verified green: 2026-07-02 on WiCAN Pro.
