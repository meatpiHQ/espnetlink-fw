# filesystem — on-target test app

Self-contained (no external gear). Mounts the real `storage` LittleFS
partition from the **main partition table** (standard rev 2.1 §7). Run:

```powershell
.\test.ps1 target filesystem
```

## What is covered

Mount (format-on-first-use on a virgin partition), whole-file write/read
roundtrip, `exists`, atomic overwrite (new content visible, no `.tmp`
leftover), parent-dir auto-create, `size` + too-small-buffer reporting the
required size, `list` (file + nested dir), delete, five malformed paths
rejected, `/sd` reserved-but-unavailable (`ESP_ERR_INVALID_STATE`), validated
streaming `open`, capacity info from the real partition, persistence across
unmount/remount.

## Expected result — serial markers, in this order

```
MOUNT ok=1
RW body=hello-fs len=8
EXISTS file=1 missing=0
ATOMIC body=v2-content leftover=0
NESTED body=deep
SIZE n=10 small=ESP_ERR_INVALID_SIZE need=10
LIST saw_file=1 saw_dir=1
DELETE ok=1 exists=0
BADPATH rejected=5
SDPATH state=1
OPEN body=deep
INFO total_kib=4096 used_ok=1
REMOUNT body=deep
TEST DONE
```

`INFO total_kib=4096` doubles as proof the main partition table is in use
(ESPNetLink's `storage` partition is 4 MB; WiCAN Pro's was 5888 KiB).
First mount of a virgin partition logs `mount failed, formatting...` —
expected. Suite inherited from WiCAN v6 (last green there 2026-07-02);
not yet run on ESPNetLink hardware.
