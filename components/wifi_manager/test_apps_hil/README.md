# wifi_manager — hardware-in-the-loop pytest suite

Automated RF scenarios against the bench Pi's access point
(`components/TESTBENCH.md`). The firmware app here exposes a serial command
console (`SET {json}` / `RESTART` / `STATUS` / `SCAN`) so pytest can persist
new WiFi settings and reboot the DUT between scenarios — every test exercises
real reboot-to-apply. Run:

```powershell
.\test.ps1 hil            # suite only (HIL app already flashed)
.\test.ps1 hil -Flash     # rebuild + flash the HIL app first
```

Prereqs: DUT on COM7, `ssh rpi001` reachable, pytest + pyserial installed
(`tools/testbench/requirements.txt`).

## What is covered (maps to S-numbers in ../test_apps/README.md)

| Test | Scenario | Pass criteria |
|---|---|---|
| `test_s1_sta_connect_and_dhcp` | join the bench AP | `STA got IP 10.42.0.x` on serial; lease visible and **pingable from the Pi**; `STATUS … sta=1` |
| `test_s2_reconnect_after_ap_loss` | AP down → up | disconnect logged; reconnect + got-IP ≤ 90 s after AP returns; pingable again |
| `test_s3_fallback_selection` | primary invisible, fallback real | `connecting to candidate 1: <fallback>` then got-IP |
| `test_s5_auth_failure_ban` | wrong PSK | after 3 auth failures: `banned; deferring connect`; `STATUS … sta=0` |
| `test_s6_recovery_with_correct_password` | fix the PSK | connects normally after reboot |
| `test_s7_ap_auto_disable` | `ap_auto_disable=true` | `STA up; auto-disabling AP` then `ap=0`; on AP loss `STA down; re-enabling AP` + `AP started` |
| `test_s9_scan_sees_bench_ap` | scan while associated | bench SSID present in `SCANJSON` (retries — RF scans can come back thin) |

Not automated yet: S4 (simultaneous primary+fallback priority — needs
ethernet on rpi001 to free a second radio), S8 (AP channel follow), S10
(HTTP over RF).

## Expected result

```
7 passed in ~115s
```

All bench AP connections are `wican-*` and are cleaned up on entry and exit,
even after an aborted run. The suite leaves the DUT's persisted WiFi settings
pointing at the bench AP — rerun `.\test.ps1 target wifi_manager` or erase
flash for pristine settings.

## Provenance

First live run (2026-07-02) caught a real component bug: wrong-PSK
disconnects report `WIFI_REASON_HANDSHAKE_TIMEOUT` (204), which the
auth-related reason list was missing — the ban never armed on hardware.
Fixed in `wifi_manager.c` and pinned by `test_s5`.
