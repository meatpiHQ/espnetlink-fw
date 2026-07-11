# wifi_manager on-target tests — bench setup (Coding Standard §7)

Two layers of on-target testing:

1. **Self-contained suite** (`pytest_wifi_manager.py` + `test_wifi_main.c`) —
   no external gear. Covers boot-apply, AP bring-up with the MAC-derived SSID,
   scan JSON, settings rejection, reboot-to-apply semantics. Green on hardware
   2026-07-02.
2. **Hardware-in-the-loop STA scenarios** — **automated as pytest** in
   `../test_apps_hil/` (serial-command HIL firmware +
   `pytest_wifi_manager_hil.py`), driven by the shared bench framework
   (`tools/testbench/`, see `components/TESTBENCH.md` §3):

   ```powershell
   # flash ../test_apps_hil once, then:
   pytest components/wifi_manager/test_apps_hil -v --dut-port COM7
   ```

   pytest controls the Pi's APs over SSH (`bench` fixture) and the DUT over
   serial (`dut` fixture). The table below is the scenario reference; the
   S-numbers map to test functions in the pytest file.

## Self-contained suite: coverage + expected result

Run: `.\test.ps1 target wifi_manager`. Covers: composition boot (settings
apply → start), schema defaults, MAC-derived AP SSID, AP bring-up, scan JSON,
schema rejection with error text, cross-field `on_validate` rejection, valid
set persisting as **pending** with the running radio untouched
(reboot-to-apply), clean stop. Expected serial markers, in order (clean
flash — persisted settings change the first two values on re-runs, by design):

```
INIT ok=1
SETTINGS mode=apsta ap_channel=6
START ok=1
AP started=1
STATUS enabled=1 sta=0 clients=0
DEVSTATUS ap=1 sta=0
SCAN ok=1 has_networks=1
SET-BAD rejected=1 err_set=1
SET-CROSS rejected=1
SET-OK ok=1 changed=1
PENDING ap_channel=11 ap_still_up=1
STOP ok=1 enabled=0
DEVSTATUS-STOP ap=0
TEST DONE
```

Last verified green: 2026-07-03 on WiCAN Pro (dev_status wiring: AP_ENABLED /
STA_CONNECTED / STA_AP_OVERLAP bits published; earlier verified 2026-07-02
against the field-table-generated schema).

## Tools + versions

| Item | Value (verified 2026-07-02) |
|---|---|
| DUT | WiCAN Pro, ESP32-S3 rev v0.2, COM7, test app on ESP-IDF v6.0.2 |
| AP machine | rpi001 — Pi 5 rev 1.1, Raspberry Pi OS bookworm, kernel 6.12.34 |
| AP radio | USB Realtek `0bda:c812` (RTL8822CU), in-kernel `rtw_8822cu`, AP mode verified |
| AP software | NetworkManager hotspot (WPA2-PSK, NM shared-mode DHCP, 10.42.0.1/24); hostapd installed for deauth-style scenarios |

## Wiring

None — RF only. Keep the DUT and the Pi within a few meters. The Pi's SSH
uplink is its **built-in wlan0**; only `wlan1` is free for AP duty unless
ethernet is plugged into the Pi (then wlan0 can be a second, simultaneous AP —
required for the "both networks visible, primary wins" case).

## Host setup

- SSH host alias `rpi001` (key auth, passwordless sudo) — see TESTBENCH §1.
- DUT serial on COM7 at 115200 (test app default).
- Configure the DUT's STA settings via `settings_manager_set` in a test build,
  or (once `settings_http` exists) `PUT /settings/wifi_manager`; reboot to
  apply (§4.2).

## Procedure + pass/fail per scenario

Bring the bench AP up/down with the nmcli one-liners in TESTBENCH §3
(`ssid WICAN_TEST_AP password test12345` assumed below). DUT settings:
`sta_ssid=WICAN_TEST_AP`, `sta_password=test12345`, `mode=apsta` unless stated.

| # | Scenario | Procedure | Pass criteria (DUT serial) |
|---|---|---|---|
| S1 | STA connect + DHCP | AP up before DUT boot | `STA got IP 10.42.0.x` within ~15 s of boot; `wifi_manager_is_sta_connected()` true |
| S2 | Reconnect after AP loss | S1, then `connection down`, wait 30 s, `connection up` | disconnect logged; reconnect task retries (DEBUG); got-IP again ≤ ~30 s after AP returns; no reboot, no crash |
| S3 | Fallback (sequential) | DUT: `sta_ssid=PRIMARY_X` (never up), `fallback1_ssid=WICAN_TEST_AP`. Boot with AP up | DUT connects to the fallback: `connecting to candidate 1: WICAN_TEST_AP`, then got-IP |
| S4 | Priority — AUTOMATED 2026-07-08 (`test_s4_priority_both_visible`; wlan0 freed when the Pi uplink moved to wlan2) | Both APs up: primary on wlan0, fallback on wlan1. Boot | `connecting to candidate 0` (primary) even though both are visible |
| S5 | Auth-fail ban | DUT `sta_password` deliberately wrong; AP up; watch ≥ 3 attempts | after 3 auth failures the SSID is skipped (ban, DEBUG `banned; deferring`); with a fallback configured the DUT moves to it |
| S6 | Ban clears on success | S5, then fix the password (persist + reboot) | connects; `wm_select_on_success` path — no residual skip |
| S7 | AP-auto-disable | `ap_auto_disable=true`, mode `apsta`, bench AP up | DUT AP visible while disconnected; after got-IP, `STA up; auto-disabling AP` and the DUT SSID disappears; kill bench AP → `STA down; re-enabling AP`, DUT SSID returns |
| S8 | AP channel follow | mode `apsta`, `ap_auto_disable=false`, bench AP pinned `channel 11`; DUT `ap_channel=6` | after got-IP: `moving AP to STA channel 11`; DUT beacon now on 11 (verify: `ssh rpi001 "sudo iw dev wlan1 scan | grep -B5 WiCAN_"`) |
| S9 | Subnet overlap warning | Bench AP with NM shared subnet forced to 192.168.80.0/24 (matches DUT AP 192.168.80.1/24): `nmcli connection modify wican-ap1 ipv4.addresses 192.168.80.5/24` | `STA and AP subnets overlap` warning; `WIFI_MANAGER_BIT_STA_AP_OVERLAP` set |
| S10 | http_server_manager over RF | DUT joined bench AP; from Pi: `curl http://<dut-ip>/index.html` | 200 with the embedded body; ETag/304 on second request with `If-None-Match` |
| S11 | Wrong password → fallback (`test_s10_wrong_password_moves_to_fallback`) | primary = bench AP with WRONG psk, fallback = second AP correct; both up | 3 auth fails → ban → `connecting to candidate 1` on the fallback, got-IP |
| S12 | Banned-only trickle (`test_s11_banned_only_visible_trickles`) | single network, wrong psk, no alternative | ban → `banned; deferring connect`; NO attempt inside the ~60 s throttle window; then a trickle attempt (same-SSID-elsewhere / drive-home case, meatpi 2026-07-08) |
| S13 | Roam-to-preferred (`test_s12_roam_to_preferred`) | connected to fallback (primary off air), `sta_roam_interval_s=30`; bring primary up | `preferred network '<primary>' visible; leaving ...` then got-IP on the primary |
| S14 | Hidden SSID (`test_s13_hidden_ssid`) | fallback AP hidden (`802-11-wireless.hidden yes`), primary a decoy | `blind attempt` (sequential fallback) then got-IP on the hidden AP |
| S15 | Same SSID, two BSSIDs (`test_s14_same_ssid_two_bssids`) | SAME ssid+psk on wlan0 (ch 11) + wlan1 (ch 1) | associates with ONE BSSID (strongest — WIFI_CONNECT_AP_BY_SIGNAL), stable ≥20 s, no flapping (legacy mesh behavior, meatpi 2026-07-08) |
| S16 | Network-trust lockdown attack-surface sweep (`tools/testbench/wifi_gate_live_test.py`, live-suite stage `wifi_gate`, NOT this pytest — runs on the PC over USB+Pi) | mark the current STA network untrusted; enumerate EVERY /api route (from source) + UI catch-all + /ws channels (from `/api/ws`) | over STA: every route×method returns 403/405 (zero handler responses), every route hits the 403 gate, all WS handshakes refused; over USB: admin still 200, MQTT still connected; restore → STA 200 (meatpi 2026-07-08) |

Notes:
- S2/S5 timing: reconnect cadence is 5 s; ban = 3 auth failures → 10 min skip
  (`WM_AUTH_FAIL_THRESHOLD` / `WM_BAN_DURATION_MS` in `wifi_manager_private.h`).
- The selection/ban *logic* itself is host-tested (`host_test/`, 11 cases);
  these scenarios validate the radio/event integration, not the algorithm.
- Reflash main firmware when done (TESTBENCH §6).

## Status (2026-07-02)

- Self-contained suite: automated, green.
- **S1, S2, S3, S5, S6, S7, S9: automated in `../test_apps_hil` and GREEN on
  the real bench — `7 passed in 114 s`** (STA connect/DHCP/ping, reconnect
  after AP loss, fallback selection, auth-fail ban, recovery, AP-auto-disable
  both directions, scan). The ban scenario caught and fixed a real component
  bug: wrong-PSK failures report `WIFI_REASON_HANDSHAKE_TIMEOUT` (204), which
  the auth-related reason list was missing.
- S4 (simultaneous primary+fallback priority): blocked on an ethernet cable
  for rpi001 (frees wlan0 as second AP).
- S8 (AP channel follow) / S10 (HTTP over RF): manual per the table above;
  automate when convenient.
