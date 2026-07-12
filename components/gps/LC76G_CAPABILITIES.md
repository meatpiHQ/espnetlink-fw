# LC76G capability sweep — what the GNSS module can do for ESPNetLink

Summary of the product-relevant capabilities from the Quectel
*LC26G/LC26G-T/LC76G/LC86G Series GNSS Protocol Specification* (v1.5) and
*AGNSS Application Note* (v1.1). Section numbers refer to those documents
(kept locally; gitignored — Quectel confidentiality notice). Written
2026-07-12 after the EASY/EPOC investigation.

## Already in use

| Command | Use |
|---|---|
| `PAIR590` / `PAIR600` | Reference time / position aiding at boot (agnss.c) |
| `PAIR508` / `PAIR496` / `PAIR498` + `PAIR513` | EPOC status / enable / constellation config (`epoc_constellation` setting) + NVRAM persist — write-on-change only |
| `PAIR511` (+ `PAIR050`, `PAIR003`, `PAIR002`) | Manual `agnss save` nav-data save (CLI). Sweep found and fixed two bugs here: `PAIR382` (sleep-mode lock) had been sent in place of `PAIR050` (fix rate, §2.4.9), and the ACK matcher missed zero-padded ids (`$PAIR001,003,…`) for 1–2 digit commands |
| `$PAIR010` (unsolicited) | Aiding requests — currently only observed; drives the future Host EPO feature |

## High product relevance

### Battery / low-power tracking

- **LOCUS autonomous logger** (`PAIR900–909`, §2.4.72+): the module logs
  fixes **into its own 128 KB flash** with normal / time / speed /
  distance-triggered modes, dump via `PAIR908`, count via `PAIR909`.
  ESPNetLink could deep-sleep the ESP32 (WoM armed) while the GNSS quietly
  records a track, then bulk-upload over LTE on wake. This is the single
  most interesting unused capability for a battery tracker.
- **Periodic power-saving mode** (`PAIR690/691`, §2.4.55): autonomous
  run/sleep duty cycling (run 3 s+, sleep up to days; smart vs strict) —
  fits "position every N minutes" tracking without host involvement.
- **Backup/RTC mode** (`PAIR650`, §2.4.52): everything off except the
  clock, timed (10 s–2 years) or indefinite; wake keeps hot-start data.
  Complements (or replaces) cutting the module's rail via `gps -P`.
- **GLP / ALP / FLP** (`PAIR680`, `PAIR732`, `PAIR730`): one-command
  duty-cycle / adaptive / fitness low-power trade-offs while still fixing.
  Note FLP force-disables SBAS, GLP, ALP, periodic mode.

### Fix quality / robustness

- **Static navigation threshold** (`PAIR070`, §2.4.17, 0–20 dm/s):
  below-threshold speed pins the output position and zeroes speed —
  kills parked-position jitter *at the source*. Synergy: our NVS-wear
  guard, HA position reports, and LOCUS logs all get cleaner. Cheap win.
- **AIC — active interference cancellation** (`PAIR074/075`, §2.4.21):
  worth enabling/verifying on this board — the BG95 LTE modem transmits
  centimetres from the GNSS RF path.
- **Jamming detection** (`PAIR391` → periodic `$PAIRSPF,<status>` output,
  §2.4.36): jamming/interference status telemetry — could surface as a
  dev_status bit + HA webhook field ("GPS jamming suspected").
- **SBAS** (`PAIR410/411`) and **DGPS/RTCM input** (`PAIR400+`,
  `PAIR432+`): accuracy upgrades if the product ever needs them.

### Configuration

- **Constellation search mask** (`PAIR066/067`, §2.4.15): per-system
  enable GPS/GLONASS/Galileo/BDS/QZSS. NOTE: changing it **reboots the
  module**; it also bounds what EPOC can predict. Currently all-on
  (GSV shows GP/GL/GA/GB) — a `gps` setting could mirror
  `epoc_constellation` if we ever want to trim search for power.
- **Fix rate** (`PAIR050/051`): 100–1000 ms (up to 10 Hz).
- **Per-sentence NMEA output rates** (`PAIR062/063`): e.g. GSV every
  5 fixes instead of every fix — the GSV burst is ~8 of the ~12 lines the
  ESP parses every second, so this directly cuts UART traffic and parser
  wakeups. Cheap win if CPU/power budgets ever pinch.
- **Navigation mode** (`PAIR080/081`): normal / fitness / balloon /
  drone-style profiles tuning the position filter.
- **UART baud** (`PAIR864/865`); **NVRAM ops** (`PAIR513` save settings,
  `PAIR514` factory defaults — both are flash writes: only on change).

### Niche (noted, no current need)

- **PPS timing** (`PAIR751–765`): PPS config, timing-product mode, TRAIM
  integrity — relevant only if ESPNetLink ever grows a precision-time story.
- **QZSS RLM / BDS B1C toggles** (`PAIR154+`, `PAIR158+`), min-SNR filter
  (`PAIR058`), 2D-fix allowance (`PAIR162`), immediate-speed mode
  (`PAIR160`), debug log output (`PAIR086`).

## Suggested order if/when picked up

1. `PAIR070` static threshold (one command at AGNSS init, immediate quality
   win) + verify `PAIR074` AIC is on.
2. `PAIR062` GSV rate reduction (UART/CPU relief, trivially reversible).
3. LOCUS + deep-sleep track logging (a real feature: needs a settings
   surface, dump-and-upload plumbing, and bench time).
4. `PAIR391` jamming telemetry into dev_status/HA.
5. Power modes (`PAIR690`/`PAIR650`) once a battery-profile product story
   exists — they interact with sleep_manager design.
