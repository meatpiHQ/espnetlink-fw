# AW2023 — Driver Implementation Reference

3-channel constant-current LED driver, I2C slave. Compiled from the AWINIC AW2023 datasheet (May 2019, V1.4).

## Quick facts

| Param | Value |
|---|---|
| I2C 7-bit address | **0x45** (fixed, no address pins) |
| I2C speed | Fast mode, 400kHz max |
| I2C logic level | 1.8V–3.3V |
| VCC | 2.5V–5.5V |
| Pull-ups | 1k–10kΩ, typ 4.7kΩ |
| Package | DFN-10L 2x2mm |
| Chip ID | 0x09 (readback of reg 0x00) |
| PWM resolution | 8-bit, 256-level, exponential or linear |
| Current resolution | 4-bit per channel, 16 levels |
| PWM carrier freq | 250Hz (default) or 125Hz |

## I2C protocol

**Write cycle** (Fig. 5): START → addr(7b)+W(0) → ACK → reg_addr(8b) → ACK → data(8b) → ACK → [more data bytes auto-increment reg_addr, each followed by ACK] → STOP.

**Read cycle** (Fig. 6) — two valid forms:
- *Repeated start*: START → addr+W → ACK → reg_addr → ACK → **Sr** (repeated start) → addr+R(1) → ACK → data byte(s) (master ACKs to continue reading auto-incremented registers, NACKs the last byte) → STOP.
- *Separated transaction*: START → addr+W → ACK → reg_addr → ACK → STOP → START → addr+R → ACK → data byte(s) → STOP.

Register address auto-increments after each byte in both read and write — you can burst-write/read contiguous registers (useful for e.g. LEDxT0/T1/T2 or PWM0/1/2).

SDA must only change while SCL is low (standard I2C data validity rule).

**Timing (Fast mode, all values from datasheet table)**

| Param | Min | Max | Unit |
|---|---|---|---|
| Fscl | – | 400 | kHz |
| Tdeg (SCL / SDA) | – | 200 / 250 | ns |
| Thd:sta | 0.6 | – | µs |
| Tlow | 1.3 | – | µs |
| Thigh | 0.6 | – | µs |
| Tsu:sta | 0.6 | – | µs |
| Thd:dat | 0 | – | µs |
| Tsu:dat | 0.1 | – | µs |
| Tr / Tf | – | 0.3 / 0.3 | µs |
| Tsu:sto | 0.6 | – | µs |
| Tbuf | 1.3 | – | µs |

## Operating mode state machine

```
        GCR1.CHIPEN=1
Standby ───────────────► Active
        ◄───────────────
        GCR1.CHIPEN=0, OR automatic UVLO/OTP trip
```

- **Standby**: only registers `RSTR` (0x00) and `GCR1` (0x01) are writable. Internal OSC is off. Current draw <5µA.
- **Active**: OSC running, LED/pattern/PWM registers usable. Quiescent ~100µA typ with all LEDs off.
- Software reset: write **0x55** to `RSTR` (0x00) — resets all functional circuits and config registers.
- Reading `RSTR` returns fixed **0x09** — use as a whoami / bus-presence check.
- On UVLO or OTP trip, `GCR1.CHIPEN` is auto-cleared by hardware and the device drops to standby. Driver should watch `ISR` (UVLOIS/OTPIS) and re-set `CHIPEN=1` once the fault clears (UVLO: VCC recovers above threshold; OTP: die temp drops below 120°C hysteresis point) — LED config registers are not documented as being cleared by this automatic transition, only by the RSTR 0x55 software reset.

## Full register map

| Addr | Name | R/W | Default | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0x00 | RSTR | WR | 0x09 (RO id) | RSTR[7:0] (write 0x55 = reset) ||||||||
| 0x01 | GCR1 | WR | 0x00 | LIE2 | LIE1 | LIE0 | – | UVLOIE | OTPIE | – | CHIPEN |
| 0x02 | ISR | R, clr-on-read | 0x10 | LIS2 | LIS1 | LIS0 | PUIS | UVLOIS | OTPIS | – | – |
| 0x03 | PATST | R | 0x00 | – | – | – | – | – | ST2 | ST1 | ST0 |
| 0x04 | GCR2 | WR | 0x00 | DUVP | DOTP | UVDIS | OTDIS | UVTH[1] | UVTH[0] | IMAX[1] | IMAX[0] |
| 0x30 | LCTR | WR | 0x00 | – | – | FREQ | – | EXP | LE2 | LE1 | LE0 |
| 0x31 | LCFG0 | WR | 0x00 | SYNC | FO | FI | MD | CUR[3] | CUR[2] | CUR[1] | CUR[0] |
| 0x32 | LCFG1 | WR | 0x00 | – | FO | FI | MD | CUR[3:0] ||||
| 0x33 | LCFG2 | WR | 0x00 | – | FO | FI | MD | CUR[3:0] ||||
| 0x34 | PWM0 | WR | 0x00 | PWM[7:0] ||||||||
| 0x35 | PWM1 | WR | 0x00 | PWM[7:0] ||||||||
| 0x36 | PWM2 | WR | 0x00 | PWM[7:0] ||||||||
| 0x37 | LED0T0 | WR | 0x00 | T1[3:0] |||| T2[3:0] ||||
| 0x38 | LED0T1 | WR | 0x00 | T3[3:0] |||| T4[3:0] ||||
| 0x39 | LED0T2 | WR | 0x00 | – | T0[3:0]... ||| REPEAT[3:0] ||||
| 0x3A | LED1T0 | WR | 0x00 | T1[3:0] |||| T2[3:0] ||||
| 0x3B | LED1T1 | WR | 0x00 | T3[3:0] |||| T4[3:0] ||||
| 0x3C | LED1T2 | WR | 0x00 | – | T0[3:0]... ||| REPEAT[3:0] ||||
| 0x3D | LED2T0 | WR | 0x00 | T1[3:0] |||| T2[3:0] ||||
| 0x3E | LED2T1 | WR | 0x00 | T3[3:0] |||| T4[3:0] ||||
| 0x3F | LED2T2 | WR | 0x00 | – | T0[3:0]... ||| REPEAT[3:0] ||||

Note: for the `LEDxT2` registers, T0 actually occupies bits 7:4 same as the others (the datasheet's own bit-position column for that row is slightly inconsistent typography, but the detailed register description confirms T0=bits7:4, REPEAT=bits3:0).

## Bit field details

**GCR1 (0x01)** — `LIEx`: enable pattern-complete interrupt per channel (1=enable). `UVLOIE`/`OTPIE`: enable those interrupts. `CHIPEN`: 0=standby, 1=active.

**ISR (0x02)**, read clears all flags — `LISx`: LEDx pattern-complete occurred. `PUIS`: power-on reset occurred. `UVLOIS`: UVLO event. `OTPIS`: over-temp event.

**PATST (0x03)** — `STx`: 1 = pattern currently running on LEDx, 0 = not running. Read-only, doesn't clear. Useful for polling instead of/alongside INTN.

**GCR2 (0x04)**:
- `DUVP`/`DOTP`: disable the *protection action* (auto CHIPEN clear) for UVLO/OTP while leaving detection itself enabled — device keeps running through the fault.
- `UVDIS`/`OTDIS`: disable *detection* entirely.
- `UVTH[1:0]`: 00=2.0V(default) / 01=2.1V / 10=2.2V / 11=2.3V.
- `IMAX[1:0]`: **00=15mA(default) / 01=30mA / 10=5mA / 11=10mA** — note this encoding is non-monotonic, easy to mis-order in a driver enum, double check against your board's expected default.

**LCTR (0x30)**:
- `FREQ`: 0=250Hz PWM carrier (default), 1=125Hz.
- `EXP`: 0=exponential PWM transition curve (default), 1=linear.
- `LE2/LE1/LE0`: master per-channel enable (0=off, 1=on) — this is the switch that actually starts a pattern once `LCFGx.MD=1`, or gates manual-mode output.

**LCFG0/1/2 (0x31/0x32/0x33)**:
- `SYNC` (LCFG0 only, bit7): 1 = sync control mode — LED1/LED2 PWM duty is sourced from LED0's PWM register; `PWM1`/`PWM2` register writes are ignored while sync is active. `LCFG0.MD` then controls operating mode globally for all three channels.
- `FO`/`FI`: fade-out/fade-in enable, **manual mode only**. Fade time is taken from that channel's `T3` (fall) / `T1` (rise) fields in its `LEDxT0`/`LEDxT1` registers.
- `MD`: 0=manual mode (direct PWM register control), 1=pattern mode (internal breathing controller).
- `CUR[3:0]`: per-channel current level, 0–15. `Io = IMAX_mA × CUR / 15` when that channel's PWM register = 255.

**PWMx (0x34–0x36)**: 8-bit duty level, 0–255. In manual mode this is the direct brightness control (with optional FI/FO smoothing). In pattern mode this register is driven internally by the pattern engine — don't fight it.

**LEDxT0/T1/T2**: 4-bit fields, each maps through the same non-linear lookup table below. `T0`=startup delay, `T1`=rise time, `T2`=on time, `T3`=fall time, `T4`=off time.

| Code | Time | Code | Time |
|---|---|---|---|
| 0000 | 0.00s (T1/T3 default) or 0.04s (T0/T2/T4 default) | 1000 | 2.1s |
| 0001 | 0.13s | 1001 | 2.6s |
| 0010 | 0.26s | 1010 | 3.1s |
| 0011 | 0.38s | 1011 | 4.2s |
| 0100 | 0.51s | 1100 | 5.2s |
| 0101 | 0.77s | 1101 | 6.2s |
| 0110 | 1.04s | 1110 | 7.3s |
| 0111 | 1.6s | 1111 | 8.3s |

Note the asymmetric reset default: `T1`/`T3` (rise/fall) default to code 0000 = **0.00s**, while `T0`/`T2`/`T4` (delay/on/off) default to code 0000 = **0.04s**. Same 4-bit code, different meaning depending on which field — don't build one shared "0000→0" assumption into a lookup helper.

`REPEAT[3:0]` (in `LEDxT2`, bits 3:0): 0000 = loop forever, 0001–1111 = repeat 1–15 times. `ISR.LISx` sets once the programmed repeat count finishes (not applicable if REPEAT=0000).

## Breathing waveform shape (pattern mode)

```
   ┌──T1──┬──T2──┬──T3──┬────T4────┐
  /        ▔▔▔▔▔▔        \
 T0        (rise)  (on)  (fall)   (off)   → repeats
```
T0 = one-time delay before the first cycle starts; T1–T4 then repeat.

## Recommended driver sequences

**Bring-up**
1. `read(0x00)` → expect 0x09, confirms bus/address correct.
2. `write(0x04, GCR2_val)` — set `IMAX`, `UVTH`, and any protection-disable bits, while still in standby (or right after enabling active, since GCR2 isn't in the standby-writable exception list — see caveat below).
3. `write(0x01, GCR1_val | CHIPEN)` — enter active mode. Also set `LIEx`/`UVLOIE`/`OTPIE` here if using `INTN`.

> Caveat: the datasheet states only `RSTR` and `GCR1` are writable in standby. It doesn't explicitly confirm `GCR2` is writable pre-`CHIPEN`. Safest driver behavior: set `CHIPEN=1` first, then write `GCR2`/`LCTR`/`LCFGx` etc.

**Manual (static/direct) brightness, one channel**
1. `write(LCFGx, MD=0, CUR=<0-15>)` — set current level, manual mode.
2. `write(PWMx, <0-255>)` — set brightness.
3. `write(LCTR, LEx=1)` (OR into existing LCTR value — don't clobber the other two channels' enable bits or FREQ/EXP).

**Manual mode with smooth fade**
1. Configure `LEDxT0`/`LEDxT1` — `T1` = fade-in time, `T3` = fade-out time.
2. `write(LCFGx, FI=1, FO=1, MD=0, CUR=...)`.
3. Each subsequent `write(PWMx, new_value)` now ramps smoothly over T1 (rising) or T3 (falling) instead of stepping.

**Single-channel breathing pattern**
1. `write(LEDxT0)`, `write(LEDxT1)` — T1..T4.
2. `write(LEDxT2, T0=..., REPEAT=...)` — writing this register is what actually starts the pattern engine timing.
3. `write(LCFGx, MD=1, CUR=...)`.
4. `write(LCTR, LEx=1)`.

**All three channels synchronized start** (per datasheet's explicit recipe)
1. `write(LCTR, 0x00)`.
2. `write(LCFGx, MD=0)` for x=0,1,2.
3. Configure `LEDxT0/T1/T2` (T0–T4, REPEAT) for all three channels.
4. `write(LCFGx, MD=1)` for x=0,1,2.
5. `write(LCTR, 0x07)` — sets LE0/LE1/LE2 simultaneously, all three patterns start together.

**RGB sync mode** (one set of timing/PWM drives all three, currents can still differ)
1. `write(LCFG0, SYNC=1, MD=<0 or 1>, CUR=...)`.
2. `write(LCFG1, CUR=...)`, `write(LCFG2, CUR=...)` — currents still per-channel.
3. Control everything else (PWM0 or LED0Tx, LE0) — LED1/LED2 duty follows LED0 automatically; their own PWM/pattern registers are ignored while SYNC=1.

**Interrupt service (INTN, open-drain active-low)**
1. On INTN falling edge, `read(0x02)` (ISR) — this both tells you the cause and clears the flags.
2. Check `PUIS` (unexpected reset — reinitialize registers), `UVLOIS`/`OTPIS` (fault — device auto-dropped to standby, re-arm `CHIPEN` once condition clears), `LISx` (pattern finished its repeat count on channel x).

## Things likely to bite a driver implementation

- **`IMAX` encoding is non-monotonic** (00=15mA, 01=30mA, 10=5mA, 11=10mA) — don't assume binary-weighted ordering.
- **ISR is clear-on-read** — a driver that reads ISR for logging/debug will silently eat real interrupt flags if it doesn't act on them in that same read.
- **T0/T2/T4 vs T1/T3 share a lookup table but different reset defaults** (0.04s vs 0.00s) for the same 0000 code — don't hardcode "0000 = 0".
- **`LCTR` is a shared register for all three channels' enable bits plus global FREQ/EXP** — a driver must read-modify-write it (or track shadow state) rather than blindly overwriting when toggling one channel.
- **SYNC mode silently ignores `PWM1`/`PWM2` writes** — if a driver lets a caller set LED1/LED2 brightness independently while SYNC=1 is active, those writes will appear to succeed but have no effect.
- **Register auto-increment on multi-byte I2C transactions** — burst writing e.g. `LED0T0`,`LED0T1`,`LED0T2` in one transaction is valid and saves I2C overhead versus 3 separate single-byte writes.
- Datasheet doesn't explicitly state whether `LCFGx`/`PWMx`/pattern registers survive an automatic UVLO/OTP-triggered standby transition (as opposed to the explicit 0x55 software reset, which definitely clears everything) — worth a bench test on your actual part before relying on "fault clears → just re-set CHIPEN" without reconfiguring.
