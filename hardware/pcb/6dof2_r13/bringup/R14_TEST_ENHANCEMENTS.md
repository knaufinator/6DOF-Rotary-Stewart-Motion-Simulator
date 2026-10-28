# R14 test and bring-up enhancements (from R13 bring-up)

2026-10-12. A request list for the R14 design and firmware teams, collected while bringing
up the R13 first article. It extends the "Requests for r14" list in `TEST_POINTS.md`.

Nothing here edits R14 design files. Each item names the R13 evidence behind it; R14
owners decide placement, part choice and acceptance.

Priority: **P1** = needed so R14 bring-up does not repeat an R13 blocker. **P2** = strong
value. **P3** = nice to have.

## Hardware — design for test

| Id | P | Request | R13 evidence |
|---|---|---|---|
| T1 | P1 | Labelled 1.5 mm through-hole test points: GND ×2 (terminals, driver bank), VINRAW, VCC5, LOGIC_5V, VCC3V3, DRIVE_ENABLE, ESTOP_MCU, STEP0/DIR0 logic before the line driver | No TPs; TP1–TP14 had to be mapped onto IC pins and capacitor leads |
| T2 | P1 | Scope ground posts beside the DB25 bank and the line drivers | Probing J3 pins 3/14 against pin 10 only |
| T3 | P2 | Keep a TP grid usable by a pogo fixture (2.54 mm multiples, one side) | Enables fixture F1 below |
| D1 | P1 | **DRIVE_ENABLE readback** to a spare non-strapping GPIO (through a resistor), plus an "OUTPUTS ENABLED" LED | Only latch state with no readback (K6). Every R13 arm needed a meter, and the R13 no-motion run could not tell "not armed" from "wrong socket" |
| D2 | P1 | **VCC5 sense** into an ADC1 channel (divider), so firmware reports the driver rail and refuses `ARM` below threshold with a clear reason | Supervisor threshold 4.93–4.96 V is invisible to firmware; "arm OK but nothing out" was a candidate cause |
| D3 | P2 | Alarm opto outputs to LEDs (or a header), one per axis | Step 10 polarity learning needs a meter today |
| D4 | P2 | Optional STEP-output sense: one DB25 STEP+ through a divider to a PCNT-capable GPIO | Independent pulse count (F03) without a scope |
| D5 | P3 | Board-ID / serial EEPROM or OTP-written serial | Traceability across articles in logs |

## Hardware — safety and drive interface

| Id | P | Request | R13 evidence |
|---|---|---|---|
| S1 | P1 | Silkscreen naming that cannot be misread: `AXIS 0 / M1` (or switch all docs to one convention), and the same on the lid | Bench "motor 1" cable went on `MOTOR 1` (J4) instead of `MOTOR 0` (J3), twice |
| S2 | P1 | Alarm LED resistors: the 1.5 k 1206 already in the R14 schematic, synced to the PCB. Size for **continuous** current: the AASD-15A ALM output conducts while the drive is healthy, so this is the normal state, not a fault state | R30–R35 220 Ω 0603: about 49 mA, 0.53 W at 12 V (K4). 2026-10-12: with pin 23 connected and the drive powered with no alarm shown, the probe read alarm0=0 (opto conducting); with the drive off, it read alarm0=1. The owner accepted the R13 overstress to continue motion testing |
| S2b | P1 | Firmware/docs: on the AASD-15A, alarm input **0 = healthy** (output conducting), 1 = fault or drive unpowered. Name it `drive_ok` or similar, not "alarm asserted" | Same observation; probe `in` text labels 0 as "alarm asserted" |
| S3 | P1 | SRV-ON (DB25 pin 6) driven from the latch or a separate supervised output, with a jumper to opt out | Drives must self-enable today (Pn003=1). Pulse inhibit is not torque-off (F05) |
| S4 | P2 | Keyed E-STOP connector (2-pin) distinct from the 12 V terminal; keep J2 and J9 visually different | J9 and J2 are the identical KF128 part; the owner's first "J2" link went on J1 instead |
| S5 | P2 | DB25 shell bonded to a defined chassis/shield point; cable-shield strategy documented | K1: link drops only with the drive powered and cabled |

## Hardware — USB and power robustness

| Id | P | Request | R13 evidence |
|---|---|---|---|
| U1 | P1 | USB ESD array and series resistors on D+/D−; common-mode choke footprint (DNP option) | Native-USB drops during flashing near a powered drive (K1) |
| U2 | P1 | Verify BOOT (GPIO0) and EN buttons: footprints, pull-ups, debounce RC per Espressif reference | One BOOT entry did not register (`boot:0xb`, K2) |
| U3 | P2 | Flash/download path that survives drive EMI: keep J1 UART header and add a "BOOT via jumper" option | Recovery when native USB is disturbed |
| P1 | P1 | 12 V input current monitor or a fuse status indication | Bench uses a current-limited supply; the field will not |
| P2 | P2 | LOGIC_5V headroom on USB: lower-drop OR-ing (ideal-diode or Schottky choice) | LOGIC_5V 4.0 V on USB only (K3) |

## Firmware

| Id | P | Request | Notes |
|---|---|---|---|
| W1 | P1 | Port `R13_BENCH_ONE_AXIS` to the R14 pin map. Add DRIVE_ENABLE readback (D1) and VCC5 (D2) to `BENCH:STATUS`, and refuse `BENCH:ARM` when VCC5 is low | Bench mode becomes self-checking |
| W2 | P1 | Built-in `SELFTEST` in the controller: rails (D2), latch readback (D1), loop, alarms, W5500 identity, flash/PSRAM | Same checks as probe `selftest`, in the production image |
| W3 | P2 | Probe firmware rebuilt for R14 with the same command set (`in`, `arm`, `pulse`, `tone`, `mon`) plus non-blocking trains (R13 E1) | The bench tool works unchanged |
| W4 | P2 | Every image reports its git SHA (`id` / `VERSION`) | Log traceability |

## Test tooling and fixtures

| Id | P | Request | Notes |
|---|---|---|---|
| F1 | P2 | Pogo fixture on the T1/T3 grid, with a DB25 loopback/dummy-load plug per connector (LED loads on STEP±/DIR±, pin-23 switch) | Turns phases A–C and E into a few minutes per board |
| F2 | P2 | `r13bench_core.py` generalised to a revision profile (pin map, TP table, step catalogue as data) so R14 adds a profile, not a new tool | Keep one tool across revisions |
| F3 | P3 | Optional SCPI scope/meter capture in the tool | Automatic frequency/count/voltage entry |

## Carry-over checks for R14 bring-up (same as R13, run first)

1. USB only: the latch cannot arm (VCC5 absent).
2. 12 V: VCC5 4.96–5.1 V, and the D2 reading matches the meter.
3. Stop loop seen; every clear is one-way; no automatic re-arm.
4. STEP/DIR only while armed; exact counts on every axis at its own connector.
5. Alarm current 5–8 mA, polarity recorded.
6. Drive readback (Pn003/Pn096/Pn033) before any pulse; bounded first motion.
