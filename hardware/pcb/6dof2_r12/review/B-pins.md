# B — Pin assignment review, r11 baseline (2026-09-05)

Source of truth: `data/pads_r11.json` / `data/components_r11.json` (594 pads, 123 parts), cross-read
against `V2_REQUIREMENTS.md` §2/§4/§6, `V2_DESIGN_REVIEW.md`, `atopile/main.ato`, the firmware
`Controller/include/helpers.h` (`phoenix`), and the datasheets cited inline (ESP32-S3 Series DS v2.2,
ESP32-S3-WROOM-1 DS v1.8, W5500 DS v1.1.0 + WIZnet "RJ45 with mag" reference schematic 2015-04-06,
AASD-series manual §2.2.3, XL1509, AM26LS31, LTV-847, USBLC6-2SC6).

## 1. Verdict

**No pin on the board needs to move and no net needs to change.** Every ESP32-S3 module pin, every
W5500 pin, all 48 AM26LS31 pins, both opto arrays, the six DB25s, the USB/ESD path, the buttons and
the power tree match the datasheets and the §6 "verified correct" list. The strapping pins are
untouched, no GPIO is used that the N8R2's flash/PSRAM claims, and the 12 STEP/DIR lines are held
low by R46–R57 through every documented power-up glitch, so no servo can see a step edge at boot.
The two things that must be fixed are not copper: **(F1) the firmware's E-stop polarity is inverted
relative to this hardware** (the board is fail-safe only if HIGH means stop), and **(F2) C15's LCSC
number in the BOM is a 10 µF part where the W5500 needs 4.7 µF on TOCAP**. One value in the task
brief is wrong and is corrected below: R30–R35 are **2.2 kΩ**, not 220 Ω, and 2.2 kΩ is right.

## 2. ESP32-S3-WROOM-1 (U2) pin table

Module numbering per WROOM-1 DS Table 3-1 (verified: 13/14 = IO19/IO20 = USB D−/D+, 36/37 = RXD0/TXD0,
3 = EN, 27 = IO0). Glitch data from ESP32-S3 DS Table 2-2; reset pulls from Table 2-1 / Table 3-1.

| Pin | GPIO | Net | Role / dir | Boot concern |
|---|---|---|---|---|
| 1, 40, 41 | GND | GND | — | — |
| 2 | 3V3 | VCC3V3 | supply | — |
| 3 | EN | ESP_EN | R4 10 k↑, C14 1 µF, SW1→GND | none; matches Espressif's recommended 10 k/1 µF RC |
| 4 | IO4 | M0S_G | STEP M0 out | 60 µs low-level glitch only; line held low by R46 10 k → no edge |
| 5 | IO5 | M1S_G | STEP M1 out | same (R48) |
| 6 | IO6 | M2S_G | STEP M2 out | same (R50) |
| 7 | IO7 | M3S_G | STEP M3 out | same (R52) |
| 8 | IO15 | M2_ALARM | opto in, R38 10 k↑ | low glitch into a pulled-up input: harmless (XTAL_32K_P unused) |
| 9 | IO16 | M3_ALARM | opto in, R39 10 k↑ | same (XTAL_32K_N unused) |
| 10 | IO17 | M5D_G | DIR M5 out | low glitch only; held low by R57 |
| 11 | IO18 | M4_ALARM | opto in, R40 10 k↑ | low **and 60 µs high** output glitch; may briefly source into a conducting opto — both parts tolerate it (see N7) |
| 12 | IO8 | M4S_G | STEP M4 out | low glitch only; held low by R54 |
| 13 | IO19 | USB_DN_MCU | native USB D− (via D6) | documented USB-pin glitches; expected |
| 14 | IO20 | USB_DP_MCU | native USB D+ (via D6) | expected |
| 15 | IO3 | NC | strap: JTAG source | floating = datasheet default; only read if EFUSE_STRAP_JTAG_SEL burnt |
| 16 | IO46 | NC | strap: boot/ROM print | internal WPD → 0 = normal boot; nothing loads it |
| 17 | IO9 | M5S_G | STEP M5 out | low glitch only; held low by R56 |
| 18 | IO10 | M0D_G | DIR M0 out | held low by R47 |
| 19 | IO11 | M1D_G | DIR M1 out | held low by R49 |
| 20 | IO12 | M2D_G | DIR M2 out | held low by R51 |
| 21 | IO13 | M3D_G | DIR M3 out | held low by R53 |
| 22 | IO14 | M4D_G | DIR M4 out | held low by R55 |
| 23 | IO21 | ESTOP_MCU | E-stop in (R1 10 k↑ on ESTOP, R60 1 k, C36 100 nF) | not in glitch table; no strap role |
| 24, 25 | IO47/48 | NC | — | — |
| 26 | IO45 | NC | strap: VDD_SPI | internal WPD → 3.3 V flash; on PSRAM modules eFuse-fixed anyway |
| 27 | IO0 | ESP_IO0 | strap: boot; R5 10 k↑, SW2→GND | WPU + R5; only SW2 pulls it low |
| 28 | IO35 | MOSI | W5500 SPI | free on N8R2 (Quad PSRAM); **PSRAM line on R8/R16V** (N1) |
| 29 | IO36 | SCLK | W5500 SPI | same |
| 30 | IO37 | MISO | W5500 SPI | same |
| 31 | IO38 | SCSN | W5500 CS | W5500 has internal pull-up on SCSn → no float during reset |
| 32 | IO39 | INTN | W5500 INTn in | MTCK; WPU at reset if pad-JTAG not disabled — harmless on an input |
| 33 | IO40 | M5_ALARM | opto in, R41 10 k↑ | MTDO; pin-JTAG unused (USB-JTAG is the debug path) |
| 34, 35 | IO41/42 | NC | — | — |
| 36 | RXD0 (IO44) | U0RXD | console in ← J1.3 | WPU at reset; header may float safely |
| 37 | TXD0 (IO43) | U0TXD | console out → J1.2 | ROM boot log appears here; fine |
| 38 | IO2 | M1_ALARM | opto in, R37 10 k↑ | low glitch, harmless |
| 39 | IO1 | M0_ALARM | opto in, R36 10 k↑ | low glitch, harmless |

Firmware map (`helpers.h` PCBv2): STEP 4–9, DIR 10/11/12/13/14/17, ETH MOSI 35 / MISO 37 / SCLK 36 /
CS 38 / INT 39 — identical to the board. `ESTOP_PIN 20` is stale (see F1).

## 3. Findings

### F1 — Defect (system): E-stop polarity in firmware is inverted for this hardware; input still disabled
- Hardware: `ESTOP` = J2.1 with R1 10 kΩ to VCC3V3; J2.2 = GND. An NC loop across J2 reads **LOW
  when closed (normal) and HIGH when opened or when a wire breaks** — that is the fail-safe sense.
- Firmware (`Controller/include/helpers.h`, `phoenix`): `ESTOP_PIN 20`, `ESTOP_ACTIVE_STATE 0`
  ("active when LOW, normally closed"), and `main.cpp` triggers on `prev_state == 1 && current ==
  ESTOP_ACTIVE_STATE`; for PCBv2 the `initDebounceButton` call is commented out because GPIO20 is
  USB D+. With this hardware, ACTIVE_STATE 0 would treat the closed loop as "stop" and an open/broken
  loop as "safe".
- Remedy (firmware only, no board change): `ESTOP_PIN 21`, `ESTOP_ACTIVE_STATE 1`, edge test
  `prev_state == 0 && current == 1`, and re-enable `initDebounceButton` for PCB_VERSION 2 (the
  internal pull-up it enables is in parallel with R1 — harmless). Keep R1/R60/C36 as they are.
  Cross-stream: none for copper; owner/firmware.

### F2 — Should-fix (BOM): C15 (W5500 TOCAP) LCSC part is 10 µF, comment says 4.7 µF
- `data/BOM_r11.csv`: `0805X475K100CT, C15, C0805, C15850`. C15850 is CL21A106KAYNNNE (10 µF), the
  same number used for C7/C11/C19/C22/C31. W5500 DS pin 20: "must be connected to a 4.7 µF capacitor".
- Remedy: set C15's LCSC number to the 4.7 µF 0805 part (atopile pinned C123653, 0805X475K100CT);
  integrator to confirm stock. 10 µF would very likely still work, but ship what the datasheet says
  and make the BOM self-consistent. Cross-stream: C (BOM line), no copper change.

### F3 — Should-fix (low, document or accept): USB-only power back-drives U6's OUTPUT and FB pins
- With no 12 V, VBUS→D3 puts ≈4.65 V on VCC5, i.e. on U6.3 (FB) and, via L1, on U6.2 (OUTPUT) while
  U6.1 (VIN) is 0 V. XL1509 abs-max: V_FB = −0.3 V … V_IN. On paper this is out of range; in practice
  the FB→VIN ESD path charges C1 to ≈4 V and the part idles.
- Remedy: none on copper (a series Schottky on the buck output would cost 0.35 V in 12 V mode and
  push the AM26LS31 below 4.75 V). Accept as bench-only mode and say so on silk near J11
  ("USB = program/bench only") — stream E. Cross-stream: E (silk text).

### F4 — Should-fix (low): 3V3 headroom on USB-only power is ~0.1–0.3 V
- VCC5 = 5.0 − V_F(D3) ≈ 4.5–4.65 V under load; AMS1117 dropout 1.1–1.3 V at 0.5–0.8 A → 3.2–3.5 V
  during Wi-Fi + W5500 peaks. Marginal, not broken. Everything else on VCC5 in this mode: U3–U5
  (below their 4.75 V min, known), D4/R18 LED (fine), U6 (F3).
- Remedy: drop-in pin-compatible SOT-223 LDL1117S33R (ST, 1.2 A, 350 mV dropout) if stocked;
  otherwise document "no Wi-Fi/Ethernet load on USB power". No copper change.

### Notes (no action required)
- N1 Module lock: WROOM-1 DS Table 3-1 note b — IO35/36/37 are Octal-PSRAM lines on R8/R16V modules
  only; N8R2 is 8 MB Quad flash + 2 MB Quad PSRAM (Table 1-1), so they are free. Keep U2 pinned to
  C2913204; never substitute an R8.
- N2 SPI path: firmware uses `SPI3_HOST`; on the S3 that (and any use of IO35–38) goes through the
  GPIO matrix, fine at the 20 MHz in use, ceiling ≈40 MHz. The IO_MUX set for SPI2 is IO10–13, which
  DIR occupies — a deliberate, acceptable trade.
- N3 Boot behaviour of STEP/DIR: ESP32-S3 DS Table 2-2 lists only *low-level* 60 µs glitches on
  GPIO4–14/17; Table 2-1 shows none of them has a pull-up at reset. With R46–R57 (10 k to GND) on
  the driver inputs (and a 33 Ω series R between GPIO and that node), the AM26LS31 inputs sit at 0 V
  through power-up, reset and download mode. No step edge is possible before firmware drives them.
- N4 W5500 hard reset is RC-only (R11 10 k / C16 100 nF, τ = 1 ms ≥ 500 µs low); the MCU cannot
  reset it. ESP-IDF W5500 driver must run with `reset_gpio_num = -1` (software reset) — firmware item.
- N5 Alarm inputs are blind without 12 V (R30–R35 hang off VINRAW): all six read HIGH on USB power.
  Firmware should not treat that as six drive alarms in bench mode.
- N6 Opto LED reverse exposure: LTV-847 V_R max 6 V; a mis-wired drive output (24 V) on DB25.23
  could exceed it. An anti-parallel 1N4148 per channel would close it; optional.
- N7 GPIO18 (M4_ALARM) has a 60 µs high-level output glitch at power-up; if the opto is conducting
  the GPIO sources a few mA into it for 60 µs. Within both parts' ratings.
- N8 ETH LEDs: R6/R7 1 kΩ vs 330 Ω in the WIZnet reference → ≈1.3 mA, dim but visible. Optional 330–470 Ω.
- N9 E-stop loop is 3.3 V / 0.33 mA wetting current over an external cable; the 1 k/100 nF filter
  plus 50 ms firmware debounce make it workable. A 24 V opto-isolated loop would be more robust in a
  future revision; not for r12.

## 4. Confirmed correct

**ESP32-S3 / USB / buttons / console**
- USB: J11 A6/B6 = USB_DP, A7/B7 = USB_DN, A4B9/B4A9 = VBUS, A5 = CC1→R2 5.1 k, B5 = CC2→R3 5.1 k
  (separate Rd, correct UFP), SBU NC, shell = GND. D7 SMF5.0A across VBUS.
- D6 USBLC6-2SC6 in-line exactly per §4.1: 1 = USB_DP / 6 = USB_DP_MCU (I/O1 line), 3 = USB_DN /
  4 = USB_DN_MCU (I/O2 line), 2 = GND, 5 = VBUS. USB_DP_MCU → U2.14 = IO20 = USB_D+, USB_DN_MCU →
  U2.13 = IO19 = USB_D− (WROOM-1 DS Table 3-1). Pair is not swapped.
- EN: R4 10 k→3V3 + C14 1 µF + SW1→GND; IO0: R5 10 k→3V3 + SW2→GND. Both buttons pull to GND;
  hold BOOT + tap RESET = download mode; USB-Serial-JTAG gives automatic entry without them.
- Strapping: GPIO0 (WPU + R5), GPIO3 (floating, default), GPIO45/46 (NC, internal WPD). No signal
  is placed on any strapping pin. IO26–IO32 are not bonded out on the module, so they cannot be
  misused. UART0: TXD0 (IO43) → J1.2, RXD0 (IO44) → J1.3, J1.1 = GND; CH340 contention is gone.

**W5500 (U8) vs datasheet v1.1.0 and the "RJ45 with mag" reference**
- SPI: 32 SCSn ← IO38, 33 SCLK ← IO36, 34 MISO → IO37, 35 MOSI ← IO35, 36 INTn → IO39, 37 RSTn
  (internal pull-up) + R11/C16. SCSn also has an internal pull-up.
- Supplies: AVDD 4/8/11/15/17/21 = VCC3V3A behind L2 (600 Ω bead) with C13/C20/C21/C26–C29 100 nF
  + C22 10 µF; VDD 28 = VCC3V3 (C30 100 nF, C31 10 µF, C17/C18); AGND 3/9/14/16/19/48 and GND 29.
- PMODE2/1/0 = pins 43/44/45 tied to VCC3V3 → 111 = all-capable auto-negotiation (also the
  internal-pull-up default). RSVD 23 → GND ("must be tied to GND"); RSVD 38–42, NC 12/13/46/47,
  DNC 7 and VBG 18 all open. TOCAP 20 → C15 4.7 µF (see F2 for the BOM number); 1V2O 22 → C12 10 nF;
  EXRES1 10 → R10 12.4 k 1 % to GND.
- Crystal: Y1 25 MHz 3225 (pins 1/3 terminals, 2/4 GND), C23/C24 18 pF, R58 1 MΩ XI–XO — identical
  to the WIZnet reference (18 pF / 1 M).
- PHY front end is the reference verbatim: TXP/TXN → jack 3/4 direct, R42/R43 49.9 Ω to VCC3V3A;
  TX centre tap jack 5 (ETH_TCT) → R59 10 Ω → VCC3V3A with C32 22 nF to GND; RX: 49.9 Ω R44/R45 on
  the **chip side** to a common node ETH_RX_BIAS that carries C35 10 nF to GND and the jack's RX
  centre tap (pin 6), with C33/C34 6.8 nF in series to jack 7/8 (ETH_JRXP/JRXN). Chassis 10/13/14
  → C25 1 nF/2 kV → GND only. LEDs: jack 1(+)/2(−) yellow ← R7 1 k / ACTLED(27); jack 12(+)/11(−)
  green ← R6 1 k / LINKLED(25); both W5500 LED outputs are active-low sinks.

**AM26LS31 ×3 (U3/U4/U5) and DB25s** — §6 confirmed
- Pinout 1A/1Y/1Z/G/2Z/2Y/2A/GND/3A/3Y/3Z/Ḡ/4Z/4Y/4A/VCC. On every driver: 1 = MxS_I → 2 = MxS_P,
  3 = MxS_N; 7 = MxD_I → 6 = MxD_P, 5 = MxD_N; 9 = M(x+1)S_I → 10/11 P/N; 15 = M(x+1)D_I → 14/13 P/N.
  Y (non-inverting) is always the _P net. Pin 4 = VCC5, pin 12 = GND, pin 16 = VCC5, pin 8 = GND.
- U3 = M0/M1, U4 = M2/M3, U5 = M4/M5; J3…J8 = M0…M5 in order. Every connector: 3 = MxS_P (PP+),
  14 = MxS_N (PP−), 4 = MxD_P (PD+), 5 = MxD_N (PD−), 10 = GND (COM), 23 = Mx_ALARM_DRV (SigOUT2);
  2 (PV) and 6 (SRV-ON) open. All 24 differential nets have exactly two nodes (no driver-side
  termination).
- Inputs: GPIO → 33 Ω (R20–R25 STEP, R12–R17 DIR) → driver input node, with R46–R57 10 k to GND on
  that node. VIH 2.0 V is met by 3.3 V drive; VIL 0.8 V is met at boot (worst case 3.3 V through a
  45 k internal WPU into 10 k → 0.6 V).

**Opto inputs (U9/U10 LTV-847S)**
- Pinout 1/3/5/7 anodes, 2/4/6/8 cathodes, 9/11/13/15 emitters, 10/12/14/16 collectors, channel k
  pairs pin (2k−1, 2k) with (17−2k, 18−2k). U9 ch1–4 = M0–M3, U10 ch1–2 = M4/M5; emitters GND,
  collectors → Mx_ALARM with R36–R41 10 k to VCC3V3. Correct.
- **R30–R35 are 2.2 kΩ** (0603WAF2201T5E = 220×10¹ Ω; LCSC C4190), not 220 Ω. Assumption: AASD
  SigOUT2 is a Darlington-photocoupler open collector returning to COM (CN2 pin 10), rated ≤70 mA
  and ≤25 V (AASD manual §2.2.3), V_CE(sat) ≈ 1 V. I_F = (11.6 − 1.2 − 1.0) / 2.2 k ≈ 4.3 mA
  (≈40 mW in the 0603): within LTV-847 I_F(max) 50 mA, and with CTR ≥ 50 % at 5 mA it sinks
  ≥ 2 mA against a 0.33 mA pull-up — saturates. 220 Ω would have been ≈43 mA and 0.4 W and would
  have been a defect; 2.2 k is correct and consistent with the r11 design review and atopile intent.
  Feeding from VINRAW (behind D1) is the deliberate r11 fix for reverse-input protection.

**Power tree**
- J9.1 VIN_J → F1 (1.1 A PTC, 16 V) → VIN12 → D1 SS34 (A = pin 2 = VIN12, K = pin 1 = VINRAW) →
  VINRAW: C1 100 µF/25 V + C4 100 nF → U6.1. All three SS34 use pin 1 = K consistently (D2 K = SWNODE,
  A = GND; D3 K = VCC5, A = VBUS).
- U6 XL1509-5.0E1 SOP-8: 1 VIN = VINRAW, 2 OUTPUT = SWNODE, 3 FB = VCC5 (fixed-5 V part: FB straight
  to the output, no divider expected or present), 4 ON/OFF = GND (low = ON; threshold 0.6 V), 5–8 GND.
  L1 68 µH, D2 SS34 catch diode on SWNODE, C2 220 µF/25 V + C5/C6/C8/C9/C10 100 nF + C11 10 µF on
  VCC5. 150 kHz, 12 V-only input by rating of C1 and F1 (as documented).
- U7 AMS1117-3.3 SOT-223: 1 GND, 2 VOUT = VCC3V3, 3 VIN = VCC5, tab 4 = VCC3V3. Output: C3 100 µF
  6.3 V X5R 1206 (≈50–60 µF at 3.3 V bias) + C7/C19/C31 10 µF + 100 nF locals — exceeds the 22 µF
  minimum; ceramic ESR has not been a stability problem for this family in practice.
- VBUS → D7 SMF5.0A, D6.5, D3 → VCC5 ORing; with 12 V present D3 is reverse-biased (5.0 vs ≤5.0 V),
  no back-feed to the host.

**E-stop**: J2.1 = ESTOP (R1 10 k→3V3, R60 1 k) → ESTOP_MCU (C36 100 nF) → U2.23 = **GPIO21** (no
strap/USB/JTAG role). τ = 100 µs filter; open loop reads HIGH = stop — fail-safe provided F1 is done.

## 5. Proposed changes (for the integrator)

| # | Item | From | To | Copper? |
|---|---|---|---|---|
| F1 | firmware `helpers.h` / `main.cpp` | ESTOP_PIN 20, ACTIVE 0, disabled | ESTOP_PIN 21, ACTIVE 1, edge 0→1, enabled for PCBv2 | no |
| F2 | BOM C15 LCSC # | C15850 (10 µF) | C123653 (0805X475K100CT 4.7 µF) or equivalent basic | no |
| F3 | silk near J11 | — | "USB: program/bench only" | silk only (E) |
| F4 | U7 part (optional) | AMS1117-3.3 | LDL1117S33R (SOT-223 drop-in) if stocked | no |

Net moves: **none**. Resistor value changes: **none**.
