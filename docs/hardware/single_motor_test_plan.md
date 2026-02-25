# Single Motor Test Plan — MCPWM Hardware Validation

Validate the ESP32-S3 MCPWM step/dir output driving a single AASD-15A servo via SN75174N RS-422 differential signaling.

**Current firmware**: `STEP_DRIVER_MCPWM` — hardware comparator/generator pulses, 250 kHz max, PCNT counting.

## Signal Chain

```
ESP32-S3 PCBv2 (3.3V MCPWM GPIO) → SN75174N RS-422 driver → twisted pair → AASD-15A CN2 DB25
```

## Bill of Materials

| # | Part | Description | Notes |
|---|------|-------------|-------|
| 1 | **ESP32-S3-DevKitC-1** | Main controller, already in project | GPIO4 = STEP, GPIO10 = DIR (motor 0) |
| 2 | **SN75174N** (DIP-16) ×3 | TI quad differential line driver | DigiKey [296-1735-5-ND](https://www.digikey.com/en/products/detail/texas-instruments/SN75174N/277381). 1 for single motor, 3 for all 6 (4 ch/chip, 2 ch/motor). [Datasheet](https://www.ti.com/product/SN75174) |
| 3 | **Solderless breadboard** | Standard 830-point | For SN75174N + passives |
| 4 | **DIP-16 socket** ×3 | IC socket for SN75174N | Assmann A 16-LC-TT (search DigiKey "CONN IC DIP SOCKET 16POS TIN") |
| 5 | **0.1µF ceramic cap** ×3 | Bypass cap for SN75174N VCC | 1 per chip, place close to VCC/GND pins |
| 6 | **5V power supply** | USB breakout or bench supply | Powers SN75174N (draws <50mA) |
| 7 | **DB25 male solder connector** | Mates with AASD-15A CN2 socket | Only 4 signal wires + GND needed |
| 8 | **Twisted pair cable** (~1m) | Cat5e patch cable works | 2 pairs: one for PULS±, one for DIR± |
| 9 | **Jumper wires** | M-M for breadboard | ESP32 → breadboard connections |
| 10 | **AASD-15A servo driver** | Target motor driver | Already owned |
| 11 | **AC servo motor** | 80ST-M02430 or compatible | Already owned |

### DigiKey Order
| DigiKey P/N | Part | Qty |
|-------------|------|-----|
| 296-1735-5-ND | SN75174N (DIP-16) | 3 |
| (A 16-LC-TT) | DIP-16 IC socket — Assmann WSW (search "CONN IC DIP SOCKET 16POS TIN") | 3 |
| 399-4151-ND | 0.1µF ceramic cap, radial through-hole (KEMET C315C104M5U5TA) | 3 |
| — | DB25 male solder-cup connector | 1 |
| — | DB25 hood/shell | 1 |

## AASD-15A CN2 DB25 Pinout (relevant pins)

| DB25 Pin | Signal | Connect To |
|----------|--------|------------|
| 3 | PULS+ | SN75174N output 1Y (pin 3) |
| 5 | PULS- | SN75174N output 1Z (pin 4) |
| 4 | DIR+ | SN75174N output 2Y (pin 6) |
| 14 | DIR- | SN75174N output 2Z (pin 7) |
| 13 | GND | Common ground (optional, for reference) |

> **Note**: The AASD-15A has optocoupler-isolated differential inputs. The SN75174N RS-422 output (±2V differential) is well within the optocoupler threshold. No termination resistor needed for short cable runs (<3m).

## SN75174N Pinout & Wiring

```
              SN75174N (DIP-16)
              ┌───────────┐
   EN1 (en)  ─┤ 1      16 ├─ VCC (+5V)
    1A (in)  ─┤ 2      15 ├─ EN2 (en)
    1Y (out) ─┤ 3      14 ├─ 4A (in)
    1Z (out) ─┤ 4      13 ├─ 4Y (out)
    2A (in)  ─┤ 5      12 ├─ 4Z (out)
    2Y (out) ─┤ 6      11 ├─ 3A (in)
    2Z (out) ─┤ 7      10 ├─ 3Y (out)
       GND   ─┤ 8       9 ├─ 3Z (out)
              └───────────┘

    Pin 8  = GND
    Pin 16 = VCC (+5V)
    EN1 enables drivers 1 & 2 (active HIGH → tie to VCC)
    EN2 enables drivers 3 & 4 (active HIGH → tie to GND to disable unused)
```

> **Verify pinout against [TI datasheet (SLLS122)](https://www.ti.com/lit/ds/symlink/sn75174.pdf) before wiring.**

### Breadboard Connections

| SN75174N Pin | Connect To | Purpose |
|--------------|------------|--------|
| 16 (VCC) | +5V supply | Power |
| 8 (GND) | Common GND (5V supply + ESP32 GND) | Ground |
| 1 (EN1) | VCC | Enable drivers 1 & 2 (active high) |
| 15 (EN2) | GND | Disable unused drivers 3 & 4 |
| 2 (1A) | ESP32 GPIO4 (STEP_PIN_1) | Step pulse input |
| 3 (1Y) | DB25 pin 3 (PULS+) | Differential step output + |
| 4 (1Z) | DB25 pin 5 (PULS-) | Differential step output - |
| 5 (2A) | ESP32 GPIO10 (DIR_PIN_1) | Direction input |
| 6 (2Y) | DB25 pin 4 (DIR+) | Differential dir output + |
| 7 (2Z) | DB25 pin 14 (DIR-) | Differential dir output - |
| — | 0.1µF cap between pin 16 and pin 8 | Bypass capacitor |

> **SN75174N advantage over AM26LS31**: Both chips produce the same RS-422 differential output. The difference is on the input side: the AM26LS31 has two inputs per channel (A and B) — you must connect the GPIO to A and manually tie B to GND, or the output is undefined. The SN75174N takes a single input per channel and internally generates the differential pair, so fewer wires on the breadboard. It also adds built-in current limiting and thermal shutdown for protection in noisy AC servo environments.

## Wiring Diagram (text)

```
ESP32-S3-DevKitC                 SN75174N (breadboard)           DB25 → AASD-15A
┌─────────────┐                  ┌─────────────────┐            ┌──────────┐
│         GND ├──────────────────┤ GND (pin 8)     │            │          │
│       GPIO4 ├──────────────────┤ 1A  (pin 2)     │            │          │
│      GPIO10 ├──────────────────┤ 2A  (pin 5)     │            │          │
└─────────────┘                  │ EN1 (pin 1)─VCC │            │          │
                                 │ EN2 (pin 15)─GND│            │          │
    +5V supply ──────────────────┤ VCC (pin 16)    │            │          │
    GND        ──────────────────┤ GND (pin 8)     │            │          │
                                 │                  │            │          │
                                 │ 1Y  (pin 3) ────╂── tw.pair ─┤ Pin 3 PULS+ │
                                 │ 1Z  (pin 4) ────╂── tw.pair ─┤ Pin 5 PULS- │
                                 │ 2Y  (pin 6) ────╂── tw.pair ─┤ Pin 4 DIR+  │
                                 │ 2Z  (pin 7) ────╂── tw.pair ─┤ Pin 14 DIR- │
                                 │                  │            │ Pin 13 GND  │
                                 │    0.1µF VCC-GND │            │          │
                                 └─────────────────┘            └──────────┘
```

## AASD-15A Configuration

Before connecting, set these parameters on the AASD-15A front panel:

| Parameter | Value | Description |
|-----------|-------|-------------|
| Pn002 | 002 | Control mode: Position (step/dir) |
| Pn003 | 001 | Servo enable on power-up |
| Pn098 | 80 | Electronic gear numerator |
| Pn109 | 002 | Position command deceleration mode |
| Pn110 | 050 | Position command filter time constant |
| Pn111 | 050 | S-curve filter Ta |
| Pn112 | 050 | S-curve filter Ts |

> Pn002=002 selects differential step/dir input mode. Pn003=000 (pulse input) selects CW/CCW mode — do not use this.

## Test Procedure

### Phase 1A: GPIO Signal Validation (no line driver, no motor)

**Status**: ⬜ Pending logic analyzer arrival

Flash firmware, probe GPIO4 (STEP) and GPIO10 (DIR) directly.

```bash
# Validate firmware-reported count first
python tools/sigtest.py --port COM7 --motor 0 --steps 1000 --rate 250000 --dir 1
```

Then capture with logic analyzer and validate the CSV:
```bash
python tools/sigtest.py --motor 0 --steps 1000 --rate 250000 --csv capture.csv --no-send
```

For continuous frequency sweep:
```bash
# 1 second of pulses at each frequency
# Send via serial console or mstat_check.py:
# FREQTEST:0:50000:1000
# FREQTEST:0:125000:1000
# FREQTEST:0:250000:1000
```

**Signal targets:**

| Metric | Target | AASD-15A spec |
|--------|--------|---------------|
| Pulse width | 2µs | ≥1.5µs |
| DIR setup time | ≥4µs before first step | ≥2µs |
| Step count accuracy | commanded == hw_counted | — |
| Max sustained rate | 250 kHz | — |
| Jitter | <1µs pk-pk | — |

**Pass criteria:**
- [ ] `SIGTEST:DONE ... PASS` (hw_counted == commanded, pos_error == 0)
- [ ] Pulse width 2µs ± 0.5µs confirmed on logic analyzer
- [ ] DIR stable ≥4µs before first step after direction change
- [ ] 250 kHz sustained without WDT panic or reboot

---

### Phase 1B: SN75174N Differential Output (no motor)

**Status**: ⬜ Pending breadboard build

Wire SN75174N per wiring diagram above. Probe differential pairs at SN75174N output pins (before DB25 connector).

**Pass criteria:**
- [ ] PULS+/PULS− differential swing ≥2V (RS-422 minimum, typical 5V with +5V supply)
- [ ] DIR+/DIR− complementary, stable
- [ ] Clean rising/falling edges — no ringing on ≤30cm breadboard wires
- [ ] EN1 pin tied HIGH, both drivers enabled

---

### Phase 1C: Single Motor Spin Test

**Status**: ⬜ Pending hardware

Wire DB25 to SN75174N. Power up AASD-15A with motor connected. Follow all AASD-15A safety procedures before energising.

1. Verify AASD-15A shows no fault (display shows `rdy` or similar)
2. Send small position target from desktop app HIL entity (e.g. 100 steps)
3. Verify motor rotates smoothly, returns to zero
4. Test direction reversal — motor should reverse cleanly
5. Use `MTEST:0` for automated 200-step forward/back self-test:
   ```
   # Send from desktop app console or serial terminal:
   MTEST:0
   ```
   Expected: `MTEST:DONE M0 error=0 PASS`
6. Rate sweep using `FREQTEST`:
   - `FREQTEST:0:50000:500` — 50 kHz for 500ms
   - `FREQTEST:0:125000:500` — 125 kHz for 500ms
   - `FREQTEST:0:250000:500` — 250 kHz for 500ms

**Pass criteria:**
- [ ] Motor rotates in correct direction for positive/negative targets
- [ ] `MTEST:0` reports `error=0 PASS`
- [ ] Motor runs smoothly at 50 kHz, 125 kHz, 250 kHz
- [ ] No AASD-15A fault codes during operation
- [ ] No phantom motion when target = current position

---

### Phase 1D: 10-Minute Soak Test

**Status**: ⬜ Pending

Run continuous back-and-forth motion at operating speed for 10 minutes.

```bash
# From desktop app: set test signal, 0.5 Hz sine, 20% amplitude on one axis
# Monitor with mstat_check.py — pos should track tgt continuously
python tools/mstat_check.py
```

**Pass criteria:**
- [ ] No position drift (pos == tgt after each half-cycle)
- [ ] No AASD-15A faults
- [ ] `loop_us(avg)` stays below 20µs throughout
- [ ] No firmware panics or reboots

---

## Validation Metrics Summary

| Metric | Target | Method | Status |
|--------|--------|--------|--------|
| Step pulse width | 2µs ± 0.5µs | Logic analyzer + sigtest.py | ⬜ Pending |
| Max step rate | ≥250 kHz | FREQTEST + logic analyzer | ⬜ Pending |
| Step count accuracy | 0 error | SIGTEST hw_counted == commanded | ⬜ Pending |
| DIR setup time | ≥4µs | Logic analyzer CSV analysis | ⬜ Pending |
| Timing jitter | <1µs pk-pk | Logic analyzer | ⬜ Pending |
| Differential swing | ≥2V | Oscilloscope at SN75174N output | ⬜ Pending |
| Motor tracking | pos_error == 0 | MTEST + MSTAT | ⬜ Pending |
| Soak (10 min) | No drift, no faults | Visual + MSTAT | ⬜ Pending |

## Phase Completion Criteria

- [ ] Phase 1A: GPIO signals validated by logic analyzer
- [ ] Phase 1B: SN75174N differential output clean on scope
- [ ] Phase 1C: Single motor spins, MTEST passes, 250 kHz confirmed
- [ ] Phase 1D: 10-minute soak with no drift or faults

## Next Steps (after single motor passes)

1. Expand to all 6 motors simultaneously (`RATETEST` command)
2. Run `SIGTEST` on all 6 motors — validate PCNT (M0–3) vs RMT (M4–5) counting accuracy
3. Full 6DOF platform integration with real motion profiles from desktop app
4. Homing sequence validation (AASD-15A Pn033–Pn039)
