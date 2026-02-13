# Single Motor RMT Test Plan — Phase 1

Goal: Validate the ESP32-S3 RMT step/dir output driving a single AASD-15A servo via differential signaling, using off-the-shelf dev boards and minimal soldering.

## Signal Chain

```
ESP32-S3-DevKitC (3.3V RMT GPIO) → SN75174N line driver → twisted pair → AASD-15A CN2 DB25
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
| Pn002 | 0 | Control mode: Position (pulse/direction) |
| Pn003 | 0 | Pulse input mode: PULS+DIR differential |
| Pn004 | Set as needed | Electronic gear numerator |
| Pn005 | Set as needed | Electronic gear denominator |
| Pn006 | 0 | Pulse direction: normal (change to 1 to invert) |

> **Important**: Pn003=0 selects differential pulse input mode. If you see Pn003=1 (CW/CCW mode) or Pn003=2 (A/B quadrature), change it to 0.

## Test Procedure

### Phase 1A: Verify RMT Output (no motor)

1. Flash the existing firmware to the ESP32-S3-DevKitC
2. Connect an oscilloscope or logic analyzer to GPIO4 (STEP) and GPIO10 (DIR)
3. Send a position command via serial (e.g., `127,127,200,127,127,127`)
4. Verify clean step pulses: 2µs pulse width, expected frequency based on position delta
5. Confirm 100µs GPTimer cadence from `handleStepDirection()`

### Phase 1B: Verify Differential Output (no motor)

1. Wire the SN75174N on the breadboard as described above
2. Probe SN75174N outputs (pins 3/4 for PULS, pins 6/7 for DIR)
3. Verify differential output: Y and Z should be complementary, ~5V swing
4. Check signal integrity: clean edges, no ringing on short (<30cm) breadboard wires

### Phase 1C: Single Motor Spin Test

1. Wire DB25 connector to SN75174N outputs
2. Power up AASD-15A with motor connected (follow AASD safety procedures)
3. Verify AASD-15A parameter Pn003=0 (differential pulse mode)
4. Send small position commands from serial terminal
5. Motor should rotate smoothly in response to step commands
6. Test direction reversal
7. Test increasing step rates up to 50kHz, then 100kHz, then 200kHz

### Phase 1D: Validation Metrics

| Metric | Target | Method |
|--------|--------|--------|
| Step pulse integrity | Clean 2µs pulses at SN75174N output | Oscilloscope |
| Max sustained step rate | ≥ 200 kHz | Oscilloscope frequency counter |
| Direction setup time | ≥ 5µs between DIR change and first STEP | Oscilloscope |
| Timing jitter | < 1µs pk-pk at 10kHz command rate | Logic analyzer |
| Motor tracking | Position matches command within 1 step | AASD-15A position display |
| No phantom steps | Zero drift with motor idle, wires connected | 10-minute soak test |

## Phase Completion Criteria

- [ ] RMT generates correct step/dir pulses on GPIO
- [ ] SN75174N produces clean differential output
- [ ] AASD-15A accepts differential pulses and moves motor
- [ ] Sustained 200kHz step rate verified
- [ ] No phantom steps or noise-induced drift
- [ ] E-stop halts motor within 5ms

## Next Phase (after completion)

Once single-motor test passes:
1. Expand to 6-motor bench test (6× SN75174N channels — need two ICs since each has 4 drivers)
2. Design custom PCB with 2× SN75174N, DB25 connectors, power regulation
3. Add W5500 Ethernet module for UDP transport testing
4. Full 6DOF platform integration
