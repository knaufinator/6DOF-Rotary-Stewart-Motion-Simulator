# Step/Dir Signal Analyzer — Test Harness

Goal: Validate the controller's MCPWM step/dir output in two stages — first with a simple ESP-to-ESP direct-wire test to prove the firmware logic, then with a proper SN75175N receiver daughter board to validate the full differential signal chain.

---

# Stage 1 — ESP-to-ESP Direct Logic Test

**Purpose**: Validate the controller ESP32-S3's MCPWM step/dir output using a second ESP32-S3 connected directly via jumper wires. No line drivers, no differential, no extra ICs. Pure GPIO-to-GPIO.

**What this proves**:
- IK → step/dir conversion is correct
- MCPWM one-shot pulses fire at expected rate and width
- Direction changes occur with proper setup timing
- Burst stepping produces correct total step counts
- No ghost pulses when idle
- Per-motor step counts match controller's `MSTAT`

## Stage 1 — Architecture

```
Controller ESP32-S3                          Analyzer ESP32-S3
┌────────────────────┐     jumper wires     ┌─────────────────────┐
│ GPIO4  (STEP_0) ───┼─────────────────────►│ GPIO4  (STEP_IN_0)  │
│ GPIO10 (DIR_0)  ───┼─────────────────────►│ GPIO10 (DIR_IN_0)   │
│ GPIO5  (STEP_1) ───┼─────────────────────►│ GPIO5  (STEP_IN_1)  │
│ GPIO11 (DIR_1)  ───┼─────────────────────►│ GPIO11 (DIR_IN_1)   │
│ GPIO6  (STEP_2) ───┼─────────────────────►│ GPIO6  (STEP_IN_2)  │
│ GPIO12 (DIR_2)  ───┼─────────────────────►│ GPIO12 (DIR_IN_2)   │
│ GPIO7  (STEP_3) ───┼─────────────────────►│ GPIO7  (STEP_IN_3)  │
│ GPIO13 (DIR_3)  ───┼─────────────────────►│ GPIO13 (DIR_IN_3)   │
│ GPIO8  (STEP_4) ───┼─────────────────────►│ GPIO8  (STEP_IN_4)  │
│ GPIO14 (DIR_4)  ───┼─────────────────────►│ GPIO14 (DIR_IN_4)   │
│ GPIO9  (STEP_5) ───┼─────────────────────►│ GPIO9  (STEP_IN_5)  │
│ GPIO17 (DIR_5)  ───┼─────────────────────►│ GPIO17 (DIR_IN_5)   │
│            GND  ───┼─────────────────────►│ GND                 │
└────────────────────┘                      └─────────────────────┘
        USB ↕ (COM3)                              USB ↕ (COM_ANALYZER)
        PC / Desktop App                          PC serial monitor
```

> All pins are 1:1 between boards. Both boards must be N8 (no PSRAM) — GPIO17 is free on N8.

## Stage 1 — Bill of Materials

| # | Part | Qty | Notes |
|---|------|-----|-------|
| 1 | **ESP32-S3-DevKitC-1** | 1 | Analyzer board (you already have the controller board) |
| 2 | **Jumper wires** (M-M) | 13 | 6 STEP + 6 DIR + 1 GND |
| 3 | **USB cable** | 1 | For analyzer board serial |

That's it. No ICs, no breadboard, no power supply. Just two ESP32-S3 boards and wires.

## Stage 1 — Wiring

### Pin-to-Pin Map

| Controller GPIO | Signal | Wire Color (suggested) | Analyzer GPIO |
|----------------|--------|----------------------|---------------|
| GPIO4 | STEP_0 | white | GPIO4 |
| GPIO5 | STEP_1 | white | GPIO5 |
| GPIO6 | STEP_2 | white | GPIO6 |
| GPIO7 | STEP_3 | white | GPIO7 |
| GPIO8 | STEP_4 | white | GPIO8 |
| GPIO9 | STEP_5 | white | GPIO9 |
| GPIO10 | DIR_0 | yellow | GPIO10 |
| GPIO11 | DIR_1 | yellow | GPIO11 |
| GPIO12 | DIR_2 | yellow | GPIO12 |
| GPIO13 | DIR_3 | yellow | GPIO13 |
| GPIO14 | DIR_4 | yellow | GPIO14 |
| GPIO17 | DIR_5 | yellow | GPIO17 |
| GND | — | black | GND |

> Both boards powered via their own USB cables. The shared GND wire is **required** for proper logic level reference.

### Wiring Tips

- Keep wires short (<15 cm) to minimize crosstalk and ringing at high step rates
- Route STEP wires away from DIR wires where possible
- For single-motor testing, you only need 3 wires: STEP_0, DIR_0, GND

## Stage 1 — Test Procedure

### Test 1.1: Single Motor Smoke Test

1. Flash controller firmware to ESP32-S3 #1 (COM3)
2. Flash analyzer firmware to ESP32-S3 #2 (COM_ANALYZER)
3. Wire 3 jumpers: STEP_0 (GPIO4→GPIO4), DIR_0 (GPIO10→GPIO10), GND
4. Open serial monitor to analyzer, send `STREAM:1`
5. Send position command to controller: `127,127,200,127,127,127X`
6. **Verify**: Analyzer reports steps on motor 0, direction = 1 (forward)
7. Send `127,127,50,127,127,127X`
8. **Verify**: Direction change reported, steps accumulate in reverse
9. Send `127,127,127,127,127,127X` (center/home)
10. **Verify**: Motor returns to ~0 position, stepping stops

### Test 1.2: Step Count Cross-Validation

1. Send `RESET` to analyzer
2. Send `MTEST:RESETX` to controller (clear controller stats)
3. Send a known position: `127,127,255,127,127,127X` (max heave)
4. Wait 3 seconds for all stepping to complete
5. Send `MSTATX` to controller — note motor 0 `totalSteps`
6. Send `STATUS` to analyzer — note motor 0 `steps`
7. **Pass**: Both counts are identical

### Test 1.3: Timing Analysis

1. Send `RESET` to analyzer, then `MONITOR:0`
2. Send a large move to controller
3. Send `STATUS` to analyzer after move completes
4. **Verify**:
   - `min_us` ≥ 4 (minStepInterval = 4µs configured in controller)
   - `max_us` is reasonable (first step after idle may be large — that's OK)
   - `avg_us` is consistent with expected step rate

### Test 1.4: All 6 Motors Simultaneous

1. Wire all 13 jumpers (6 STEP + 6 DIR + GND)
2. Send `RESET` to analyzer, `MTEST:RESETX` to controller
3. Send a multi-axis command: `200,200,200,200,200,200X`
4. Wait for motion to complete
5. Send `SUMMARY` to analyzer, `MSTATX` to controller
6. **Verify**: All 6 motor step counts match between controller and analyzer

### Test 1.5: Ghost Pulse / Idle Soak Test

1. Wire all 13 jumpers
2. Send `RESET` to analyzer, then `STREAM:1`
3. Send center position to controller: `127,127,127,127,127,127X`
4. Wait 60 seconds — do NOT send any more commands
5. Send `SUMMARY` to analyzer
6. **Pass**: All motors show `steps = 0` (zero ghost pulses)

### Test 1.6: Burst Stepping Stress Test

1. Send `RESET` to analyzer
2. Rapidly alternate between two positions (1 Hz):
   ```
   127,127,255,127,127,127X  (max heave)
   127,127,0,127,127,127X    (min heave)
   ```
3. Repeat 10 times, then let motion complete
4. Send `SUMMARY` to analyzer
5. **Verify**: `dir_chg` count = ~20 (10 round trips), step counts match controller

## Stage 1 — Validation Metrics

| Metric | Pass Criteria | Method |
|--------|---------------|--------|
| **Step count accuracy** | Analyzer count = Controller `MSTAT` count | Test 1.2 |
| **Position tracking** | Net position returns to ~0 after round trip | Test 1.1 |
| **Timing: min interval** | ≥ 4µs (controller minStepInterval) | Test 1.3 |
| **Timing: jitter** | < 3µs variation at steady state | Test 1.3 `max_us - min_us` |
| **Direction setup** | DIR changes before next STEP pulse | Test 1.1 `dir_chg` count |
| **6-motor simultaneous** | All 6 counts match | Test 1.4 |
| **Idle noise floor** | Zero steps in 60s idle | Test 1.5 |
| **Burst reliability** | Zero missed steps over 10 round trips | Test 1.6 |

## Stage 1 — Completion Criteria

- [ ] Single motor step counts match between controller and analyzer
- [ ] All 6 motors produce correct, independent step counts
- [ ] Timing stats show ≥ 4µs minimum interval
- [ ] Zero ghost pulses in 60-second soak test
- [ ] Burst stepping produces correct total counts
- [ ] Direction changes tracked correctly

> **Once Stage 1 passes**: The controller firmware, IK, and MCPWM are validated. Any issues found in Stage 2 are isolated to the differential signal chain (SN75174N → cable → SN75175N).

---

# Stage 2 — Full PCB Test with SN75175N Receivers

**Purpose**: Validate the complete signal chain from controller through SN75174N line drivers, twisted-pair cable, and SN75175N differential receivers to the analyzer ESP32-S3. This emulates the AASD-15A servo driver input path.

**What this proves** (beyond Stage 1):
- SN75174N differential output is clean and within RS-422 spec
- Cable propagation and impedance are acceptable
- SN75175N correctly recovers the single-ended logic
- Voltage dividers produce safe 3.0V levels for ESP32-S3
- No signal integrity issues at full step rates (up to 250 kHz)

## Stage 2 — Architecture

```
Controller ESP32-S3 ──► SN75174N (driver) ──► twisted pair ──► SN75175N (receiver)
                                                                    │
                                                            voltage dividers
                                                              (5V → 3.0V)
                                                                    │
                                                            Analyzer ESP32-S3
                                                                    │
                                                              USB Serial ──► PC
```

## Stage 2 — Bill of Materials

| # | Part | Description | Qty | Notes |
|---|------|-------------|-----|-------|
| 1 | **SN75175N** (DIP-16) | TI quad RS-422 differential receiver | 3 | Companion to SN75174N. [Datasheet](https://www.ti.com/lit/ds/symlink/sn75175.pdf) |
| 2 | **DIP-16 socket** | IC socket for SN75175N | 3 | Same as SN75174N sockets |
| 3 | **0.1µF ceramic cap** | Bypass cap for VCC | 3 | 1 per IC |
| 4 | **1kΩ resistor** (1/4W) | Voltage divider top leg | 12 | Standard through-hole |
| 5 | **1.5kΩ resistor** (1/4W) | Voltage divider bottom leg | 12 | Standard through-hole |
| 6 | **5V power supply** | USB breakout or bench supply | 1 | Powers SN75175N |
| 7 | **Perfboard or breadboard** | Mounts ICs + resistors | 1 | Half-size breadboard works |
| 8 | **DB25 female connector** | Mates with cable from SN75174N driver board | 1–6 | 1 for single motor, 6 for all |
| 9 | **Jumper wires** | M-M for breadboard connections | — | ESP32 ← daughter board |

> For single-motor testing, you only need 1× SN75175N, 1× socket, 1× cap, 2× 1kΩ, 2× 1.5kΩ.

### DigiKey Order (6-motor full build)

| DigiKey P/N | Part | Qty |
|-------------|------|-----|
| 296-1736-5-ND | SN75175N (DIP-16) quad RS-422 receiver | 3 |
| (A 16-LC-TT) | DIP-16 IC socket (search "CONN IC DIP SOCKET 16POS TIN") | 3 |
| 399-4151-ND | 0.1µF ceramic cap (KEMET C315C104M5U5TA) | 3 |
| CF14JT1K00CT-ND | 1kΩ 1/4W carbon film resistor | 12 |
| CF14JT1K50CT-ND | 1.5kΩ 1/4W carbon film resistor | 12 |

## Stage 2 — SN75175N Pinout

```
              SN75175N (DIP-16) — Quad RS-422 Receiver
              ┌───────────┐
    1A (in+) ─┤ 1      16 ├─ VCC (+5V)
    1B (in-) ─┤ 2      15 ├─ G̅2 (enable, active LOW)
    1Y (out) ─┤ 3      14 ├─ 4B (in-)
    2A (in+) ─┤ 4      13 ├─ 4A (in+)
    2B (in-) ─┤ 5      12 ├─ 4Y (out)
    2Y (out) ─┤ 6      11 ├─ 3B (in-)
 G̅1 (enable)─┤ 7      10 ├─ 3A (in+)
       GND   ─┤ 8       9 ├─ 3Y (out)
              └───────────┘

    G̅1 (pin 7)  = enables receivers 1 & 2 (active LOW → tie to GND)
    G̅2 (pin 15) = enables receivers 3 & 4 (active LOW → tie to GND)
    VCC (pin 16) = +5V
    GND (pin 8)  = ground
```

> **Important**: SN75175N has **active-LOW** enables (opposite of SN75174N which is active-HIGH). Tie both enable pins to GND.

## Stage 2 — Voltage Divider (5V → 3.3V)

The SN75175N is powered at 5V, so its outputs swing to ~5V HIGH. ESP32-S3 GPIOs are **not** 5V tolerant (max 3.6V). Each receiver output needs a voltage divider:

```
SN75175N output ──┬── 1kΩ ──┬── ESP32-S3 GPIO input
                  │         │
                  │       1.5kΩ
                  │         │
                  └─────────┴── GND

Vout = 5V × 1.5kΩ / (1kΩ + 1.5kΩ) = 3.0V  ✓ (well within ESP32 VIH = 2.48V)
Output impedance = 600Ω → rise time ~6ns with 10pF GPIO capacitance
```

## Stage 2 — IC Allocation (6 Motors)

| IC | Channels | Motor STEP | Motor DIR |
|----|----------|------------|-----------|
| **SN75175N #1** | Ch1 + Ch2 + Ch3 + Ch4 | Motor 0 STEP, Motor 1 STEP | Motor 0 DIR, Motor 1 DIR |
| **SN75175N #2** | Ch1 + Ch2 + Ch3 + Ch4 | Motor 2 STEP, Motor 3 STEP | Motor 2 DIR, Motor 3 DIR |
| **SN75175N #3** | Ch1 + Ch2 + Ch3 + Ch4 | Motor 4 STEP, Motor 5 STEP | Motor 4 DIR, Motor 5 DIR |

### Analyzer ESP32-S3 GPIO Pin Map (same as Stage 1)

| Motor | STEP GPIO | DIR GPIO |
|-------|-----------|----------|
| 0 | GPIO4 | GPIO10 |
| 1 | GPIO5 | GPIO11 |
| 2 | GPIO6 | GPIO12 |
| 3 | GPIO7 | GPIO13 |
| 4 | GPIO8 | GPIO14 |
| 5 | GPIO9 | GPIO17 |

## Stage 2 — Detailed Wiring (IC #1, Motors 0 & 1)

### Differential inputs from SN75174N driver board (or DB25 cable)

| SN75175N #1 Pin | Source | Via Divider → | Analyzer GPIO |
|-----------------|--------|---------------|---------------|
| 1 (1A) | Motor 0 PULS+ (SN75174N 1Y / DB25 pin 3) | — | — |
| 2 (1B) | Motor 0 PULS- (SN75174N 1Z / DB25 pin 5) | — | — |
| 3 (1Y) | — | 1kΩ/1.5kΩ | GPIO4 (STEP_IN_0) |
| 4 (2A) | Motor 0 DIR+ (SN75174N 2Y / DB25 pin 4) | — | — |
| 5 (2B) | Motor 0 DIR- (SN75174N 2Z / DB25 pin 14) | — | — |
| 6 (2Y) | — | 1kΩ/1.5kΩ | GPIO10 (DIR_IN_0) |
| 7 (G̅1) | GND | — | — |
| 8 (GND) | GND | — | — |
| 9 (3Y) | — | 1kΩ/1.5kΩ | GPIO5 (STEP_IN_1) |
| 10 (3A) | Motor 1 PULS+ | — | — |
| 11 (3B) | Motor 1 PULS- | — | — |
| 12 (4Y) | — | 1kΩ/1.5kΩ | GPIO11 (DIR_IN_1) |
| 13 (4A) | Motor 1 DIR+ | — | — |
| 14 (4B) | Motor 1 DIR- | — | — |
| 15 (G̅2) | GND | — | — |
| 16 (VCC) | +5V | — | — |

> ICs #2 and #3 follow the same pattern for Motors 2–5.

## Stage 2 — Wiring Diagram (Single Motor)

```
SN75174N Driver Board              SN75175N Daughter Board             Analyzer ESP32-S3
(controller side)                  (test harness)                      (DevKitC-1)
┌─────────────────┐               ┌─────────────────────┐            ┌──────────────┐
│ 1Y (pin 3) PULS+├──── tw.pair ──┤ 1A (pin 1)          │            │              │
│ 1Z (pin 4) PULS-├──── tw.pair ──┤ 1B (pin 2)          │            │              │
│                  │               │ 1Y (pin 3)──┬─1kΩ─┬─╂── 3.0V ──►│ GPIO4 STEP_IN│
│                  │               │             │     │  │            │              │
│                  │               │             │   1.5kΩ│            │              │
│                  │               │             │     │  │            │              │
│                  │               │             └──GND┘  │            │              │
│                  │               │                      │            │              │
│ 2Y (pin 6) DIR+ ├──── tw.pair ──┤ 2A (pin 4)          │            │              │
│ 2Z (pin 7) DIR- ├──── tw.pair ──┤ 2B (pin 5)          │            │              │
│                  │               │ 2Y (pin 6)──┬─1kΩ─┬─╂── 3.0V ──►│ GPIO10 DIR_IN│
│                  │               │             │     │  │            │              │
│                  │               │             │   1.5kΩ│            │              │
│                  │               │             │     │  │            │              │
│                  │               │             └──GND┘  │            │              │
│                  │               │                      │            │              │
│                  │               │  G̅1 (pin 7)──GND    │            │              │
│                  │               │  G̅2 (pin 15)──GND   │            │              │
│                  │               │  VCC (pin 16)──+5V   │            │              │
│                  │               │  GND (pin 8)──GND    │            │              │
│                  │               │  0.1µF VCC-GND       │            │         GND ─┤
└─────────────────┘               └─────────────────────┘            └──────────────┘
                                           │                                  │
                                      Common GND ◄────────────────────────────┘
```

## Stage 2 — Test Procedure

Re-run the same tests from Stage 1 (1.1 through 1.6) through the differential signal chain. The analyzer firmware is identical — same pin map, same commands.

### Stage 2 Additional Tests

**Test 2.1: Signal Integrity Comparison**
1. Record timing stats from Stage 1 (`min_us`, `max_us`, `avg_us`)
2. Run same motions through full differential chain
3. Compare: `min_us` should still be ≥ 4µs, jitter should not increase significantly
4. Any significant degradation points to cable or driver/receiver issues

**Test 2.2: Cable Length Test**
1. Test with short cable (~30cm) — baseline
2. Extend to 2m twisted pair
3. Compare timing stats — should be minimal difference for RS-422

**Test 2.3: Step Rate Sweep**
1. Vary controller step rate: 10kHz, 50kHz, 100kHz, 200kHz
2. At each rate, verify analyzer count matches controller count
3. Look for missed steps at high rates (signal integrity limit)

## Stage 2 — Completion Criteria

- [ ] All Stage 1 tests pass through differential chain
- [ ] Step counts match at 200 kHz sustained rate
- [ ] No additional jitter introduced by line driver/receiver
- [ ] Cable lengths up to 2m produce no errors
- [ ] Voltage divider outputs measured at ~3.0V (multimeter check)

---

## Firmware

The analyzer firmware is in `test_harness/` at the project root. It is a standalone ESP-IDF project for a second ESP32-S3-DevKitC. **The same firmware is used for both Stage 1 and Stage 2** — the pin mapping and analysis logic are identical regardless of whether signals come from direct wires or through differential receivers.

### Features

- **GPIO interrupt** on STEP rising edge for all 6 channels
- **Per-motor tracking**: net position, total steps, step rate, min/max/avg step interval, direction changes
- **Rate calculation**: windowed step count per reporting interval
- **Idle detection**: reports when no pulses received for configurable timeout
- **JSON output** over USB serial for tool integration
- **ASCII command interface**: `STATUS`, `SUMMARY`, `RESET`, `STREAM:1/0`, `MONITOR:N`

### Serial Commands

| Command | Response | Description |
|---------|----------|-------------|
| `STATUS` | JSON object with all motor stats | One-shot status dump |
| `SUMMARY` | Human-readable table | Quick visual check |
| `RESET` | `{"ok":"stats_reset"}` | Clear all counters |
| `STREAM:1` | `{"ok":"streaming_on"}` | Continuous JSON output at 2 Hz |
| `STREAM:0` | `{"ok":"streaming_off"}` | Stop streaming |
| `MONITOR:N` | `{"ok":"monitor_N"}` | Focus detailed timing on motor N |
| `HELP` | Command list | — |

### Build & Flash

```bash
cd test_harness
idf.py set-target esp32s3
idf.py build
idf.py -p COM_ANALYZER flash monitor
```

> Use a different COM port than the controller ESP32-S3.

---

## Future Enhancements

- **PCNT hardware counters** (ESP32-S3 has 4 units) for guaranteed-accurate counting at >200kHz
- **Desktop app integration**: add analyzer as a panel in the desktop app
- **Pulse width measurement**: use ANYEDGE interrupt to measure actual pulse width vs expected 2µs
- **Custom PCB**: combine SN75175N receivers + voltage dividers + ESP32-S3 module on a single board
- **Loopback mode**: analyzer sends results back to controller for automated self-test
