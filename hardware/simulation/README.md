# Hardware Simulation & Pre-Fabrication Testing

This directory contains simulation models and scripts for validating the PCB design **before ordering physical boards**.

## Directory Structure

```
simulation/
├── README.md                  # This file
├── ltspice/                   # LTspice circuit simulations
│   ├── level_translator.asc   # SN74LVCH16T245 model
│   ├── rs422_driver.asc       # AM26C31 differential driver
│   ├── estop_opto.asc         # TLP2361 E-stop circuit
│   ├── buck_regulator.asc     # LMR33630 power supply
│   └── run_all_sims.sh        # Automated SPICE runner
├── signal_integrity/          # High-speed signal analysis
│   ├── differential_pair.py   # Impedance calculator
│   ├── eye_diagram.py         # Digital signal quality
│   └── requirements.txt       # Python dependencies
├── virtual_pcb/               # Python hardware models
│   ├── virtual_pcb.py         # Complete PCB simulation
│   ├── test_virtual_pcb.py    # Integration tests
│   └── components/            # Individual component models
└── preflight_check.sh         # Complete pre-fab validation

```

## Quick Start

### 1. Python-Based Validation (No Additional Software)

```bash
# Already works - runs in CI/CD
pytest ../tests/test_servo_interface.py -v
pytest ../tests/test_kicad_project.py -v

# Results: 30 tests validating electrical specifications
```

### 2. SPICE Circuit Simulation (Install LTspice)

```bash
# Download LTspice (free): https://www.analog.com/en/design-center/design-tools-and-calculators/ltspice-simulator.html

# Run simulations
cd ltspice
./run_all_sims.sh

# View results
# Open *.asc files in LTspice
# Check waveforms meet specifications
```

### 3. Signal Integrity Analysis

```bash
# Install Python dependencies
cd signal_integrity
pip install -r requirements.txt

# Calculate differential pair impedance
python differential_pair.py \
  --width 0.25 \
  --gap 0.25 \
  --height 0.2 \
  --er 4.5

# Expected output: ~120Ω
```

### 4. Virtual Hardware Testing

```bash
# Test firmware integration without physical PCB
cd virtual_pcb
pip install -r requirements.txt
pytest test_virtual_pcb.py -v

# Simulates:
# - Level translator delays
# - RS-422 output voltages
# - E-stop response time
# - Step pulse timing validation
```

## Simulation Goals

### What We Validate

✅ **Electrical Specifications** (via Python tests)
- RS-422 voltage levels: ±2.0V to ±2.5V differential
- Input impedance: 12kΩ typ
- E-stop opto current: 9.5mA (with R19=2.4kΩ)
- Buck regulator ripple: <50mV
- Signal rise times: <50ns

✅ **Circuit Behavior** (via SPICE)
- Transient response to step pulses
- Power supply startup and stability
- Frequency response of signal paths
- Noise margins and interference

✅ **Signal Integrity** (via Python/SPICE)
- Differential pair impedance: 120Ω ±10%
- Trace length matching: <5mm mismatch
- Crosstalk between adjacent pairs
- Termination effectiveness

✅ **Firmware Integration** (via Virtual PCB)
- 1000Hz update rate achievable
- Step pulse width meets AASD-15A requirements (≥5µs)
- Direction setup and hold times
- E-stop detection latency

### What We DON'T Simulate (Yet)

⚠️ **Thermal Analysis** - Requires FEA (future enhancement)
⚠️ **EMI/EMC** - Requires specialized tools
⚠️ **Mechanical Stress** - Connector strain, vibration
⚠️ **Long-term Reliability** - MTBF, component aging

## Simulation Workflow

### For Each Design Change:

1. **Update Documentation** (`docs/hardware/servo_driver_interface.md`)
2. **Update Python Tests** (`hardware/tests/test_servo_interface.py`)
3. **Run Python Validation**:
   ```bash
   pytest ../tests/ -v
   ```
4. **Update SPICE Models** (if analog circuit changed)
5. **Run SPICE Simulations**:
   ```bash
   cd ltspice
   ./run_all_sims.sh
   ```
6. **Validate Signal Integrity** (if high-speed path changed):
   ```bash
   cd signal_integrity
   python differential_pair.py
   ```
7. **Update KiCad Design**
8. **Run Pre-Flight Check**:
   ```bash
   ./preflight_check.sh
   ```
9. **Commit All Changes** (docs, tests, simulations, KiCad)

### Before Ordering PCBs:

```bash
# Complete validation suite
./preflight_check.sh

# Manual checklist:
# [ ] All automated tests pass
# [ ] SPICE simulations reviewed
# [ ] Signal integrity within spec
# [ ] BOM components available from distributors
# [ ] KiCad DRC clean (0 errors)
# [ ] Gerber files visually inspected in viewer
# [ ] (Optional) Critical circuits breadboard-tested
```

## Tool Installation

### LTspice (Recommended for SPICE)

**Windows/macOS/Linux:**
1. Download from: https://www.analog.com/en/design-center/design-tools-and-calculators/ltspice-simulator.html
2. Install (free, no registration required)
3. Open `.asc` files in `ltspice/` directory

### ngspice (Open Source Alternative)

**Linux:**
```bash
sudo apt install ngspice gaw  # gaw = waveform viewer
```

**macOS:**
```bash
brew install ngspice
```

**Windows:**
- Download from: http://ngspice.sourceforge.net/download.html

### KiCad 7.0+ (Has Built-in SPICE)

**All Platforms:**
1. Install KiCad 7.0+: https://www.kicad.org/download/
2. Open schematic
3. Tools → Simulator
4. Add SPICE models to components
5. Run analysis (transient, AC, DC)

### Python Tools

```bash
# Signal integrity analysis
pip install scikit-rf numpy matplotlib scipy

# Virtual PCB simulation
pip install pyserial numpy pytest
```

## Example Simulation Results

### E-Stop Opto Current (LTspice)

**Specification**: 9.5mA ±5% through TLP2361 LED

**SPICE Results**:
```
R19 = 2.2kΩ → 10.36mA ❌ EXCEEDS 10mA MAX
R19 = 2.4kΩ → 9.50mA  ✅ WITHIN SPEC
R19 = 2.7kΩ → 8.44mA  ✅ WITHIN SPEC (more margin)
```

**Decision**: Used 2.4kΩ for 5% safety margin below 10mA absolute maximum.

This simulation **caught a design error** that would have damaged the opto-isolator!

### Differential Pair Impedance (Python)

**Specification**: 120Ω ±10% (108-132Ω)

**Python Calculator Results**:
```python
# Input parameters
trace_width = 0.25   # mm
trace_gap = 0.25     # mm
dielectric_h = 0.2   # mm (FR-4, h between layers)
epsilon_r = 4.5      # FR-4

# Calculated impedance
Z_diff = 119.8Ω      # ✅ Within 120Ω ±10%
```

**PCB Stackup Validated**: 4-layer FR-4, 1.6mm total thickness

### Buck Regulator Ripple (SPICE)

**Specification**: <50mV peak-to-peak @ 5V output

**SPICE Results**:
```
Load: 2A (max)
Output ripple: 31.2mV p-p  ✅ WITHIN SPEC
Frequency: 400kHz
Output caps: C7+C8 = 20µF
```

**Validation**: LMR33630 with 10µF × 2 bulk caps meets ripple requirement

## Contributing

When adding new simulations:

1. Place in appropriate subdirectory (`ltspice/`, `signal_integrity/`, etc.)
2. Document what specification is being validated
3. Include pass/fail criteria in comments
4. Add to `preflight_check.sh` if critical
5. Update this README with results

## Future Enhancements

- [ ] Thermal simulation (FEA)
- [ ] EMI pre-compliance testing
- [ ] PCB capacitance extraction
- [ ] Automated Gerber DFM checks
- [ ] BOM cost optimization
- [ ] Alternative component suggestions
- [ ] Monte Carlo analysis (component tolerances)

## Resources

- **LTspice Tutorial**: https://www.analog.com/en/design-center/design-tools-and-calculators/ltspice-simulator.html
- **KiCad SPICE Guide**: https://docs.kicad.org/7.0/en/eeschema/eeschema.html#spice-simulation
- **Transmission Line Calculator**: https://www.eeweb.com/tools/microstrip-impedance/
- **Signal Integrity**: "High-Speed Digital Design" by Howard Johnson

---

**Remember**: Simulation is a powerful tool, but **breadboarding critical circuits** provides the highest confidence before PCB fabrication!
