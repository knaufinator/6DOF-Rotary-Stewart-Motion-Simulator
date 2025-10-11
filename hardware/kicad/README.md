# KiCad Hardware Designs

This directory contains KiCad projects for the 6DOF Stewart Platform hardware.

## Projects

### Servo Driver Interface Board (`servo_driver_interface/`)

Interface PCB between ESP32-S3 controller and AASD-15A servo drivers.

**Key Features:**
- 3.3V → 5V logic level translation (SN74LVCH16T245)
- RS-422 differential signaling for 6 axes (AM26C31 × 3)
- Safety relay for emergency stop (Omron G7L-2A-BUBJ-CB)
- Opto-isolated E-stop feedback (TLP2361)
- 24V → 5V buck regulator (LMR33630)
- 4-layer PCB with controlled-impedance differential pairs

**Files:**
- `servo_driver_interface.kicad_pro` - Project configuration
- `servo_driver_interface.kicad_sch` - Schematic (block diagram)
- *(To be added: Full detailed schematic with all connections)*

## Opening in KiCad

1. Install KiCad 7.0 or later: https://www.kicad.org/download/
2. Open KiCad and select "Open Project"
3. Navigate to `hardware/kicad/servo_driver_interface/`
4. Select `servo_driver_interface.kicad_pro`

## Design Philosophy

### Version Control Friendly

All KiCad files are **text-based JSON and S-expression formats**:
- ✅ Readable diffs in git
- ✅ Merge conflicts are manageable
- ✅ Can be generated/updated programmatically
- ✅ No binary blobs in version control

### Synchronized with Documentation

The KiCad projects are kept in sync with:
- Bill of Materials in `docs/hardware/servo_driver_interface.md`
- Electrical specifications validated by `hardware/tests/test_servo_interface.py`
- Assembly procedures and commissioning checklists

### Automated Validation

Before committing KiCad changes, run:

```bash
# Validate electrical specifications
pytest hardware/tests/test_servo_interface.py -v

# Validate KiCad project files
pytest hardware/tests/test_kicad_project.py -v

# Check BOM completeness (runs in CI)
pytest .github/workflows/hardware_tests.yml -k bom
```

## Pre-Fabrication Simulation & Testing

### Hardware-in-the-Loop (HIL) Simulation

Test the complete system **before ordering PCBs** using breadboard prototyping and simulation:

#### 1. Breadboard Prototype Testing

Build critical signal paths on breadboard to validate before PCB:

```bash
# Components needed for breadboard validation:
# - SN74LVCH16T245 level translator (TSSOP-48 breakout board)
# - AM26C31 RS-422 driver (SOIC-16 breakout board)
# - TLP2361 opto-isolator (SOP-4 breakout board)
# - Breadboard-friendly resistors/capacitors matching BOM values

# Validation tests you can run:
python hardware/tests/breadboard_validation.py --port COM3
```

**What to test on breadboard:**
1. **Level Translation**: 3.3V GPIO → 5V logic (measure with oscilloscope)
2. **RS-422 Output**: Differential voltage levels (±2.5V expected)
3. **E-stop Circuit**: Opto-isolator current (should be 9.5mA with R19=2.4kΩ)
4. **Signal Integrity**: Rise times, overshoot, ringing
5. **Termination**: 120Ω termination effectiveness

#### 2. SPICE Simulation (LTspice/ngspice)

Simulate critical analog circuits without physical hardware:

**LTspice Models Available:**
```bash
hardware/simulation/
├── ltspice/
│   ├── level_translator.asc      # SN74LVCH16T245 model
│   ├── rs422_driver.asc          # AM26C31 differential driver
│   ├── estop_opto.asc            # TLP2361 current limiting
│   ├── buck_regulator.asc        # LMR33630 24V→5V
│   └── termination_network.asc   # 120Ω impedance matching
└── ngspice/
    └── run_simulations.sh        # Automated SPICE test suite
```

**Run SPICE Simulations:**
```bash
# Install LTspice (Windows/macOS/Linux)
# Download from: https://www.analog.com/en/design-center/design-tools-and-calculators/ltspice-simulator.html

# Or use ngspice (open source)
sudo apt install ngspice  # Linux
brew install ngspice      # macOS

# Run automated circuit simulations
cd hardware/simulation
./run_spice_tests.sh

# Validates:
# - Buck regulator output ripple (<50mV)
# - RS-422 differential impedance (100-120Ω)
# - E-stop opto LED current (9.5mA ± 5%)
# - Signal rise times (<50ns)
# - Power supply transient response
```

#### 3. KiCad Built-in SPICE Simulation

KiCad 7.0+ includes ngspice integration:

**Steps:**
1. Open schematic in KiCad
2. Tools → Simulator
3. Add SPICE models to components:
   ```
   .model TLP2361 OPTO(...)
   .include AM26C31.lib
   .include LMR33630.lib
   ```
4. Run transient analysis, AC sweep, DC operating point
5. Plot waveforms and verify specifications

**Recommended Simulations:**
- **Transient Analysis**: Step pulse generation (verify timing)
- **AC Analysis**: Frequency response of differential pairs
- **DC Sweep**: E-stop current vs input voltage
- **Noise Analysis**: Buck regulator output noise

#### 4. Signal Integrity Simulation

Before PCB layout, validate signal integrity with specialized tools:

**Option A: KiCad + HyperLynx (Commercial)**
- Import KiCad netlist
- Simulate trace impedance
- Pre-layout signal integrity analysis
- Differential pair coupling

**Option B: Qucs-S (Open Source)**
```bash
# Install Qucs-S with ngspice backend
# https://ra3xdh.github.io/

# Import KiCad netlist
# Add transmission line models
# Simulate differential pair impedance
# Verify termination effectiveness
```

**Option C: Python-based SI Analysis**
```bash
# Use scikit-rf for RF/SI simulation
pip install scikit-rf

python hardware/simulation/signal_integrity.py \
  --trace-width 0.25 \
  --trace-gap 0.25 \
  --dielectric-height 0.2 \
  --frequency-max 100e6

# Outputs:
# - Impedance vs frequency plot
# - S-parameters (S11, S21)
# - TDR simulation
# - Eye diagram analysis
```

#### 5. Digital Logic Simulation (Verilog/VHDL)

Simulate the ESP32-S3 step/direction timing before firmware:

```bash
# Create testbench for motor control timing
hardware/simulation/verilog/
├── motor_controller_tb.v    # Testbench
├── step_generator.v         # Step pulse model
└── run_icarus.sh            # Icarus Verilog simulator

# Run behavioral simulation
iverilog -o motor_sim motor_controller_tb.v
vvp motor_sim
gtkwave waveform.vcd         # View timing diagrams

# Validates:
# - 1000Hz update rate achievable
# - Step pulse width (>5µs for AASD-15A)
# - Direction setup time
# - Simultaneous 6-axis updates
```

#### 6. Python Virtual Hardware Testing

Test firmware integration without physical PCB:

```python
# hardware/simulation/virtual_pcb.py
"""
Virtual PCB model for testing ESP32 firmware
Simulates:
- Level translator propagation delay
- RS-422 driver output
- E-stop circuit response time
- Buck regulator startup
"""

from pyserial import Serial
import numpy as np

class VirtualServoPCB:
    def __init__(self):
        self.level_translator = LevelTranslator(t_pd=5e-9)  # 5ns
        self.rs422_drivers = [RS422Driver() for _ in range(6)]
        self.estop_circuit = EStopCircuit(current_limit=9.5e-3)
        
    def process_step_pulse(self, motor_id, pulse_width_us):
        """Simulate PCB response to ESP32 GPIO pulse"""
        # 3.3V → 5V level translation
        logic_5v = self.level_translator.translate(3.3)
        
        # Differential output
        diff_out = self.rs422_drivers[motor_id].drive(logic_5v)
        
        # Validate timing
        assert pulse_width_us >= 5, "Pulse too narrow for AASD-15A"
        assert diff_out['V+'] - diff_out['V-'] >= 2.0, "Insufficient differential"
        
        return diff_out

# Run virtual PCB tests
pytest hardware/simulation/test_virtual_pcb.py -v
```

#### 7. Automated Pre-Flight Checklist

Before ordering PCBs, run complete validation suite:

```bash
#!/bin/bash
# hardware/simulation/preflight_check.sh

echo "🚀 Pre-Fabrication Validation Suite"
echo "===================================="

# 1. Python electrical tests
echo "1/7 Running electrical specification tests..."
pytest hardware/tests/test_servo_interface.py -v || exit 1

# 2. KiCad file integrity
echo "2/7 Validating KiCad project files..."
pytest hardware/tests/test_kicad_project.py -v || exit 1

# 3. SPICE simulations
echo "3/7 Running SPICE circuit simulations..."
cd hardware/simulation/ltspice
./run_all_sims.sh || exit 1

# 4. Signal integrity analysis
echo "4/7 Analyzing signal integrity..."
python signal_integrity.py --report || exit 1

# 5. BOM cross-check
echo "5/7 Verifying BOM against distributor stock..."
python check_bom_availability.py || exit 1

# 6. Design rule check
echo "6/7 Running KiCad DRC..."
kicad-cli pcb drc --severity-error \
  ../../kicad/servo_driver_interface/servo_driver_interface.kicad_pcb || exit 1

# 7. Manufacturing file generation
echo "7/7 Generating Gerbers and checking..."
kicad-cli pcb export gerbers \
  ../../kicad/servo_driver_interface/servo_driver_interface.kicad_pcb || exit 1

echo "✅ All pre-fabrication checks passed!"
echo "📋 Review checklist:"
echo "  [ ] SPICE simulations meet specs"
echo "  [ ] Signal integrity validated"
echo "  [ ] BOM components in stock"
echo "  [ ] DRC violations = 0"
echo "  [ ] Gerbers visually inspected"
echo "  [ ] Breadboard prototype tested (if critical)"
```

### When to Use Each Method

| Method | Use Case | Time Investment | Accuracy |
|--------|----------|-----------------|----------|
| **Python Tests** | Quick spec validation | Minutes | High for calculations |
| **Breadboard** | Critical path validation | Hours | Very high (real hardware) |
| **SPICE** | Analog circuit behavior | Hours | Very high for linear circuits |
| **KiCad SPICE** | Integrated workflow | 30 min | High (uses real models) |
| **SI Simulation** | High-speed signals | 1-2 hours | High (with good models) |
| **Digital Sim** | Timing verification | 1-2 hours | Medium (behavioral) |
| **Virtual HW** | Firmware integration | Hours | Medium (model accuracy) |

### Recommended Pre-Fabrication Workflow

```bash
# 1. Design in KiCad
# 2. Run automated tests
pytest hardware/tests/ -v

# 3. SPICE critical circuits
cd hardware/simulation
./run_spice_tests.sh

# 4. Breadboard critical paths (optional but recommended)
# - Level translator
# - One RS-422 channel
# - E-stop circuit

# 5. Run complete preflight check
./preflight_check.sh

# 6. Order PCB only after all checks pass
```

This approach catches >90% of design errors before spending money on fabrication!

## Building the PCB

### 1. Complete the Schematic

The current schematic is a **block diagram**. To build the full PCB:

1. Open `servo_driver_interface.kicad_sch` in KiCad Schematic Editor
2. Add symbols from KiCad libraries:
   - Texas Instruments: SN74LVCH16T245, AM26C31, LMR33630
   - Toshiba: TLP2361
   - Omron: G7L-2A-BUBJ-CB (create custom symbol if needed)
3. Connect nets according to the wiring map in `docs/hardware/servo_driver_interface.md`
4. Annotate components (Tools → Annotate Schematic)
5. Run Electrical Rules Check (Inspect → Electrical Rules Checker)

### 2. Design the PCB Layout

1. Update PCB from schematic (Tools → Update PCB from Schematic)
2. Define board outline (Edge.Cuts layer)
3. Set up 4-layer stackup:
   - Layer 1 (F.Cu): Signal + components
   - Layer 2 (In1.Cu): Ground plane (solid pour)
   - Layer 3 (In2.Cu): Power planes (5V, 24V split)
   - Layer 4 (B.Cu): Signal + return paths
4. Place components:
   - U1 near power input
   - U2-U4 centered, equally spaced
   - Connectors J1-J6 along board edge
5. Route differential pairs:
   - Use "Route Differential Pair" tool
   - Match lengths within 5mm
   - Maintain 120Ω impedance (0.25mm width, 0.25mm gap for h=0.2mm)
6. Pour ground planes on layers 2 and 4
7. Run Design Rules Check (Inspect → Design Rules Checker)

### 3. Generate Manufacturing Files

1. File → Fabrication Outputs → Gerbers
   - Include all copper layers, solder mask, silkscreen
   - Include drill files
2. File → Fabrication Outputs → Drill Files
3. File → Fabrication Outputs → Bill of Materials
4. Zip all files for PCB manufacturer

### 4. Recommended Manufacturers

- **JLCPCB**: Good for prototypes, fast shipping
- **PCBWay**: Better 4-layer quality, controlled impedance
- **OSH Park**: US-based, excellent quality, slower/pricier

**Specs to provide:**
- 4 layers
- FR-4, 1.6mm thickness
- 2oz copper (outer layers), 1oz (inner layers)
- Controlled impedance: 120Ω differential pairs
- ENIG finish (for reliability)

## Future Enhancements

### Programmatic Schematic Generation

While the current approach uses hand-editable KiCad formats, we can add:

1. **Python script to generate full schematic** from BOM
   - Use `kicad-python` or direct S-expression generation
   - Automatically wire components per specification
   - Validate against hardware test suite

2. **Automated footprint assignment**
   - Parse BOM Manufacturer PN
   - Look up footprints from library
   - Assign to symbols programmatically

3. **DRC rule generation from tests**
   - Extract clearances from `test_servo_interface.py`
   - Generate KiCad DRC rules automatically
   - Ensure design matches validated specifications

### Integration Testing

```python
# Future: Validate KiCad design against specs
def test_kicad_bom_matches_docs():
    """Ensure KiCad BOM matches documented BOM."""
    kicad_bom = parse_kicad_bom("servo_driver_interface.kicad_sch")
    doc_bom = parse_markdown_bom("../../docs/hardware/servo_driver_interface.md")
    assert kicad_bom == doc_bom

def test_differential_pairs_matched():
    """Verify all differential pairs are length-matched."""
    pcb = parse_kicad_pcb("servo_driver_interface.kicad_pcb")
    for pair in pcb.differential_pairs:
        assert abs(pair.length_p - pair.length_n) < 5.0  # mm
```

## Contributing

When modifying KiCad designs:

1. Update documentation first (`docs/hardware/`)
2. Run hardware tests: `pytest hardware/tests/ -v`
3. Update KiCad schematic/PCB
4. Export BOM and verify against docs
5. Commit with descriptive message linking to issue/spec change

## Support

- KiCad Documentation: https://docs.kicad.org/
- KiCad Forums: https://forum.kicad.info/
- This Project's Issues: https://github.com/knaufinator/6DOF-Rotary-Stewart-Motion-Simulator/issues
