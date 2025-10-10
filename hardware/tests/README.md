# Hardware Testing Requirements

Install Python dependencies for circuit simulation and validation:

```bash
pip install PySpice numpy matplotlib pytest
```

## Optional: Full SPICE Simulation Setup

For advanced signal integrity analysis, install:

### Linux/WSL:
```bash
sudo apt-get install ngspice libngspice0 libngspice0-dev
pip install PySpice
```

### Windows:
1. Download ngspice from: http://ngspice.sourceforge.net/download.html
2. Install to `C:\Program Files\ngspice`
3. Add to PATH: `C:\Program Files\ngspice\bin`
4. Install PySpice: `pip install PySpice`

### macOS:
```bash
brew install ngspice
pip install PySpice
```

## Test Suites

### 1. **Electrical Specification Tests** (`test_servo_interface.py`)
   - RS-422 voltage levels and impedance matching
   - Termination resistor validation
   - E-stop circuit timing and current limits
   - Buck regulator output ripple
   - Signal rise time budget analysis

### 2. **SPICE Simulation Tests** (`test_spice_simulations.py` - optional)
   - Transient analysis of differential pairs
   - Frequency response of termination networks
   - Power supply stability under load transients
   - E-stop relay switching dynamics

### 3. **PCB Layout Tests** (`test_servo_interface.py`)
   - Ground plane coverage verification
   - Differential pair length matching
   - Trace impedance calculations

## Running Tests

```bash
# Run basic electrical spec tests (no SPICE required)
pytest hardware/tests/test_servo_interface.py -v

# Run with coverage
pytest hardware/tests/ --cov=hardware --cov-report=html

# Run only safety-critical tests
pytest hardware/tests/ -k "estop or safety" -v
```

## CI/CD Integration

These tests can run in GitHub Actions without requiring physical hardware:
- Validates BOM component selections
- Catches specification violations during design
- Ensures safety margins are maintained
- Documents electrical design decisions in code

## Future Enhancements

1. **KiCad Integration**: Extract component values directly from schematic
2. **Gerber Validation**: Parse PCB files to verify trace widths, clearances
3. **Thermal Analysis**: Model power dissipation in buck regulator
4. **EMI Simulation**: Predict radiated emissions from differential pairs
5. **Mechanical Stress**: Validate connector torque specs and vibration resistance
