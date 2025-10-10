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

# Check BOM completeness
pytest .github/workflows/hardware_tests.yml -k bom
```

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
