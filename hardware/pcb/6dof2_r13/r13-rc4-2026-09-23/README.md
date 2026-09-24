# 6DOF 2 r13 RC4 — untested prototype

**DO NOT ORDER. Nothing has been ordered or paid for.** This is the completed
local PCB-design handoff, not a validated, programmed, plug-and-run servo controller.
Firmware adaptation and physical testing are still required. See [STATUS.md](STATUS.md)
for the evidence and remaining purchasing gates.

The board remains **170 x 200 mm, four layers**. This correction did not enlarge
the PCB. It corrected the twelve mounting holes in the six matching Connfly
DS1034-25FUNSI44 / C77833 DB25 footprints to 3.20 mm, with one local trace adjustment.
The six separate board-standoff holes remain 4.5 mm. All 144 components are placed
and routed; the ESP32 antenna's intentional edge overhang is not a staged part.

The final source is PCB capture202 and schematic capture189. The old gray/black
edge artifact was a stale EasyEDA `BOARD` display shape, now regenerated from the
existing closed perimeter. The actual routing, ground plane and manufacturing
outline did not change during that display repair. The schematic has nine labeled,
bounded functional sections with named inter-section nets.

## Open or inspect

- Import [the portable EasyEDA project](source/6DOF2-r13-rc4-20260923-UNTESTED.epro2)
  into a **new project**. Keep the supplied raw source and evidence as the baseline.
- Read [the schematic review PDF](drawings/6DOF2_r13_Schematic_Review.pdf) and
  [PCB overview](drawings/208_final_pcb_review.png). The PCB image hides Inner1 for
  readability; the ground plane is present in the design and fabrication outputs.
- Use [the real-model assembly STEP](mechanical/6DOF2_r13_assembly.step) for
  mechanical review, subject to the documented connector/body-model limitations.
- Verify `SHA256SUMS.txt` and run
  `python evidence/tools/verify_rc4_package.py .` from this package folder before
  using the files. Do not mix this package with earlier checkpoints or RC2/RC3 outputs.

## Files for a future authorized JLC quotation

| Purpose | File |
| --- | --- |
| PCB fabrication, including drills | [Gerber ZIP](fabrication/6DOF2_r13_Gerber.zip) |
| Fully populated assembly | [JLC BOM](assembly/6DOF2_r13_BOM_JLC.csv) |
| Placement | [JLC CPL](assembly/6DOF2_r13_CPL_JLC.csv) |

Upload those three inputs separately; the entire handoff folder is not a Gerber
upload. The normalized BOM includes **46 SKUs / 144 required parts**. Use the
normalized CPL as supplied: its six DB25 rotation corrections are already applied.
The untouched native exports are retained under `assembly/raw/`.

Request **Standard PCBA including all SMT and through-hole/hybrid assembly**;
the nine through-hole components per board must not be omitted. Five boards need
30 DB25 connectors before attrition. Public catalog availability is not confirmed
allocation. Review JLC's processed BOM, connector orientations, CAM/DFM and full
assembly scope before authorizing any purchase. See [the buying guide](../HOW_TO_BUY.md).

## Bring-up is still pending

USB-alone programming is supported by the design intent; external 12 V is not
required for programming. This has not yet been proven on hardware. USB power is
not motor/drive power. Firmware work is listed in
[6DOF2_FIRMWARE_TODO.md](../../6DOF2_FIRMWARE_TODO.md), including GPIO47 inhibit,
GPIO48 fresh-arm behavior and power-restoration sequencing. Existing firmware
must not be assumed compatible.

Begin with an unpowered inspection and current-limited, unloaded bench bring-up.
Use all six insulated standoffs and local support while mating connectors; do not
mate/unmate a powered servo system. Verify cable pinouts, supply behavior, inhibit
and re-arm behavior before drive connection, and follow the project's single-motor
test procedure before any multi-axis motion. This PCB is not a certified E-stop,
STO or brake system. Enclosure fit, connector-force support, thermal behavior,
USB/Ethernet performance and EMC remain physical-test work.

The handoff preserves source snapshots, native DRC, source/netlist comparisons,
actual isolated-import checks, CAM/drill/ground-plane checks, real STEP validation,
schematic image/PDF coverage, sourcing evidence and local JLC outline-preflight
screenshots. Zero DRC is a design check, not proof of working hardware.
