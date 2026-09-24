# 6DOF 2 r13 routed source checkpoint — 2026-09-23

**UNTESTED PCB — DO NOT ORDER FROM THIS CHECKPOINT.**

This replaces the 2026-09-22 paused/off-board-parts checkpoint. All 21 added
components are placed and routed. Native EasyEDA DRC is empty after save, close
and reopen. This is a design-source handoff, not a released JLC fabrication/assembly
package or a tested motion controller. No r13 order was placed.

## Files

- `6DOF2-r13-20260923-routed-UNTESTED-DO-NOT-ORDER.epro2`: exported EasyEDA Pro
  project, including schematic, PCB and project library. Open/import into a separate
  project copy for review; do not overwrite your only existing project.
  Archive-content checks do not replace an import round-trip test.
- `r13_routed_overview_clean.png`: inspected native board overview. Olive shading
  is the inner GND plane; hatching denotes intentional keepouts. The ESP32 antenna
  end intentionally overhangs the right edge; edge-facing connector bodies may
  extend past the outline. These are not staged, unplaced new parts.
- `PCB-source.txt`, `Schematic-source.txt`, `Schematic-netlist.json`: raw recovery
  evidence, not Gerbers or standalone portable projects.
- `evidence/`: native DRC, source/label/plane verification and export evidence.
- `SHA256SUMS.txt`: checkpoint integrity manifest, not release approval.

## Verified layout state

- 170 × 200 mm, four layers; six DB25 connectors and six mounting holes.
- 144 components, 654 physical pads, 967 tracks, 398 vias, 689 fills, eight regions
  and one rebuilt native GND pour.
- All 144 reference labels visible; no label anchors outside the board.
- New routes and label positions survived save/close/reopen. Ground islands were
  checked against actual generated plane copper, not just a routing assumption.
- Native DRC: zero errors (`102_final_drc.json`, source checkpoint `100_*`).

## Before ordering

Generate and audit one matching r13 Gerber/drill/BOM/CPL/STEP set; do not reuse
archived r12 files. Complete schematic/PCB comparison, mechanical/3D review,
silkscreen/font qualification, JLC CAM and processed assembly-stock checks.
Review all six DB25 assembly rotations, including the retained-footprint 180°
correction. Required parts must not be omitted to bypass a stock warning.

The repaired USB segment has a verified reference corridor. Legacy USB transitions
still include via-antipad interruptions; existing transitions and stackup impedance
are not signal-integrity-qualified. Bench qualification remains required for USB
power/inrush, regulator temperature, rail thresholds and inhibit behavior.
Zero DRC does not establish electrical performance or motion safety.

### Project-import caveat

The exported archive passes CRC and matches the active schematic/PCB electrical
geometry, associations and corrected TI footprint caches. It is not byte-identical
to the raw source: EasyEDA omits two Inner1/Inner2 physical-layer records and 125
generated/default rule-selector records. Import restoration of the four-layer
stackup and rule defaults has not been tested. After importing a separate copy,
verify those settings against `PCB-source.txt`, rebuild the plane, save/reopen and
rerun DRC before using it. See `evidence/103_portable_archive_independent_audit.json`.

## Programming and testing

The hardware is designed for USB-alone programming, without 12 V. USB-only operation
must leave drive outputs disabled. Firmware is deferred: GPIO47 inhibit/GPIO48 arm,
removal of the GPIO48 status-LED waveform, supply supervision, E-stop/alarm handling,
protocol fixes and pulse-engine safeguards are listed in
`../../6DOF2_FIRMWARE_TODO.md` in the repository drop-off.

Do not connect energized drives or run motion with old firmware. Start with
disconnected-drive, current-limited bench tests and implement the independent
machine emergency-stop/braking system. Read the parent `HOW_TO_BUY.md` and
`HOW_TO_USE.md`; these are prototype instructions, not a working-package claim.
