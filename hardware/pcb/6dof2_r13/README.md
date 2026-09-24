# 6DOF 2 r13 RC4 — untested prototype manufacturing handoff

**2026-09-23: ROUTED; NATIVE DRC EMPTY; UNTESTED PCB. Nothing ordered or paid for.**
The owner intends to order fully assembled prototypes for testing shortly.
**DO NOT ORDER** until the [exact release status](r13-rc4-2026-09-23/README.md)
and actual JLC processed-allocation/full-assembly gates are satisfied. Local file
verification is not a working-controller or factory functional-test certificate.

The r13 hardware revision is placed, routed, labeled, saved and reopened.
PCB source checkpoint `202_final_pcb_capture.json` has **144 components,
654 pads, 967 tracks, 398 vias, 689 fills, eight regions and one native GND pour**.
Native DRC `203_final_drc.json` has no errors. All 144 designators are visible with
no outside label anchors. U2's antenna overhang is intentional; no new parts remain
staged off-board. The six matching DB25s use **DS1034-25FUNSI44 / C77833** and twelve
corrected **3.20 mm flange holes**. Schematic checkpoint189 retains all 144 parts /
640 logical pins and nine labeled design sections, with the old frame/overlapping
subtitle removed and the ESP32 bottom net labels separated from its pin text.

Current handoff: [r13-rc4-2026-09-23](r13-rc4-2026-09-23/README.md).

| Folder | Use |
| --- | --- |
| `fabrication/` | Gerber/drill ZIP, uploaded to JLC's PCB field |
| `assembly/` | Normalized BOM and CPL, uploaded separately; raw exports retained |
| `mechanical/` | Actual-model STEP and documented fit/model limitations |
| `source/` | Portable EasyEDA archive, exact native sources and netlists |
| `drawings/` | Board views and 30-page indexed native schematic review PDF |
| `evidence/` | DRC, connectivity, CAM, orientation, sourcing and persistence audits |

Use the release README and SHA-256 manifest for exact filenames and acceptance.
A portable source project or outer transfer ZIP is **not a fabrication upload**.
The earlier archive-default omission was investigated with a separate imported copy:
the current PCB's four-layer stack/rules restore with empty native DRC. Keep the
included raw source and final archive/import evidence. Never test an import over the
working production project. The previous routed/shutdown checkpoints and RC1/RC2/RC3/r12
files are history; do not use them to fill gaps or mix revisions.

The final gray-strip/oversized-background artifact was a stale EasyEDA `BOARD.path`
display cache. Regenerating board shape from the existing closed outline and
save/close/reopening changed only that cache; all copper, components, nets and the
170 x 200 mm outline stayed exact. Final native DRC and the manual JLC outline
preflight passed; the confirmation was cancelled without ordering. The RC4 package
contains fresh PCB exports and the unchanged, hash-rebound SCH189 review PDF.

Independent audits prove exact routing persistence, all non-GND copper connected,
and all 123 GND pads joined to the actual single native ground plane. The repaired
USB pair's reference corridor passes. Fourteen legacy USB trace pieces approach
their own or the other USB leg's via antipads; no unrelated voids or new reference
loss were found. This does not establish controlled impedance or measured USB
signal integrity. Native DRC is not electrical or firmware validation.

The released population is **144 required components / 46 supplier SKUs**, no DNI.
Use **Standard PCBA**, including all SMT/THT/hybrid joints and all nine THT components
per board. Five assembled boards need 30 C77833 DB25s before JLC's actual attrition.
The September 23 public catalog screening is not reserved assembly allocation.
Require zero unmatched/shortfall lines, complete population and accepted connector
orientations in the processed order; do not uncheck required parts to clear a warning.
Even after manufacture is permitted, these remain **untested prototypes**.

Firmware is explicitly deferred. [Required changes and bench acceptance](../6DOF2_FIRMWARE_TODO.md)
must be completed before drive-connected operation. There is no released r13 binary,
factory programming image or functional-test procedure yet. Ordinary assembly alone
will not deliver a proven working motion controller.

See [the board overview](../6DOF2_BOARD.md) for the current revision and interface status.

## Prototype instructions

- [How to buy](HOW_TO_BUY.md): complete JLC SMT/THT assembly, required connector
  quantities, separate Gerber/BOM/CPL uploads and processed-order checks.
- [How to use](HOW_TO_USE.md): USB-alone programming, power/inhibit contract,
  disconnected-drive bench gates and the firmware work required before motion.

Both guides remain prototype instructions. Their presence, a clean DRC result or a
complete transfer package does not mean an order or functional test was completed.
