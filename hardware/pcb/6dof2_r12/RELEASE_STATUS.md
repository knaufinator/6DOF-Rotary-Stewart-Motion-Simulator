# 6DOF PCB r12 manufacturing status

## Status: RELEASE CANDIDATE - not ordered

r12 supersedes r11. It is the full design pass the owner asked for after JLC's r11 DFM
finding: placement and pin assignments reviewed, jagged routes replaced, electrical and DFM
findings applied, coupled differential pairs, thermal copper, plain-language silkscreen, and a
matching two-part enclosure (Fusion `6DOF_2_enclosure_r12c`). The r9 order
W2026090303177354 was cancelled and refunded by JLC when the revised files were requested, so
r12 is the revision to re-order. **Ordering requires the owner's explicit authorization.**

Review record: `review_r12/PLAN.md`, `review_r12/SYNC.md` (decisions I1-I11), reports A-H.

## What changed from r11

| Area | Change |
|---|---|
| Placement | 26 parts moved: decoupling caps to their pins (C25 was under the jack, C8/C9/C10, C18/C19, C17, C4), R6/R7 un-swapped and spaced at x 120.2, C6 off the board edge, R10/C26/C27 clear of U8, D6 and R44 per stream C |
| Copper | every affected net re-routed; 29 via-in-pad vias removed and 14 nets re-routed; ETH TX/RX, USB and USB_MCU routed as coupled pairs with GND stitch vias (stream H); ACTLED on Inner2 (the only clean path, decision I11); stream D smoother applied board-wide: micro-jogs 1,362 -> 0, segments 4,480 -> 1,738, length -1.06 % |
| Thermal | U7 (AMS1117) tab: 34 mm2 Top + 170 mm2 Bottom VCC3V3 copper with 6 vias; U6 (XL1509) GND pins: 24 mm2 Top with 3 vias. E's 500/150 mm2 targets are not reachable in the space; this is 20x / 3x the r11 copper |
| Silkscreen | 18 strings at 70 mil (1.07 mm rendered cap height), 6 mil stroke (r11's 25-40 mil text was under JLC's 1 mm minimum): MOTOR 0..5, USB-C, +12V / GND / 12V IN, E-STOP, RESET / BOOT, UART with GND TX RX, ETHERNET. Body clearance checked offline (min 0.44 mm) and on the render |
| BOM | C15 (W5500 TOCAP) LCSC code corrected: C15850 (10 uF) -> C123653 (4.7 uF 0805 Walsin 0805X475K100CT, per datasheet). In stock at JLC. Fixed in `easyeda/ref2lcsc.json` and on the EasyEDA component |

## Gates

| Gate | Result |
|---|---|
| offline clearance (`v2_check_plan.py`, every applied delta) | 0 issues |
| open connections (`v3_open_report.py`) | 0 |
| dead copper / hanging tails (`v2_dead_stubs.py`) | 0 / 0 (GND stitch and thermal vias with no track are intentional plane vias) |
| smoother gate (`v4_gate.py`) | PASS, jogs 0 |
| native DRC after save/close/reopen | 0 violations (empty drc array) at 876 tracks / 344 vias / 745 fills / 53 pour fills |
| Gerber audit / BOM-CPL / stock | see `release_audit.json` |

Native DRC found four same-net artefacts the offline gates skip (a duplicate GND via, two GND
vias 0.5 mm apart, a dead ETH_RX_BIAS via, a dead VCC3V3 track); all removed by
`v3_97_drc_cleanup.js` before the export.

## Also done with r12 (not in the package)

- Firmware (phoenix branch of 6DOF-Rotary-Stewart-Motion-Simulator, `Controller/`): E-stop for PCB v2 on GPIO 21, active HIGH (NC loop: LOW = closed = run, HIGH = open or broken wire = stop), edge tests made polarity-independent, monitoring enabled at boot, and a boot-time check that starts paused if the loop is already open. Uncommitted in the working tree.

## Carried forward

- Component bodies are still not modelled (only F1's strip and the jack block are keepouts);
  the r10 lesson stands - check the JLC CAM/DFM view for body collisions.
- RS-422 line protection on the motor-driver pairs; VBUS/VIN12 full-height runs.
- Enclosure open questions in `review_r12/reports/F-enclosure.md`.
