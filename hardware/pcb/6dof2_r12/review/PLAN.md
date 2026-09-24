# 6DOF r12 — full design pass before resubmission

**Written 2026-09-03.** JLC cancelled order W2026090303177354 for file replacement; nothing is
in production and we are free to resubmit when this pass is finished. r11 is the baseline.

## Goal

Ship r12: the same electrical design, reviewed end to end by independent reviewers, with
placement tidied, routes straightened, every component and pin assignment justified, and an
enclosure designed against the finished board. No gate we already pass may regress.

## Baseline (r11, live `6DOF 2`, PCB `bc0fa261f3c54e66b7be29928385553a`)

- 170 x 200 mm, 4 layers (Top / Inner1 GND pour / Inner2 signal / Bottom), 123 components,
  883 tracks, 312 vias, 648 fills. Native DRC 0, 0 open connections, 0 dead copper.
- Six M4 mounts at (-10,10) (-10,190) (80,10) (80,190) (140,10) (140,190) mm.
- Known weaknesses going in: routes are A*-grid staircases (4 mil steps, many 45 degree
  micro-jogs); component bodies are modelled nowhere (two DFM misses so far: J9 over F1, silk
  under the DB25 shells); VBUS/VIN12 run the full board height; antenna sits ~55 mm inboard.

## Workstreams (run in parallel, all READ-ONLY against the board)

| id | stream | owner agent | deliverable |
|---|---|---|---|
| A | Placement & alignment | placement-review | `reports/A-placement.md` + a proposed move list with coordinates |
| B | Pin assignments | pin-review | `reports/B-pins.md` - every ESP32/W5500/driver pin justified or flagged |
| C | Electrical routing | electrical-review | `reports/C-electrical.md` - widths, pairs, decoupling, plane, stitching |
| D | Route geometry | route-geometry | `reports/D-routes.md` + `tools/v4_smooth.py` (offline, checker-gated) |
| E | DFM & mechanical | dfm-review | `reports/E-dfm.md` - body clearances, access, wire room, silk |
| F | Enclosure | enclosure-design | `reports/F-enclosure.md` + `enclosure/` Fusion script + spec |

The board has ONE writer: the integrating session. Reviewers never call the EasyEDA bridge.
They work from `data/` and the offline tools, and write only inside `review_r12/`.

## Order of integration (after reports land)

1. Read every report; reconcile conflicts in `SYNC.md` (decisions section).
2. Apply placement moves (A, E) through guarded scripts; re-route affected nets.
3. Run the route smoother (D) over the whole board; gate on `v2_check_plan.py` 0 issues and
   `v3_plan_open.py` 0 opens; apply as a rip+create delta.
4. Electrical fixes (C) and pin changes (B) - pin changes are schematic edits and get their own
   confirmation step; they are the only thing here that can change function.
5. Pour rebuild, save/close/reopen, native DRC 0, dead copper 0, silk re-rendered.
6. Cut r12: export, audits, hashes, status; package for JLC.
7. Enclosure (F) finalised against the r12 STEP.

## Non-negotiable gates

Native DRC 0 after reopen; `v3_open_report.py` 0; `v2_dead_stubs.py` 0/0; Gerber audit 0;
BOM/CPL 0 errors; every silk label visually confirmed clear of component outlines; every
connector body clearance stated numerically (no more "the pads are clear").

## Where things are

- Board tools: `G:/My Drive/projects/6dof/pcb/easyeda/` (see `V2_CONSOLIDATION_HANDOFF.md`
  toolchain table). Units mil, `MM = 39.37007874015748`, y up, origin at board (-20, 0) mm.
- Data snapshot for this pass: `review_r12/data/` (state, CPL, BOM, renders).
- Design intent and verified-correct items: `V2_REQUIREMENTS.md` (sections 2, 4, 6 especially).
- Coordination: `review_r12/SYNC.md` - read first, append last.
