# r13 shutdown checkpoint — 2026-09-22 evening

**PAUSED AT OWNER REQUEST. UNTESTED / INCOMPLETE / DO NOT ORDER.**

The owner is shutting down the computer. Do not resume automatically or order parts.
No order, payment, commit or push was performed. Preserve all pre-existing staged
and unstaged firmware-repository work.

## Actual saved state

- EasyEDA project `16231355292e4948ba37840dcdf926b0`, **6DOF 2**.
- PCB `bc0fa261f3c54e66b7be29928385553a`; schematic page
  `fd2c3589563945538936811f609de096`, parent
  `d897bc7d6ee2454dbccbd8e2a193104a`.
- Schematic: 144 parts, 640 pins, 496 connected pins, 124 nets. The earlier
  save/reopen audit passed 603 assertions; later supplier/footprint-only edits
  still need a fresh netlist and comparison.
- Circuit includes diode-OR LOGIC_5V for **USB-alone programming**, genuine
  LDL1117S33R, supervised four-IC latched driver inhibit, GPIO47 inhibit / GPIO48
  fresh-edge arm, local driver-enable pulls, 1k LS-input pulls, USB series resistors
  and corrected capacitor choices. `inhibit_design.md` is the exact contract.
- Nine schematic section boxes and local named stubs are present. The old A4 frame
  is too small for the entire drawing; schematic PDF/layout witness remains pending.
- PCB has 144 parts / 654 pads after import. **21 new parts are still staged
  off-board. The 26-part placement plan has NOT been applied. Old copper remains
  and conflicts with changed pad nets. Do not treat the visible PCB as functional.**
- Last known pre-placement copper inventory: 876 tracks, 344 vias, 745 fills,
  8 regions, one native GND pour. No routing/thermal cleanup has been applied.
- Six DB25 connectors, six mounting holes, board outline and existing high-speed
  routes remain geometrically unchanged.
- `10_normalize_source_suppliers.json`: corrected 106 legacy supplier tokens in
  BOTH schematic and PCB; the other 38 were already public codes. Recheck all144
  after reopen and require a zero-action schematic/PCB comparison before ordering.
- `23_relink_land_patterns.json`: latest captured schematic and PCB sources,
  saved then reopened after footprint links for U11/U12/U14 were changed.

## Footprints: important persistence lesson

Direct modification of a placed child pad reported success but reverted after
save/close/reopen (`13_*`, `14_*`). Do not retry that approach.
Copied two project footprints into the actual personal library, then edited them:

| Footprint | Personal-library UUID | Verified lands |
| --- | --- | --- |
| 6DOF_R13_TI_DCU0008A | 19dd43e481dc47618ab886e576371f22 | 8 pads, 33.5 × 11.9 mil |
| 6DOF_R13_TI_DCK0006A | ee9d76f865d2477fa5cf33121b9a2118 | 6 pads, 15.8 × 35.5 mil |

Personal library: `dfa15b47affc40b8846523cd8aea0214`. Dimensions were rounded UP
to EasyEDA's observed 0.1 mil pad-dimension precision; centres retain 0.0001 mil
precision. `22_quantize_land_patterns_reconciled.json` proves both footprint
editors reopened with these sizes. `23_*` relinked the three instances in both
documents, but **placed child-pad geometry after relink has not yet been audited**.
Upstream Device associations may still hold old footprints; verify a fresh full
import comparison and actual child geometry after a second save/reopen.
Library-copy source must be the project UUID, not the personal library UUID.
`openInEditor` on an existing tab needs explicit activation; do not assume focus.

## Ready offline plans, not applied

- `placement_plan_r13.json` / `placement_audit_r13.json`: 26 placements (21 new,
  five moved power components), 20 quantized TI lands, 12 local route sketches.
  Independent continuous checks passed 3,393 envelope comparisons; minimum gap
  0.299926 mm. U6 and 118 other components fixed. U7 thermal reserve ~828 mm²
  before routing; require >=500 mm² actual combined copper and >=6 thermal vias.
- Copper-migration analysis is being checkpointed separately. U5.4 was a VCC5
  trunk junction; both touching trunks must be removed/rerouted before DRIVE_ENABLE
  can be safe. Changed /G pins also still touch old GND copper.
- `route_delta_r13.py` is an unexecuted candidate-router wrapper, not validated
  r13 routing. It must use a fresh placed/ripped state, never global old state.
- Full 144-part / 46-SKU catalog check found sufficient nominal availability,
  including C77833 DB25 (30 nominal for five boards). This is NOT JLC allocation
  or processed-BOM approval. `SOURCING_PREFLIGHT.md` records qualifications.

## Resume in this order

1. Read PCB Project Development, full EasyEDA skill and this checkpoint. Rediscover
   bridge/window identity after restart; `run.mjs` pins the old window and must fail
   if identity changes. Snapshot before modifications.
2. Reopen actual project; verify three corrected placed IC footprints, all144
   supplier IDs, six DB25/outline/mount geometry, fresh schematic netlist and full
   schematic/PCB comparison. Resolve any footprint-cache problem first.
3. Apply the audited 26-part placement, exact old-copper migration, then freshly
   export state. Remove old U7 thermal fills/vias and all stale retagged-net contacts.
4. Route new topology; preserve/repair USB and Ethernet coupled routing deliberately.
   Build real U7 thermal copper, rebuild native GND pour, continuously audit geometry.
5. Save/close/reopen; exact inventory/net comparison, native DRC, no open/dead copper,
   plane coverage, connector orientation, clean editor/rendered witnesses.
6. Export a fresh full portable EasyEDA project and matching Gerber/GKO/drill,
   BOM/CPL (six DB25 CPL orientations independently checked), STEP, schematic PDF,
   renders and manifest. Do not relabel archived r12 files. Require JLC processed
   matching/allocation for every required part, including full THT assembly.
7. Deposit verified manufacturing package in repo, still **UNTESTED PROTOTYPE**.
   Firmware remains deferred; no ready-to-plug/functionally-tested claim.

## Repo drop-off

Repo: `C:/Users/Chris/Documents/GitHub/6DOF-Rotary-Stewart-Motion-Simulator`.
`docs/hardware/6dof2_r13/` contains draft HOW_TO_BUY/HOW_TO_USE and a clearly
marked WIP checkpoint, not fabrication outputs. Firmware TODO includes GPIO48
WS2812 conflict, GPIO47-safe defaults, qualification/rearm timing, stop/alarms,
USB transport, Ethernet packet/MAC and pulse-engine issues. Existing r12 artifacts
are retained as superseded history; unrelated dirty work has not been deleted.
