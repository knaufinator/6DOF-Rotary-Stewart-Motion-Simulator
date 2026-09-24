# r13 saved checkpoint 100 — 2026-09-23

**ROUTED / NATIVE DRC EMPTY / UNTESTED / DO NOT ORDER.**

The earlier shutdown checkpoint has been resumed. The owner requested continuation
and placement of the staged parts. No order or payment was performed. Preserve all
pre-existing staged and unstaged firmware-repository work.

Current PCB evidence is `100_routed_labeled_capture.json`, captured
**2026-09-23 12:38:42 UTC / 08:38:42 EDT**. Capture SHA-256:
`22c0bede843ee63a5d1dca034326b00e9a1ef2eed144f9599e369c0ddc7cd5e3`.
Native source text SHA-256:
`284e27148e25e48cc817f879fc999f346fe6762017167230dc45a71f904084fe`.
`102_final_drc.json` reports an empty native DRC error list after reopen.
All earlier numbered captures are historical evidence, not the current baseline.
No complete manufacturing release has been generated or approved; nothing ordered.

## Actual saved state

- EasyEDA project `16231355292e4948ba37840dcdf926b0`, **6DOF 2**.
- PCB `bc0fa261f3c54e66b7be29928385553a`; schematic page
  `fd2c3589563945538936811f609de096`, parent
  `d897bc7d6ee2454dbccbd8e2a193104a`.
- Schematic: 144 parts, 640 logical pins, 496 wire primitives. `44_*` is the
  saved/reopened capture after association repair. The independent
  `44_independent_schematic_alignment_audit.json` passes 27 checks: all logical
  pin nets are identical to `42_*`, only six Device/Footprint attribute values
  changed, and all 144 required supplier codes match the reviewed 46-SKU plan.
  Exact capture `44_schematic_aligned_reopened.json` SHA-256:
  `83098d5466c120326d04a132530e04fc601eb62c707a6f182002f42066890bd8`.
  The original circuit audit records 496 connected pins and 124 nets; the source
  association repair did not change pin connectivity.
- Circuit includes diode-OR LOGIC_5V for **USB-alone programming**, genuine
  LDL1117S33R, supervised four-IC latched driver inhibit, GPIO47 inhibit / GPIO48
  fresh-edge arm, local driver-enable pulls, 1k LS-input pulls, USB series resistors
  and corrected capacitor choices. `inhibit_design.md` is the exact contract.
- Nine schematic section boxes and local named stubs are present. The old A4 frame
  is too small for the entire drawing; schematic PDF/layout witness remains pending.
- PCB checkpoint 100 has **144 parts / 654 physical pads / 967 tracks / 398 vias /
  689 fills / 8 regions / 1 native GND pour**. **All 21 new parts are on-board;
  routing is complete by native DRC and independently checked connectivity.** The initial 26-part plan (including five relocated
  power components) persisted in `40_*`. The 14-part control group subsequently
  moved -15 mm X / +25 mm Y in `62_*`; `64_*` independently proves all 14 anchors
  and 46 child pads, preserving every unrelated API object.
- Exact old-copper migration is applied: 60 tracks, 25 vias and 48 fills removed,
  four shortened track prefixes recreated in the historical `40_*` checkpoint.
  U7 now has two verified thermal copper fills, 12 thermal vias and repaired C3/C7
  GND access; `60_ground_access_independent_audit.json` records actual native-plane
  coverage and 14.1 mil thermal-via antipad clearance. Do not repeat those repairs.
- Six DB25 connectors, six mounting holes and the 170 x 200 mm outline remain
  fixed. U2's antenna overhang is intentional, not a staged part left off-board.
  USB routing changed deliberately; do not claim all old high-speed copper is
  unchanged. Native DRC is complete; high-speed electrical validation remains open.
- `70_other_signals.json` applied 15 signal tracks and 8 vias while removing the
  nine specifically proven isolated remnants in `68_isolated_track_removal_plan.json`.
  `72_*` rebuilt the native plane; `73_*` is the historical subsequent capture. The applied
  continuous candidate check connects ACTLED, LINKLED, M0D_P, M2D_N, M2D_P,
  M2_ALARM, M3_ALARM and U0TXD. This is not fresh native whole-board DRC proof.
- `74_*`/`75_*` corrected the USB reference corridor. `80_*` independently proves
  the local decoupler routing and actual native ground access. `84_*` completed
  STEP/GND, including reconnection of the retained M0S_I network. `88_*` applied
  the final joint 56-track / 32-via / four-track-removal routing plan. `93_*`
  removed one redundant same-GND via and replaced its short return trace to fix
  the final hole-spacing error. Do not repeat any of these completed batches.
- `91_routing_persistence_plane_independent_audit.json`: 349 passing checks,
  exact routing persistence and all 32 new non-GND antipads >=14.099 mil.
  `96_post_hole_ground_usb_independent_audit.json`: 102 passing checks for the
  last repair, empty native DRC, all non-GND copper connected, and all 67 strict
  GND islands / 123 GND pads joined to the one actual native plane through
  positive plated-copper overlap. This does not assume every GND via is joined.
- The four repaired USB traces retain full main-plane projection. Across all
  20 USB traces, 14 legacy pieces approach their own or another USB transition's
  antipad; every unsupported interval is classified, with no unrelated void or
  new reference loss since `85_*`. This is not controlled-impedance or paired-via
  return-path acceptance; electrical validation remains required.
- `98_*` applied label plan v2 and `99_*` reopened the board. `101_*` records all
  144 designators visible, zero outside anchors. `100_final_source_binding_independent_audit.json`
  passes 12 checks: every component, pad, track, via, fill, region, native pour
  definition and generated plane is identical to audited `96_*`; only 26 ATTR
  records and DOCHEAD changed. `102_*` is the final empty native DRC result.
- `r13_routed_overview_clean.png` (`105_*`) is the accepted settled editor witness.
  The first `104_*` overview contained a stale hover overlay and is not deliverable.
  Targeted labels, including C41 beneath its capacitor, were inspected at 568%.
  This is not global Gerber font/printability qualification.
- `10_normalize_source_suppliers.json`: corrected 106 legacy supplier tokens in
  BOTH schematic and PCB; the other 38 were already public codes. The `44_*`
  independent audit rechecks all 144 schematic codes against both sourcing and
  `40_*` PCB codes. Still require a zero-action full comparison before ordering.
- `23_*` is historical failed placed-footprint relink evidence, NOT final geometry.
  Use `37_*` for corrected footprint proof, `64_*` for the control relocation proof,
  `100_*` for current PCB source, and `44_*` for current schematic source.

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
editors reopened with these sizes. This did NOT update the placed component cache:
the PCB source bodies in `25_*`, `30_*` and GUI-replacement `33_*` were identical,
despite apparent success. Component API `modify()` does not accept Device or
Footprint association properties, and no corresponding setters are documented.

Successful recovery (`29_*`, `34_*`, `35_*`, `37_*`): create new custom Devices
whose associations point to the corrected footprints; instantiate new PCB
components with `pcb_PrimitiveComponent.create({libraryUuid,uuid},...)`; verify
physical child pad numbers/dimensions; restore each child net by pad number using
`pcb_PrimitivePad.modify(id,{net})`; delete only the exact superseded components;
restore original Designator, Unique ID and metadata. Do not copy reserved
Device/Footprint/Symbol keys through OtherProperty. Retain the original Unique ID
(gge137/gge138/gge140), not the old PCB primitive ID, for schematic linkage.
`37_independent_footprint_persistence_audit.json` proves corrected placed geometry
after save/close/reopen. Library metadata alone was not sufficient.

For the already wired schematic, no recreation was needed: the documented
`sch_PrimitiveAttribute.modify(attrId,{value:newUuid})` changed only six existing
Device/Footprint ATTR values. Preserve Symbol, PartId, Unique ID and all geometry.
`43_*` applied the repair; `44_*` reopened it; independent `42_*` versus `44_*`
comparison proves all 640 pin nets unchanged and every other source-body record
unchanged. This is verified on this build; do not assume an ATTR value edit alone
would replace PCB child geometry or work for a different pin-compatible part.

Final project-local associations (project `16231355292e4948ba37840dcdf926b0`):

| Parts | Device | Footprint |
| --- | --- | --- |
| U11 | 35db244c731d10c8 | 2fa12c8f68575cc0 |
| U12/U14 | 491001de77a17649 | 2470ffd604459d7e |

Full schematic/PCB import-action comparison remains a separate required gate.
Library-copy source must be the project UUID, not the personal library UUID.
`openInEditor` on an existing tab needs explicit activation; do not assume focus.

## Applied plans and remaining release work

- `placement_plan_r13.json` / `placement_audit_r13.json`: 26 placements (21 new,
  five moved power components), 20 quantized TI lands, 12 local route sketches.
  Independent continuous checks passed 3,393 envelope comparisons; minimum gap
  0.299926 mm. U6 and 118 other components fixed. U7 thermal reserve ~828 mm²
  before routing; require >=500 mm² actual combined copper and >=6 thermal vias.
- `copper_migration_plan.json`, `copper_migration_contact_audit.json` and
  `COPPER_MIGRATION_HANDOFF.md` describe the now-applied exact migration, recorded
  in `38_*` and persisted in `40_*`. The independent contact/gate analysis passed;
  this was necessary cleanup, not finished routing at that time. R18 VCC5 / R19
  VCC3V3 and all new topology are now connected and independently audited.
- `control_translation_final_guarded_plan_02.json` and `CONTROL_TRANSLATION_REVIEW.md`
  cover the applied 14-part translation, four local STEP cuts, nine local control
  traces and two 24/12 mil supervisor vias. `64_control_persistence_independent_audit.json`
  proves the saved result. Do not use the earlier unsuffixed plan: its candidate
  supervisor via was rejected for M3_ALARM clearance.
- `68_isolated_track_removal_plan.json` proves the nine applied cleanup deletions
  did not change pad-bearing connectivity; no components, vias, fills or planes
  were deleted. The much larger pad-free M0S_I network (11 tracks, 10 vias, 22 fills
  at that checkpoint) was intentionally preserved and was reconnected in `84_*`.
- `route_delta_r13.py` has now generated candidates that passed continuous checks
  and were applied in bounded batches. It is still an approximate search, not a
  substitute for fresh continuous validation, native DRC or plane evidence.
  `r13_include_islands.install(RO)` must run after `r13_exact_geometry.install(RO)`
  when reconnecting pad-free signal copper. Its SHA-guarded runtime patch changes
  only unsafe logging, exposes real non-GND copper islands, and invents no pads;
  seven offline tests pass. No shared legacy router source was edited.
- `other_signal_routes_final.json` is the earlier eight-net plan. Its former
  open-net list is superseded by `final_all_routing_frozen_85.json`, live `88_*`,
  final `100_*` and empty native `102_*`. All non-GND nets have exactly one
  physical copper island; no orphan or unnetted copper remains. Pad-bearing
  open-net counts alone would not prove that the retained remnants were joined.
- Full 144-part / 46-SKU catalog check found sufficient nominal availability,
  including C77833 DB25 (30 nominal for five boards). This is NOT JLC allocation
  or processed-BOM approval. `SOURCING_PREFLIGHT.md` records qualifications.

## Resume in this order

1. Read PCB Project Development, full EasyEDA skill and this checkpoint. Rediscover
   bridge/window identity after restart; `run.mjs` pins the old window and must fail
   if identity changes. Snapshot before modifications.
2. Confirm current checkpoint is at least `100_*` PCB / `44_*` schematic; capture
   current live state before any new plan. Do not replay completed recreation,
   placement, migration, thermal/ground, control, USB, routing, hole repair or labels.
   Verify three corrected placed IC footprints, all 144 supplier IDs, six DB25/outline/
   mount geometry, fresh schematic netlist and full schematic/PCB comparison.
3. Preserve the completed native-plane/USB proofs. Any further copper or via change
   requires renewed full connectivity, swept reference, antipad and native DRC checks.
   Review the remaining legacy USB transition/impedance limits explicitly; do not
   describe all 20 tracks as having uninterrupted ground beneath every point.
4. Finish full schematic/PCB comparison and manufacturing/mechanical review:
   connector orientation/body fit, ground/shield strategy, thermal assumptions,
   clean 2D/3D witnesses and r13 enclosure compatibility.
5. Portable export `103_*` has archive status
   `PASS_CORE_DESIGN_CONTENT_WITH_UNVERIFIED_IMPORT_SETTINGS`. CRC, core geometry,
   bindings and corrected cached footprints match PCB `100_*` / schematic `44_*`.
   The archive omits two Inner1/Inner2 `LAYER_PHYS` records and 125 default/generated
   `RULE_SELECTOR` records. Import restoration of stackup and rules is untested;
   do not claim exact round-trip equivalence. Preserve the included raw PCB source,
   which retains these settings, and verify a separately imported copy's layer stack
   and rule settings before relying on it. Do not import over current live work.
6. Generate matching fresh Gerber/GKO/drill,
   BOM/CPL (six DB25 CPL orientations independently checked), STEP, schematic PDF,
   renders and manifest. Do not relabel archived r12 files. Require JLC processed
   matching/allocation for every required part, including full THT assembly.
7. Deposit verified manufacturing package in repo, still **UNTESTED PROTOTYPE**.
   Firmware remains deferred; no ready-to-plug/functionally-tested claim.

## Repo drop-off

Repo: `C:/Users/Chris/Documents/GitHub/6DOF-Rotary-Stewart-Motion-Simulator`.
Current drop-off destination is `docs/hardware/6dof2_r13/routed-checkpoint-2026-09-23/`,
including `6DOF2-r13-20260923-routed-UNTESTED-DO-NOT-ORDER.epro2`, native sources,
netlists, audit evidence and checksum manifest. Root owns actual export/drop-off
completion. These source artifacts are not the complete manufacturing package.
The `.epro2` is a core-design archive with the explicit unverified import-settings
limitation above, not a proven exact settings round trip; keep `PCB-source.txt`.
`wip-checkpoint-2026-09-22/` is retained historical recovery evidence, not the current
PCB status. HOW_TO_BUY/HOW_TO_USE remain prototype instructions. Firmware TODO includes GPIO48
WS2812 conflict, GPIO47-safe defaults, qualification/rearm timing, stop/alarms,
USB transport, Ethernet packet/MAC and pulse-engine issues. Existing r12 artifacts
are retained as superseded history; unrelated dirty work has not been deleted.
