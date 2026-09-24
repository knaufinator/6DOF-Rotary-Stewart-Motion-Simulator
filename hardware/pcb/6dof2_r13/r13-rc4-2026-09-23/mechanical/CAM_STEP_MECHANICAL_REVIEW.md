# RC4 CAM, STEP and mechanical review

**Local audit PASS. UNTESTED prototype. DO NOT ORDER.**

Bound inputs: saved/reopened PCB202, schematic189, archive204, fabrication/model export205 and STEP206. Independent import212 preserves active source, physical stackup/rules and resolved netlist with zero native DRC. Every current audit/render below was freshly generated from RC4 files.

## Measured results

- `source_hole_plane_audit.json`: **1,477 checks pass**. Component placements and pad geometry are preserved; the twelve corrected DB25 NPTHs and the precise M4S_I local repair are verified. Plane differences from checkpoint100 outside 2.5 mm neighborhoods of the twelve holes total 9.15e-13 mm², numerical noise.
- `cam/strict_cam_source_audit.json`: **51 checks pass**. Eleven Gerbers use explicit metric3.6; the single four-edge closed profile measures 170 × 200 mm. All **581 distinct drills/slots** reconcile, including twelve actual **3.199999 mm** DB25 NPTHs. The separate via drill file duplicates 398 accounted vias. Bottom Paste is legitimately absent because all assembled parts are Top-side.
- `cam_equivalence.json`: **29 checks pass**. Expanded painted geometry matches RC3 exactly, preserving the sequence of dark/clear painting groups; drill multisets also match. Native timestamp, unused-aperture and aperture-number differences do not change geometry. All ten nonempty layer PNGs plus the registered copper/outline/drill composite are byte-identical. No prior vertical plane artifacts remain in manufacturing views.
- `step_model_reconciliation.json`: **435 checks pass**. Actual OpenCascade geometry contains **144 real valid component assemblies / 861 component solids**, plus the PCB; no stand-in boxes. Assigned models and native placements reconcile.
- `step_inventory_equivalence.json`: **147 checks pass**. Every actual component and PCB bounding box, volume, face/solid count, BRep validity and transform matches RC3 exactly. Fresh PCB bounds are X −19.9873407..149.9875998 mm and Y 0.0126999..199.9873001 mm, consistent with the nominal Gerber outline and exporter edge treatment. The actual board contains the corrected holes; its unchanged volume is 56,138.3083973 mm³. This is a geometric-inventory comparison alongside exact native-source checks, not an independent topological-isomorphism proof.

Fresh CAM and actual STEP top/bottom/isometric views were inspected. STEP renders intentionally omit copper/silkscreen appearance layers and are mechanical views, not assembly-placement drawings.

**Stackup qualification remains open.** The intended layer order is Top / Inner1 GND / Inner2 signal / Bottom, nominal 1.6 mm board and approximately 1 oz copper per layer. Actual source202 `LAYER_PHYS` records specify Top 1.379 mil and the other three copper layers 1.378 mil each (about 35 µm), one FR4 record of 59.449 mil (1.5100 mm), and two 0.394 mil masks. Copper plus that single dielectric sums to 1.6500348 mm; including masks gives 1.67005 mm, matching the actual STEP. The records do not define all three dielectric separations of a qualified four-layer fabrication stack. Persistence/import success does not prove manufacturing or controlled-impedance validity. JLC must accept an actual dielectric/copper construction and the design's high-speed reference-spacing/impedance review before any order. No CAD geometry was changed to disguise this remaining gate; see `stackup_qualification.json`.

The gray left strip was traced to a stale serialized `BOARD` background, separate from the correct layer-11 outline and fabricated/STEP body. Native Generate Board Shape corrected only that record; all other source and API state remained exact. Local JLC preflight210 now recognizes the outline, reaches confirmation without the warning and was cancelled. No order was submitted.

## Mechanical limits

The selected Connfly DS1034-25FUNSi44's **3.20 ±0.05 mm** board-hole requirement is implemented. Its flange maximum is 53.34 mm and mounting pitch 47.04 ±0.2 mm. The real but legacy Amphenol L77SDB25SOL2 assigned model does not qualify the exact Connfly locking-screw stack or cable hood; check delivered parts and all six mating cables before motion.

The middle DB25 row is about 91.5 mm from its nearest board mount. Use all six mounts on correctly spaced insulated standoffs and an insulating local support fixture during de-energized cable mating. Connector-load transfer and cable strain relief require physical testing; corner mounts alone do not establish stiffness.

C3 is **1206/3216, not 1210**. Its maximum 3.4 × 1.8 × 1.8 mm body exceeds the generic model by 0.5 mm in height; that allowance is included in headroom checks. Current models clear the nominal r12 flat floor/lid vertically by at least 3.170/3.191 mm, but this does **not** qualify that enclosure. Review MagJack body/plug/latch/boot access, preserve U2's antenna overhang and metal-free area, and revise ventilation above U7 around PCB X110..133.5 / Y100..124 mm. No enclosure files were changed by this audit.

## Artifact identity

| RC4 artifact | SHA256 |
| --- | --- |
| Gerber ZIP | `44fe2c34b97ee5ce68c5447788e1e77647e0741dcf36d1a453a79871524876c6` |
| STEP, 76,547,181 bytes | `456de7a4919cc36c6f6c2673de218e552bf4b9c5732ff15f9c2519bbec40a715` |
| Portable project | `c42dc31219236dca955aaf34b4688501a36ff6eb6fcefcf141cbf48ceb0c9ea4` |

`STATUS.md` lists remaining JLC, firmware, physical and prototype-test gates. Nothing here authorizes ordering or powered motion.
