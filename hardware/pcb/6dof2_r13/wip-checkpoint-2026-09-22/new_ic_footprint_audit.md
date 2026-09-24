# r13 new-IC footprint check

Scope: actual PCB pads in `09_capture_pcb.json`, captured after schematic import, not final placement/routing. All30 pad-number/net assignments pass the reviewed schematic contract; U11 pin3 is intentionally NC. All are top-side rectangular SMT pads without drilled holes. The numerical report includes every pad relative to its component origin. The staged negative coordinates do not indicate a footprint defect: these components have not been placed on the board yet.

| Part | Actual footprint copper, mm | Manufacturer comparison | Result |
|---|---|---|---|
| U7 LDL1117S33R | Pins1–3 2.499×1.100 at2.300 pitch; tab2.339×3.599; opposed banks at approximately±2.970 | ST SOT223 max leadwidth0.85, tabwidth3.15, pitch2.3, max leadtipspan7.3 | Compatible retained geometry. Pin1GND,2OUT,3IN,4tabOUT correct. |
| U11 SN74LVC1G74DCUR | Eight0.701×0.259 lands,0.500pitch, opposed-center span2.700 | TI DCU0008A example uses0.85×0.30,0.50pitch, center span3.10 | Pinmap/pitch correct; recommend official larger/outer lands before routing. |
| U12/U14 SN74LVC1G98DCKR | Six0.363×0.866 lands,0.650pitch, opposed-center span1.766 | TI DCK0006A example uses0.40×0.90,0.65pitch, center span2.20 in this rotated orientation | Pinmap/pitch correct; recommend official larger/outer lands before routing. |
| U13 TPS389001DSER | Pin1 0.841×0.279; otherfive0.749×0.279;0.500pitch; regular center span1.272 | TI DSE0006A example0.80/0.70×0.25;0.50pitch; center span1.20 | Compatible asymmetric pin1; no missing central thermalpad. |

The U11 and SC70 libraries overlap the component leads and are not proven hard failures. However, they reduce solder-joint tolerance relative to the manufacturers' example lands. U11 has only approximately0.101mm nominal toe extension beyond the maximum centered lead-tip envelope; side extension at maximum leadwidth is approximately0.0045mm per side, before positional/placement tolerances. TI's example improves both margins. SC70 toe extension is approximately0.116mm beyond the centered maximum lead-tip envelope; adopting the example raises this to0.350mm. The recommended exact pad positions and sizes are recorded in JSON. Retain numbering and footprint zero-angle when applying them.

U11 zero-angle is rotated180degrees relative to TI's printed top-view drawing; SC70 is rotated90degrees counterclockwise. Neither is mirrored. U13 agrees with TI's top-view land numbering; the package-outline underside view is intentionally mirrored. Do not mistake a bottom-view drawing for a pinmap error. Final CPL/JLC assembly preview still must confirm actual placed pin1 orientation; this report does not authorize an unverified rotation offset.

U13's longer pin1 land is intentional, matching TI's asymmetry. It must not be equalized to the otherfive. Its actual inner/outer land edges give at least the same nominal lead overlap as TI's example; rowgap is0.221mm. Retain this footprint unless a later solder-mask/paste/DRC check supplies a specific reason to change it.

Primary evidence: [TI SN74LVC1G74, DCU0008A drawings](https://www.ti.com/lit/ds/symlink/sn74lvc1g74.pdf), [TI SN74LVC1G98, DCK0006A drawings](https://www.ti.com/lit/ds/symlink/sn74lvc1g98.pdf), [TI TPS3890, DSE0006A drawings](https://www.ti.com/lit/ds/symlink/tps3890.pdf), [ST LDL1117 pin table and SOT223 dimensions](https://www.st.com/resource/en/datasheet/ldl1117.pdf). TI package/land pages were downloaded and visually rendered under `reference_drawings/`. ST download timed out locally; the official PDF's extracted pin table and mechanical dimension table were checked via web reader. No ST-rendered image is claimed.

This is not a solder-stencil, placement, DRC, reflow-process, electrical-function, or manufacturing-release approval. Final source persistence, fresh net/DRC checks, routed pads, copper-to-edge, solder-mask/paste checks and assembly preview remain required.
