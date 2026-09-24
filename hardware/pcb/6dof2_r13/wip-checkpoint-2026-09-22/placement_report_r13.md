# r13 offline placement proposal — checked candidate

Use `placement_plan_r13.json` for the 26 absolute component transforms: 21 new components plus U7, D3, C3, C6 and C7. All other 118 component positions and rotations remain unchanged, including U6 and the complete buck network, all six DB25 connectors, MCU, Ethernet and mechanical mounting pads.

The plan is bound to `09_capture_pcb.json`, SHA-256 `544296c3d0c19c09b4b32294c249162ddd33f73a961985aab1646695dcd0f825`. Coordinates are millimetres, PCB Y upward, positive rotation counterclockwise. Explicit mil values are included. Do not apply the proposal to a different baseline without revalidation.

## Placement decisions

U7 moves to **(130,112), rotation 0**, with its output tab facing left into a reserved VCC3V3 thermal-spreading region. D3, new D8 and the input capacitors sit to the right; output capacitors sit beside/below the tab. This brings the regulator substantially closer to the MCU and USB supply than the old top power group while leaving U6's switching loop unchanged.

Reserve **x110–128.3, y100–124 mm on Top and Bottom** for U7 output copper. Gross area is 439.2 mm² per face. Conservatively subtracting every intersecting component envelope leaves about **828.4 mm² combined**, before trace clearances. This is room reserved for copper, not a claim that the final board already has this thermal area. Final copper must retain at least 500 mm² combined usable area, a broad connection to the output tab and at least six 0.3 mm drill / 0.6 mm diameter thermal vias. Preserve the existing Inner1 GND reference plane. Verify regulator temperature in the untested prototype under maximum actual load.

U11–U14 and their passives sit west of the MCU at x110–122, y61–75 mm, outside the fixed Ethernet/crystal and antenna regions. R63–R65 sit beside each driver's G pin. R69/R70 are at the MCU USB pins, with equal 1.79659 mm pad-centre local branches. Connector locations and mechanics do not move.

## Geometry and local route evidence

`placement_audit_r13.json` independently verifies all 144 component origins, all 648 component-owned pad transforms and 20 required official TI land corrections. Six additional standalone mounting pads are checked separately. It examines **3,393 changed-versus-all component-envelope pairs** continuously; no new pair violates 0.25 mm separation. Minimum separation is **0.299926 mm, C37/U11**. The layer-11 board outline and all existing rectangular/circular regions are respected.

Bodies use the existing r12 Gerber/package estimates transferred from the corresponding old component origins/rotations, unioned with complete native pad envelopes. New bodies use conservative package dimensions. These are continuous bounds, not an anchor-point or occupancy-grid test, but they are not native courtyard geometry.

The plan incorporates the independent sourcing audit's larger TI lands, conservatively rounded **up** to the copied-footprint editor's verified 0.1 mil size grid: U11 **0.85090 × 0.30226 mm (33.5 × 11.9 mil)**, centres x ±1.55 mm; U12/U14 **0.40132 × 0.90170 mm (15.8 × 35.5 mil)**, rows y ±1.10 mm. Numbering and orientation are unchanged. Centres may quantize to 0.0001 mil; the plan records a 0.000003 mm centre-verification tolerance. **Apply these 20 pad corrections to copied library footprints and relink/reload before routing.** Direct PCB pad overrides did not persist in the live editor; source-library editing and relinking are the verified method reported by the root agent. Native pad-size grid rounding is not lost persistence.

The full independent placement and route checks were rerun with these slightly larger persisted sizes: all 3,393 envelope comparisons and all 12 local routes still pass, with no coordinate changes required. C40 remains (118,70.6), rotation 90; its U14 envelope separation is now 0.349076 mm. Overall minimum remains 0.299926 mm at C37/U11.

Twelve critical top-layer routes are provided as exact pad-level sketches. All pass a continuous foreign-pad test using 0.1524 mm clearance plus track half-width, and do not cross unrelated component bodies. New IC supply-decoupling pad distances are 0.951–1.970 mm; local driver-enable pull distances are 1.921–2.023 mm. U7 input/output bypass distances are 2.330/2.230 mm. U13 CT is 1.664 mm from its capacitor. **U12's supply route must first escape upward through (113,66.3), then reach C38; a direct diagonal would cut across adjacent input pin 6.** Fit a nearby GND via to every bypass/CT ground pad and each new IC ground.

`placement_preview_r13.png` was rendered and visually inspected. Gold lines show only the checked local route sketches; they are not a representation of complete routing.

## Required implementation and release checks

- Remove only old U7 thermal islands `ff8566d52dbb3d7a` and `bf841e4a36d24918`, and their positively identified VCC3V3 thermal vias. Preserve U6 GND island `dce1f8915cb5e576` and unrelated copper.
- Rip and reroute affected nets before copper/clearance verification. The candidate checker deliberately makes no assertion about old track collisions after components move.
- Route local decoupling, supervisor sense/CT and USB pin-to-series-resistor links before global routes. Keep sense/CT away from the buck switch node. Route the USB pair together with a continuous reference plane, no branches or antenna keepout intrusion.
- Rebuild ground pours and U7 spreaders, measure usable copper area after exclusions, and verify thermal-via connectivity.
- Run native DRC, connectivity and footprint/pad persistence checks after save, close and reopen. Verify JLC assembly pin-1 orientation and final BOM/CPL correspondence. This offline placement pass is not manufacturing or functional approval.

Reproduce the candidate with `node plan_r13_placement.mjs`, then run `check_r13_placement.py`. The latter independently validates transforms, compares continuous envelopes and emits the route audit and preview SVG.
