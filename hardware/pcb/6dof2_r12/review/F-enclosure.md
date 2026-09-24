# F - Enclosure report (r12)

**Path taken: LIVE.** Fusion's built-in MCP (:27182) was down at launch (both endpoints refused);
the script was written offline first, then the endpoint came up and the model was built,
inspected, iterated and saved in Fusion as **`6DOF_2_enclosure_r12`** (project *Default Project*).
The Claude MCP client had already failed for this session, so the server was driven directly over
its HTTP JSON-RPC interface (same `fusion_mcp_execute`/`fusion_mcp_read` tools; a 60-line client
in the session scratchpad). To rebuild from scratch: Fusion > Utilities > Scripts & Add-Ins >
Scripts > "+" > `review_r12/enclosure/6dof_enclosure_r12.py` > Run (creates a new document).

Deliverables: `enclosure/6dof_enclosure_r12.py` (parametric script, passes
`validate_fusion_script.py` and py_compile), `enclosure/SPEC.md` (every dimension and cut-out),
screenshots `enclosure/shot_r12_*.png` (with reference bodies) and `shot_r12_case_*.png`
(case only), Fusion notes `G:/My Drive/projects/fusion/6dof-enclosure/NOTES.md`.

## What was designed

Two-part FDM case, base tray + flat lid with a locating lip, for the 170 x 200 board.

- **Top access** for the six DB25s: six 18 x 57 lid windows sized for a 56 x 17 plug hood + 1,
  centred on each connector's pin field; 38 mm solid rib between the two columns, 8 mm between
  rows, 7.5 mm to the walls. Windows labelled M0-M5 (J3 J4 J5 J6 J7 J8), debossed.
- **Side access** through the y = 0 wall: USB-C 13 x 8 (overmould-sized) and one 23.4 x 11.3
  opening for the two KF128 wire faces (J9 12 V, J2 E-stop). Through the x = 150 wall: RJ45
  17.1 x 15. The RJ45 orientation was verified from the pads - signal row at x 124.4, shield
  tabs at x 144.3, so the mouth faces +x (the board edge), silk face at x 149.54.
- **Terminal screws and console header**: the KF128 screws are on top of the 10 mm blocks and the
  lid is 18 mm above the board, so a screwdriver cannot reach them with the lid on; the console
  header J1 is a *vertical* 1x3 and cannot be side-accessed at all. Both get one **lid service
  window** 31.6 x 8.5 at case x 124.4..156.0, y 2..10.5. Chosen over a taller lid (would need
  +12 mm of height for a driver and still not solve J1) and over "remove the lid" (E-stop wiring
  should not require opening the motor-cable side).
- Buttons SW1/SW2: 3 mm pin holes in the lid at (141,14) and (148.5,14), labelled EN / IO0.
- Ventilation: 14 lid slots over the buck/LDO/inductor/caps and the two LTV-847S banks; 12
  low intake slots on the left and rear walls under the board.
- Antenna: the +x wall steps out 4 mm beside the ESP32 (y 54..86) - E measured the antenna tip
  at PCB x 150.4, i.e. 0.4 mm *past* the board edge, so without the bay the wall would sit 0.6 mm
  from it. No insert, boss or screw within 12 mm of the antenna; nearest metal is the RJ45 shield.
- Fixings: six dia-9 standoffs with M4 heat-set inserts (board on M4 x 8 screws through its 4.5
  holes); lid on four dia-9 corner pillars with M3 inserts. Pillars sit on the outer corners
  because the board fills the cavity to 1 mm - an inside boss would clip the board corner.
- Walls 2.5, floor 2.0, lid 2.0 (+3.0 lip), 18 mm above the board, outer 177 x 207 (186 x 216
  over the pillars), 29.6 tall assembled.

## What was verified in Fusion

Reference bodies were built in the same document from the board data: the PCB slab with its six
holes, every placement as a body (E's measured extents for connectors and tall parts, pad-bbox
slabs for the rest), the DB25 shells and screw heads, and the *mated* plugs and cables (DB25
hoods 56 x 17 x 40, RJ45 plug, USB-C overmould, terminal wires, a 4 mm screwdriver column over
each terminal screw, console housing + cable, button pins). Base and Lid were boolean-intersected
with each of them: **all seven volumes 0.000 cm3** (the method was proven with a deliberate probe
returning the expected 0.2 cm3), plus point-containment probes at each opening. Screenshots:
`shot_r12_iso-top-right.png` (hoods, plugs and cables passing through the openings),
`shot_r12_top.png`, `shot_r12_front.png`, `shot_r12_right.png`, and the case-only
`shot_r12_case_iso.png` / `_top` / `_front` / `_right`.

Iterations: (1) first build - all clear; (2) label layout - ESTOP/EN/IO0 overlapped, "ETH" sat on
the lid edge; re-laid out; (3) E's body table landed - KF128 faces overhang the board edge by 1.3
mm, RJ45 mouth at 149.54, USB lip at -0.1, DB25 shells 13.0 tall, antenna tip 150.4 - inputs
updated, terminal opening widened/lowered to 23.4 x 11.3, rebuilt in place, re-verified, saved.

## Open questions for the owner

1. **DB25 plug hood size** - 56 x 17 assumed (typical plastic hood). Metal hoods or thumbscrew
   knobs wider than 17 mm need the `DB25_HOOD_W` parameter raised; the 38 mm centre rib has room.
2. **Lid screw style** - four M3 heat-set + pan heads at the corners as modelled; alternatives are
   countersunk (needs a 3 mm lid) or six screws (add pillars mid-length on the long walls).
3. **Cable strain relief** - six DB25 cables leave vertically 45 mm above the lid. A cable bar or
   a taller shroud around the windows may be wanted; not modelled.
4. **Service window vs removable access** - confirm the 31.6 x 8.5 window over the terminal
   screws/console header is acceptable (it exposes 12 V and E-stop terminals on the top face);
   the alternative is a snap-in blank or a wider lid margin.
5. **RJ45 face position** - E flags the silk (25.4 long, face at 149.54) vs the datasheet (21.3,
   face ~145.5). The wall opening covers both; if the face is 4 mm back, the latch is reachable
   but recessed 7.5 mm from the wall outer face.
6. **Status LEDs D4/D5** (PCB x 144..146, y 148..152, near the LDO) have no lid window. Add two 2
   mm light-pipe holes at case (165, 149) / (165, 152.5) if they should be visible.
7. **Antenna bay** - keep the 4 mm step (modelled) or drop it (`ANT_BAY = 0`) for a flush wall
   0.6 mm from the antenna tip.
8. **Standoff bosses** press a 9 mm ring around each mount hole on the bottom side; the bottom
   copper there should be masked (it is a signal layer) - a check for E / the integrator.

## Proposed changes (board)

None required for the case. Notices logged in SYNC.md: the KF128 overhang, the vertical console
header (side access impossible without a right-angle part), and the antenna tip past the edge.
