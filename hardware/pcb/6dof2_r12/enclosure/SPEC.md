# 6DOF 2 enclosure r12 - dimensioned specification

Built by `6dof_enclosure_r12.py` (Fusion 360 script, every number below is a named parameter at
the top of the file). Fusion document: **`6DOF_2_enclosure_r12`** in project *Default Project*
(created 2026-09-05 through the built-in MCP server; bodies `Base`, `Lid`, plus hidden
reference bodies `PCB_ref`, `Parts_ref`, `Plugs_ref`).

All dimensions mm. **Case coordinates**: origin = the board's lower-left corner
(= PCB (-20, 0)); `case_x = pcb_x + 20`, `case_y = pcb_y`; z = 0 at the underside of the base
floor. Board inputs: `data/components_r11.json`, `data/pads_r11.json` and stream E's measured
body table `tools/E_bodies.json` (Gerber silk outlines).

## 1. Overall

| item | value |
|---|---|
| board | 170 x 200 x 1.6, six M4 mounts (4.5 mm NPTH) at case (10,10) (10,190) (100,10) (100,190) (160,10) (160,190) |
| board-to-wall clearance | 1.0 each side -> cavity 172 x 202 (x -1..171, y -1..201) |
| wall / floor / lid plate | 2.5 / 2.0 / 2.0 |
| outer shell | x -3.5..173.5, y -3.5..203.5 (177 x 207) |
| corner pillars | dia 9.0 centred on the four outer corners (-3.5,-3.5) (173.5,-3.5) (-3.5,203.5) (173.5,203.5); full height; extreme footprint 186 x 216 (x -8..178, y -8..208) |
| antenna bay | +x wall stepped out 4.0 for y 54..86 (inner face x 175, outer x 177.5, y 51.5..88.5); plastic only, no insert/boss within 12 mm |
| z levels | floor top 2.0 / standoff top = board underside 8.0 / board top 9.6 / lid underside = wall top 27.6 / lid top 29.6 |
| clearance above board | 18.0 (tallest parts: MagJack 13.5, DB25 shell 13.0, KF128 10.0) |
| lid lip | 1.5 thick x 3.0 deep ring inside the wall, 0.2 clearance (lip faces at x -0.8 / 170.8, y -0.8 / 200.8), notched around the pillars |
| volumes (Fusion) | Base 126.3 cm3, Lid 63.2 cm3 (about 150 g + 75 g of PLA/PETG at 100 %) |

## 2. Fixings

| item | value |
|---|---|
| board standoffs | 6 x dia 9.0, height 6.0 above the floor, at the six mount holes; blind hole 5.6 dia x 7.0 deep for **M4 x 6 heat-set inserts** (1.0 mm of floor remains); board fixed with **M4 x 8 pan-head screws** through the 4.5 mm board holes |
| lid fixing | 4 x corner pillars, blind hole 4.0 dia x 6.0 deep from the wall top for **M3 heat-set inserts**; lid has 3.4 clearance holes with a 6.4 x 0.8 counterbore; **M3 x 8 pan/button head** |
| why the pillars sit outside the wall | the board fills the cavity to 1 mm of the wall, so a boss inside the corner would clip the board corner (corner-to-pillar-centre distance 4.95 > pillar radius 4.5: no contact, verified 0 interference) |

## 3. Cut-outs

### 3.1 Lid (through the 2.0 plate; z 24.6..29.6 cut)

| # | serves | case x | case y | size | notes |
|---|---|---|---|---|---|
| L1 | J3 DB25 **M0** | 17.43..35.43 | 6.5..63.5 | 18 x 57 | plug hood 56 x 17 + 1.0; centred on the J3 pin field (PCB x 6.426, y 35.0) |
| L2 | J4 DB25 **M1** | 73.41..91.41 | 6.5..63.5 | 18 x 57 | PCB x 62.408 |
| L3 | J5 DB25 **M2** | 17.43..35.43 | 71.5..128.5 | 18 x 57 | y 100.0 |
| L4 | J6 DB25 **M3** | 73.41..91.41 | 71.5..128.5 | 18 x 57 | |
| L5 | J7 DB25 **M4** | 17.43..35.43 | 136.5..193.5 | 18 x 57 | y 165.0 |
| L6 | J8 DB25 **M5** | 73.41..91.41 | 136.5..193.5 | 18 x 57 | |
| L7 | J9/J2 terminal screws + J1 console header | 124.42..156.01 | 2.0..10.5 | 31.6 x 8.5 | service window; KF128 screws are on top of the blocks at y ~4, J1 is a vertical header (dupont housing enters from above) |
| L8 | SW1 (EN) pin hole | centre (141.0, 14.0) | | dia 3.0 | button 2.5 tall, 18 mm below the lid: use a pin / paper clip |
| L9 | SW2 (IO0) pin hole | centre (148.5, 14.0) | | dia 3.0 | |
| L10 | power-stage vents (U6 buck, L1, C1, C2, U7 LDO) | 120..150 | 6 slots 1.6 wide at y 151.75 + n*3.5 (to 169.25) | 30 x 1.6 each | |
| L11 | U9 driver vents | 96..110 | 3 slots at y 79.75, 83.25, 86.75 | 14 x 1.6 | |
| L12 | U10 driver vents | 96..110 | 5 slots at y 129.75 .. 143.75 | 14 x 1.6 | |
| L13 | lid screws | pillar centres | | 3.4 thru + 6.4 x 0.8 c'bore | |

Solid lid ribs: 38.0 between the DB25 columns (35.43..73.41); 8.0 between DB25 rows; 7.5 to the
inner wall at each end (y -1..6.5 and 193.5..201); 4.6 between L2/L4/L6 and the driver vents.
Button holes are 2.0 from the service window edge (rib 14 - 1.5 - 10.5 = 2.0).

**Labels: superseded 2026-09-05 by stream G** - see `reports/G-lid-ux.md` §2 and `enclosure/lid_ux.svg`
(bold Arial, 0.7 deep, `MOTOR 0..5`, `RESET`/`BOOT`, `E-STOP LOOP / CLOSED = RUN`, `12 V DC IN`,
`CONSOLE 3.3V`, `USB-C PROGRAM & BENCH ONLY`, `ETHERNET`); Fusion doc `6DOF_2_enclosure_r12b`.
F's original layout, kept for the record - Labels (debossed 0.6 deep, Fusion default font): `M0`..`M5` 5 mm tall beside each DB25 window
(left column at x 8.2, right column at x 97.9, on the window's y centre); `6DOF 2` 7 mm at
(135, 187); `USB` (115, 14), `12V` (130.4, 14), `EN` (141, 18.5), `IO0` (148.5, 18.5),
`ESTOP` (141.2, 23), `ETH` (165.5, 51), all 3-3.5 mm.

### 3.2 Base, front wall (y = 0 side; wall occupies y -3.5..-1)

| # | serves | case x | z | size | notes |
|---|---|---|---|---|---|
| F1 | J11 USB-C | 108.5..121.5 | 7.2..15.2 | 13 x 8 | plug overmould 12 x 7 + 1; centred on the receptacle axis (PCB x 95, z = board top + 1.6). Receptacle lip is at PCB y -0.1, i.e. 0.9 inside the wall's inner face |
| F2 | J9 12 V + J2 E-stop (KF128-5.08-2P) | 124.12..147.48 | 8.8..20.1 | 23.4 x 11.3 | one opening for both blocks (0.8 rib between them is not worth printing). Block wire faces are at PCB y **-1.3** (they overhang the board edge, per E's silk), so the faces sit 0.3 inside the wall's inner plane, 2.2 behind the outer face; opening = block extent 104.92..126.68 + 0.8 per side, from board top -0.8 to block top +0.5 |

Rib between F1 and F2: 2.6. Wall left above F2: 7.5; above F1: 12.4.

### 3.3 Base, right wall (x = 150 PCB / 170 case; wall occupies x 171..173.5)

| # | serves | case y | z | size | notes |
|---|---|---|---|---|---|
| R1 | J10 RJ45 MagJack | 28.5..45.6 | 9.1..24.1 | 17.1 x 15 | jack body 16.1 x 13.5 + 1; mouth at PCB x 149.54 (silk) faces +x - verified from the pads: the 12 signal pins are on one row at x 124.4 (rear) and the two shield tabs at x 144.3 (front sides), so the opening faces the board edge. Bridge above the opening 3.5 |

### 3.4 Base intake slots (under the board, z 3.5..6.5, 12 x 3 each)

Rear wall (y = 200 side): x 40, 60, 80, 100, 120, 140 (+12). Left wall (x = 0 side): y 30, 60,
90, 120, 150, 180 (+12). Cross-flow: in low at the left/rear, out through the lid vents over
the power stage and drivers.

## 4. PCB -> case coordinate map (mm)

| ref | part | PCB extent (x0..x1, y0..y1) | case extent | height | how the case serves it |
|---|---|---|---|---|---|
| J3/J5/J7 | DB25 left column | 0.13..12.63, y c 35/100/165 +-26.5 | 20.13..32.63 | 13.0 | lid windows L1/L3/L5 |
| J4/J6/J8 | DB25 right column | 56.21..68.71 | 76.21..88.71 | 13.0 | lid windows L2/L4/L6 |
| J10 | RJ45 HR961160C | 124.14..149.54, 29.0..45.1 | 144.14..169.54 | 13.5 | right-wall R1 |
| J11 | USB-C TYPE-C-31-M-12 | 90.53..99.47, -0.1..7.3 | 110.53..119.47 | 3.2 | front-wall F1 |
| J9 | KF128 12 V | 104.92..115.8, -1.3..9.4 | 124.92..135.8 | 10.0 | front-wall F2 (wires), lid L7 (screws) |
| J2 | KF128 E-stop | 115.8..126.68, -1.3..9.4 | 135.8..146.68 | 10.0 | front-wall F2, lid L7 |
| J1 | console header 1x3 | 127.19..134.81, 2.73..5.27 | 147.19..154.81 | 11.5 (pin tips) | lid L7 (top entry) |
| SW1 / SW2 | tactile EN / IO0 | c (121,14) / (128.5,14) | (141,14) / (148.5,14) | 2.5 | lid L8 / L9 |
| U2 | ESP32-S3-WROOM-1 | 124.9..150.4, 61..79 | 144.9..170.4 | 3.1 | antenna bay (wall 4.6 from the antenna tip, plastic) |
| L1 / C1 / C2 | inductor / electrolytics | 111.2..123.9,151.8..164.5 / 93.5..100.2,164.3..171 / 125..131.7,165.7..172.4 | +20 | 6.0 / 7.7 / 7.7 | under vents L10 |
| U9 / U10 | LTV-847S banks | 71.3..91.3, 78.7..88.9 / 131.6..141.8 | 91.3..111.3 | 3.6 | vents L11 / L12 |
| mounts | 6 x 4.5 NPTH | (-10,10) (-10,190) (80,10) (80,190) (140,10) (140,190) | (10,10) (10,190) (100,10) (100,190) (160,10) (160,190) | | standoffs |

## 5. Verification (done live in Fusion, 2026-09-05)

`Parts_ref` (all 123 placements: measured bodies for the connectors/tall parts, 1.5 mm pad-bbox
slabs for the rest, DB25 shells, M4 screw heads) and `Plugs_ref` (six 56 x 17 x 40 DB25 hoods,
RJ45 plug 12 x 11.5 x 30, USB-C overmould 12 x 7 x 30, terminal wire bundle, 4 mm screwdriver
columns over each terminal screw, console housing + cable, button pins) were boolean-intersected
with `Base` and `Lid`: **all seven intersections are 0.000 cm3** (method proven with a 0.2 cm3
probe). Point-containment probes confirm solid wall beside each opening and void inside it.

## 6. Printing

- FDM, PETG or ASA recommended (DB25 hoods lever on the lid). 0.4 nozzle, 3 perimeters, 20-30 % infill.
- **Base**: floor down, no supports. The side openings have flat 13-23 mm bridges; they print
  cleanly at 2.5 mm wall thickness (or add a 45 deg roof if your printer bridges poorly).
- **Lid**: top face down, lip pointing up, no supports; the debossed text is on the bed face and
  comes out crisp. The DB25 windows and vents are plain through-holes.
- Bed: base 186 x 216, lid 186 x 216 - fits a 220 bed diagonally-free only on 250+ beds
  (Voron 300/350 fine).
- Heat-set inserts: 6 x M4 x 6 in the standoffs, 4 x M3 x 5.7 in the pillars; install with a
  soldering iron before fitting the board.

## 7. Assumptions to confirm (see reports/F-enclosure.md)

DB25 plug hood 56 x 17 (drives the lid windows); RJ45 jack face position (silk 149.54 vs
datasheet ~145.5 - the wall opening covers both); KF128 screw position over the pin row (drives
the service window's y 2..10.5); lid screw style; strain relief for six vertical DB25 cables.
