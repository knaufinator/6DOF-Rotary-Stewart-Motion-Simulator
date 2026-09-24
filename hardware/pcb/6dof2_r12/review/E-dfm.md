# E — DFM & mechanical review of r11 (for r12)

**Verdict: NOT READY — 5 defects, all fixable without changing function.** Bodies were modelled for
all 123 parts (`tools/E_bodies.json`), the top-silk Gerber was parsed stroke by stroke, and the
copper Gerbers were measured against the mount holes and the outline. The two classes that bit us
before (body-over-body, silk-under-body) are clean except for one repeat: the four KF128 pin labels
sit under the terminal-block bodies. New classes found: 23 via-in-pad, the AMS1117 and XL1509 have
no heat-spreading copper, and every silk label is below JLC's 1.0 mm text minimum.

Method: bodies = nominal package outline incl. leads (0603/0805/1206/SMA/SOD/SOT/SOIC/LQFP) or the
footprint's own silk outline from `Gerber_TopSilkscreenLayer.GTO` (all connectors, U2, L1, C1/C2,
U9/U10). Text/outline strokes clustered from the Gerber (161 text, 208 outline clusters). Mask
openings from `.GTS`; copper from `.GTL/.G1/.G2/.GBL`; drills from the `.DRL` files. Numbers in mm.

## 1. Body model — key facts (full table in `tools/E_bodies.json`)

| part | body rect (x0,y0 .. x1,y1) | h | source / remark |
|---|---|---|---|
| J3/J5/J7 DB25 | 0.13, 8.5/73.5/138.5 .. 12.63, 61.5/126.5/191.5 | 13 | fp silk 12.5 x 53 centred on the pin rows; board-locks (3.05 NPTH) on the row centreline 47.04 apart = **vertical D-sub, mates from +z** |
| J4/J6/J8 DB25 | 56.21 .. 68.71, same y | 13 | as above (SYNC said 55.2..67; Gerber says 56.2..68.7) |
| J9 / J2 KF128 | 104.9..115.1 / 115.9..126.1, y **-1.3 .. 9.4** | 10 | width 10.2 (datasheet); depth from fp silk (datasheet 8.5 would be y 0.5..9.0). Doc layer shows 2x3 wire openings at y -1..2: wire entry faces -y, face overhangs the edge 0..1.3 |
| J11 USB-C | 90.53, -0.09 .. 99.47, 7.30 | 3.2 | fp silk; lip 0.1 past the edge (correct) |
| J10 RJ45 | 124.14, 29.0 .. 149.54, 45.1 | 13.5 | fp silk 25.4 x 16.1; pads at the -x (rear) edge, shield tabs/2.8 pegs at x 144.29 -> **opening faces +x**, face 0.46 inboard. HR961160C nominal length 21.3 would put the face at ~145.5: integrator to confirm from the 3D model |
| U2 ESP32-S3 | 124.90, 61.0 .. **150.40**, 79.0 | 3.1 | fp silk: antenna end 0.4 past the +x edge |
| J1 header | 127.19, 2.73 .. 134.81, 5.27 | 8.5 (housing 17) | fp silk |
| L1 / C1 / C2 | 12.65 sq / 6.75 sq / 6.75 sq | 6.0 / 7.7 / 7.7 | fp silk |
| U9/U10 LTV-847S | 71.29..91.27 x 78.72..88.92 / 131.55..141.75 | 3.6 | fp silk 19.97 long, leads span 10.2 |

## 2. Body-vs-body clearance (every pair < 1.0 mm)

| A | B | gap | heights | verdict |
|---|---|---|---|---|
| R10 | U8 | **-0.40** to lead-span rect; 0.55 to the 7x7 mould; 0.5 to the corner pad | 0.5 / 1.6 | defect (A already proposes (109.60, 52.40, 90): verified 0.97 from U8.13, 0.55 from the mould, 0.78 from C28 — accept) |
| C35 | R59 | 0.47 | 0.9 / 0.5 | should-fix: R59 -> (110.90, 45.90, 0) gives 0.6 (or accept, JLC min 0.25) |
| R12-R13-R14-R15-R16-R25 | 0.47 each | 0.5 | waived by A (on U2 pin pitch) — accept, JLC min 0.25 |
| R20-R21-R22-R23 | 0.47 each | 0.5 | same |
| R58 | Y1 | 0.67 | 0.5 / 0.8 | ok |
| J2 | J9 | 0.80 (fp outlines touch; 10.2-wide bodies) | 10 / 10 | ok, designed to butt |
| C16 | C20 | 0.94 | | ok |
| R42 | R44 | 0.94 (fp silk brackets intrude 0.1 into each other's mask opening) | | ok (A moves R44) |
| J1 | J2 | 1.09 (0.51 to the fp outline) | 8.5 / 10 | note: no finger room west of the DuPont housing |
| L1 | C2 | 1.17 | 6.0 / 7.7 | ok |

Tall-part shadowing: none. No part > 5 mm sits inside any DB25 hood zone, in front of J10/J11,
on the wire side of J9/J2, or within 12 mm above the KF128 screws (tallest part behind J9/J2 within
12 mm: SW1/SW2 2.5 mm, F1 1.1 mm).

## 3. Connector access

| conn | faces | face-to-edge | mating envelope | obstructions | verdict |
|---|---|---|---|---|---|
| J11 USB-C | -y edge | lip at y -0.09 (0.1 overhang) | plug 12.5 x 6.5 outside the edge | none | OK |
| J9 12 V | wires -y, screws +z | face y -1.3 (fp) .. 0 (datasheet) | wire zone off-board; screwdriver from +z | none (SW1 2.5 mm at 3.1 mm north of J2) | OK |
| J2 E-stop | same | same | same | J1 housing 1.1 mm east (not in the screwdriver path) | OK |
| J1 console | +z | 2.73 from y=0 | 1x3 DuPont 7.62 x 2.54 x ~14 | J2 (10 mm) 1.1 west; M4 washer (140,10) 7.0 east | OK, tight |
| J10 RJ45 | +x edge | 0.46 (fp) / ~4.5 (if 21.3 body) | plug + boot 14 x 12 beyond x 150; latch on the PCB side (tab-down) | none; enclosure wall cutout >= 17 (y 28..46) x 14 from the PCB plane | OK; confirm body length |
| J3/J5/J7 | **+z** (vertical) | 20.1 from x=-20 | hood 16 x 56 -> x -1.6..14.4; hood underside ~6.5 above PCB | zone empty; labels at x 16.7 clear (0.8 from a 19-mm hood) | OK |
| J4/J6/J8 | +z | — | x 54.4..70.4 | zone empty; a 19-mm hood covers U9/U10 (3.6 tall < 6.5) | OK |
| SW1/SW2 | +z | bodies y 12.5..15.5 | finger/actuator | J2 3.1 south, J1 housing 7.2 | OK (F: actuators) |

For F: the DB25 plug orientation flips between columns (pin-1 row at x 5.0 on J3/5/7, at x 63.8 on
J4/6/8); the hoods of the left column overhang x -1.6 (inside the board), never the edge.

## 4. Findings

### Defects

**E1 — KF128 pin labels under the terminal-block bodies (repeat of the DB25 class).**
"+12V" (104.0..105.8), "GND" (111.9..113.2), "ESTOP" (117.3..119.6), "GND" (122.9..124.2) all at
y 7.95..8.37, inside J9/J2 (body top y 9.0 datasheet / 9.4 footprint). Invisible once the blocks are
fitted. Remedy in §7 (row at y 9.95). "CONSOLE UART 3V3" also starts at x 125.87, 0.2 inside J2's
fp outline (0.25 above the datasheet body) and its last 0.6 mm (x > 135.5) is under the (140,10)
M4 washer — replace, see §7.

**E2 — Via-in-pad: 23 vias with the hole centre inside an SMD pad** (JLC DFM warning; solder
wicks into the barrel, 0.3 mm hole, vias are tented on the mask so paste sits on a tented hole):
U4.8, R51.2, R2.2, D7.1, U9.9, C13.1, J10.5, J10.6, J10.11, U2.29, U2.31, SW1.1, R20.2, R17.2,
C36.1, R60.2, R43.1, C27.1, R19.1, D3.1, D3.2, C17.1 and U9.13 (ring 0.24 inside). Plus 3 in the
U2 thermal pad (41) — acceptable as thermal vias. Near-misses with ring < 0.1 from the pad copper:
C25.2 (-0.10), C14.2 0.05, SW2.2 0.06, U5.16 0.07, U2.9 0.07, C36.2 0.08, SW1.2 0.08, C17.2 0.09.
Remedy: move each via so the ring edge is >= 0.15 outside the pad copper (0.6-0.8 mm shift along
the exit track); D3.1/D3.2 are D3's only connection (all D3 copper is on Bottom) — route a 0.76 mm
top stub 1.2 mm off each pad, then via. Routing work for D/integrator; list is in `tools/E_dfm_out.txt`.

**E3 — AMS1117 (U7) has no heat-spreading copper.** Tab pad 4 (VCC3V3) is 2.34 x 3.6 = 8.4 mm²
touched only by 0.2 and 0.3 mm tracks; total VCC3V3 track copper within 12 mm = 9.3 mm² top +
1.8 bottom, one via. Dissipation (5 - 3.3) x ~0.6 A (ESP32-S3 Wi-Fi + W5500 + 3 x AM26LS31) ~ 1.0 W,
peaks 1.5 W. SOT-223 on pad-only copper: theta-JA ~ 90 C/W -> Tj ~ 115-125 C at 25 C ambient,
more in an enclosure -> thermal-limit territory. Remedy: a VCC3V3 top pour >= 500 mm² attached to
the tab (fits x 130..146, y 150..166 around C3/C6/C7 after A's C6 move) mirrored on Bottom with
>= 6 vias 0.61/0.3; merge with C's S4 20-mil trunk that leaves the tab. Target theta-JA ~ 50 C/W.

**E4 — XL1509 (U6) GND pins 5-8 have no thermal path.** Only 8-mil tracks touch pins 4-8 and
there is **no GND via within 6 mm** of U6 (nearest plane connection is via C4/C5 elsewhere). The
datasheet uses pins 5-8 as the heat path; at 12->5 V, 0.8 A the loss is ~0.8-1.0 W -> SOIC-8 with
no copper ~ 110 C/W -> Tj > 110 C. Remedy: GND top copper >= 150 mm² under/around pins 5-8
(x 100..104, y 154..162 is free) with >= 4 vias to Inner1; also shortens the D2/C4 return loop (C).

**E5 — Silk text below JLC minimum.** JLC: character height >= 1.0 mm, stroke >= 0.15 mm.
Measured from the Gerber: 25-mil labels cap 0.33, 30-mil 0.42, 40-mil 0.58, all with 0.127 mm
stroke (5 mil) -> all 19 labels fail both limits, incl. the ten new 40-mil labels. Designators:
0.65 cap, 0.152 stroke (height fails). JLC prints anyway but will not guarantee legibility, which
is the point of the labels. Remedy: labels 60-70 mil / 6-mil stroke (cap 0.87-1.0), see §7.

### Should-fix

- **S1 — U2 overhangs the +x edge by 0.4 mm** (antenna end x 150.40; region keepout 142.9..153).
  Either recess (U2 -> (133.0, 70.0), 0.6 inboard, keeps all pin coordinates A relies on shifted
  -1.0 x; A's R12-R16/R17/R20-R25 rows move with it) or hang the antenna properly (>= 5 mm) —
  both are RF decisions; if it stays, order with stamp-hole rails (S2).
- **S2 — Process edges.** JLC adds rails on two opposite edges; y=0 has J9/J2 (-1.3), J11 (-0.1)
  overhanging and x=150 has U2 (-0.4), J10 (0.46), C6 (0.79). No opposite pair is V-cut-able:
  state "process edge with stamp holes, on the x=-20 / y=200 side if only one is needed" in the
  order remarks. C6: accept A's move to (145.55, 160.47) (body 3.6 from the edge; copper 0.49 -> 3.1).
- **S3 — Designators:** J1's "J1" collides with the "GND" pin label (127.2..129.0, 5.8..6.5);
  13 designators are printed on their own part/pads and will be clipped/hidden (R59, C26, C12, C30,
  Y1, R12, R13, R14, R15, R16, R21, R22, R23); U2 and J10 designators are off the board (x 150.2 /
  150.9); U3, U4, U5 (and probably C31) have no designator on the top silk; J3-J8 designators sit
  under their shells (x 2.2 / 66.0). Positions in §7.
- **S4 — Inner1 GND plane is 0.508 from the board edge** at (-19.39, 0.61) (outer copper min is
  0.494, C6 pad). JLC outer limit 0.3, inner 0.5: inset the plane 0.6.
- **S5 — R59/C35 0.47 mm** (row above); **B's label** "USB: PROG/BENCH ONLY" — position in §7.
- **S6 — RJ45 body length** (25.4 fp vs 21.3 datasheet) only moves the enclosure cutout depth;
  confirm from the EasyEDA 3D shell before F freezes the wall.

### Notes (no action)

- Mask dams: LQFP-48 0.13, USB-C 0.0, ESP32 0.17, R59/C35 0.185, C16/C20 0.137 (< 0.2 -> JLC gang-opens; normal).
- 15 vias are 20/12 mil (0.508/0.305, ring 0.10) vs the 24/12 standard; above JLC's 0.05 min.
- Hole-to-other-net copper: 0 tracks < 0.254; hole-to-hole min 0.44 (other-net).
- Footprint silk brackets of R42/R44 and C1 intrude 0.07-0.10 into a mask opening: JLC clips, harmless.
- Pin-1/polarity marks present on the Gerber: U3/U4/U5/U6/U8/U9/U10 (dots), U2 (filled triangle at
  pin 1, x 143.1 y 76.5..77.5), D6 (dot), D1-D3/D7 (bar in outline), D4/D5 (0.5 x 1.0 bar), C1/C2
  (bevel). DB25/J1 pin 1 = square pad; polarity is keyed. Confirm on the JLC render (memory rule).
- A's C25 / R6 / R7 "under the MagJack" used the layer-15 keepout box (121.1..149.5 x 22.9..53.6,
  28 x 31 mm) as the body. The footprint's own outline is 25.4 x 16.1 (y 29.0..45.1): C25 is 8.2
  clear, R6/R7 1.2 clear. A's moves are still harmless (C25 at (145.19, 56.20) is 1.4 below the
  antenna keepout, R6/R7 at x 121.1 are 2.2 from the jack) — accept, but record which body is canon.

## 5. JLC rule check

| rule | JLC | r11 | status |
|---|---|---|---|
| min trace / space | 3.5 / 3.5 mil | 8 mil min width; DRC 0 | ok (space not re-measured) |
| via hole / dia | 0.15 / 0.25 | 0.305 / 0.61 (297), 0.305 / 0.508 (15) | ok |
| via annular ring | 0.05 (0.13 rec.) | 0.152 / 0.10 | ok |
| THT annular ring | >= 0.15 | DB25 0.28, KF128 0.40, J1 0.25, USB-C slots 0.20 | ok |
| drill to other-net copper | 0.254 | none below | ok |
| copper to outline (outer / inner) | 0.3 / 0.5 | 0.494 / 0.508 | ok / at limit (S4) |
| NPTH | no copper | 6 x 4.5 (pad = hole), 12 x 3.05, 2 x 2.8, 2 x 0.6 | ok |
| mount keepout r 6.0 | copper-free | nearest copper 6.00 (Inner1 pour edge), >= 6.68 other layers | ok |
| mask dam | 0.2 | 0 / 0.13 / 0.17 on 0.5-1.27 pitch | gang-open (note) |
| via-in-pad | none | 23 (+3 thermal) | **E2** |
| silk height / stroke | 1.0 / 0.15 | 0.33-0.65 / 0.127-0.152 | **E5** |
| silk over pad | clipped | 16 designators, fp outlines of J10/J9/J2/J11 | legibility (S3) |
| component to edge | >= 1.0 rec. | C6 0.79, J10 0.46, U2 -0.4, J11 -0.1, J9/J2 -1.3 | connectors intended; S1/S2 |
| bottom side | — | no components, 167 THT/via openings only | ok |

## 6. Assembly, thermal, mounting

- THT (9 parts, 157 joints): no THT pad within 2.0 mm of any SMT body; hand/selective solder access clear.
- Panel rail: see S2. Nothing but the intended edge connectors is within 3 mm of an edge after A's C6 move.
- Thermal: U7 and U6 are E3/E4. D1 (0.25 W), D3 (0.3 W, USB-only), D2 (~0.4 W) on SMA pads with
  1.27 mm feeds: theta-JA ~ 100 C/W -> +25-40 K, acceptable. C's report covers trace heating (max 10.6 C).
- Mounting: six 4.5 mm NPTH at (-10,10) (-10,190) (80,10) (80,190) (140,10) (140,190), pad = hole
  (no ring), r 6.0 keepout regions on layer 12 with rules [2,6,5,7,8]; copper-free on all four layers
  (measured). Washer r 4.5: nearest bodies 7.0 (J1 to (140,10)), 7.4 (R2 to (80,10)), 10.1 (DB25s).

## 7. Proposed changes (exact)

| # | item | from | to |
|---|---|---|---|
| E1 | "+12V" | (104.9, 8.13) 30 mil | (107.46, 9.95) 60 mil, 6-mil stroke |
| E1 | "GND" (J9) | (112.52, 8.13) | (112.54, 9.95) 60 mil |
| E1 | "ESTOP" | (118.46, 8.13) | (118.46, 9.95) 60 mil |
| E1 | "GND" (J2) | (123.54, 8.13) | (123.54, 9.95) 60 mil |
| E1/E5 | "12V IN" | (110.0, 10.5) 40 | (110.0, 11.6) 60 mil |
| E1/E5 | "USB-C PROG / PWR" | (95.0, 10.5) 40 | "USB-C PROG/PWR" (96.5, 11.6) 60 mil (R2 1.2 west, D7 0.35 north) |
| E1/E5 | "CONSOLE UART 3V3" | (131.0, 9.5) 40 | "UART 3V3" (131.0, 11.6) 60 mil (J2 1.2 west, washer 0.8 east, SW1 0.5 north) |
| E5 | "GND" "TX" "RX" (J1) | y 6.35, 25 mil | same x, y 6.6, 40 mil |
| E5 | "RST" "BOOT" | y 17.5, 30 mil | same x, y 17.6, 50 mil |
| E5 | "M0..M5 SERVO DRIVE CN2" x6 | 40 mil | 70 mil, same anchors, rot 90 (box x 16.4..17.4 / 46.4..47.4, 19 mm long) |
| E5 | "ETHERNET" | (134.0, 50.5) 40 | same, 70 mil (129.7..138.3) |
| B | new "USB: PROG/BENCH ONLY" | — | (95.5, 14.6) 50 mil |
| S3 | "J1" designator | (128.1, 6.2) | (131.0, 8.0) |
| S3 | "U2" / "J10" designators | off-board | (134.0, 80.2) / (134.3, 47.0) |
| S3 | "U3" "U4" "U5" designators | absent | (34.4, 39.4) (34.4, 104.8) (34.4, 170.3) |
| S3 | R59 C26 C12 C30 Y1 R12-R16 R21-R23 | on own pads | nudge >= 0.3 outside the pad openings (auto-place, then render) |
| E2 | 23 vias | in pads | ring >= 0.15 outside pad copper; D3 via top stubs |
| E3 | U7 tab copper | 8.4 mm² | VCC3V3 pour >= 500 mm² top + bottom, 6 vias |
| E4 | U6 pins 5-8 | 8-mil tracks, 0 vias | GND pour >= 150 mm², 4 vias to Inner1 |
| S1 | U2 | (134.0, 70.0) | (133.0, 70.0) or leave + stamp-hole rails |
| S4 | Inner1 plane inset | 0.508 | 0.6 |
| S5 | R59 | (110.896, 45.72) | (110.90, 45.90) |
| A | C6, R10, C25, R6, R7 | A's moves | verified against bodies: accept |

Every silk position above was checked against §1 bodies and pads offline; the integrator must still
confirm on the re-rendered silk (memory rule: text under a body is invisible).

## 8. Correct as is

USB-C flush; RJ45 faces the edge with nothing in front; terminal-block wire entry at the edge with
screwdriver access clear; all six DB25 hood zones empty; no tall part shadows anything; six NPTH
mounts with copper-free r 6 keepouts on all layers and >= 7 mm washer clearance; THT pads clear of
SMT parts; annular rings, drill-to-copper, hole-to-hole, outer copper-to-edge all inside JLC limits;
polarity/pin-1 marks present; the ten 40-mil labels are outside every body rect (CONSOLE marginal).

## 9. Not checked

Trace-to-trace spacing (trusting native DRC 0); paste layer; real 3D bodies (RJ45 length, KF128
depth); bottom silk (empty); BOM/CPL rotations (other streams).
