# A — Placement & alignment review (r11 baseline → r12)

Stream A, 2026-09-05. Read-only, from `review_r12/data/` (components/pads/state r11, CPL, BOM,
renders). Bodies inferred from footprint names; DB25 and KF128 extents taken from SYNC.md;
MagJack body and antenna keepout as given. All coordinates mm, y up, origin (-20, 0).
Analysis scripts: scratchpad `place_a.py` (pairwise bodies, margins, decoupling, grouping) and
`verify_moves.py` (re-checks every proposed move against bodies, zones, edge, mounts, and lists
the existing Top copper/vias each new body lands on).

## Verdict

The board is placed sensibly at the block level (bottom-edge connector row, DB25 columns,
ESP flush right, power block in a straight row at y 158.17, every M?D/M?S series resistor
sitting exactly on its U2 pin's x/y) and no two *bodies* overlap — with one exception: **C25's
lower pad sits 0.5 mm under the MagJack body** (body overlap 1.07 mm), which is the same
"pads clear, bodies collide" class that JLC caught on J9/F1. Three more items are real
defects by the stated rules: R6/R7 abut the MagJack body at 0.12 mm *and* are swapped
relative to the LED pins they serve (20 mm crossing traces each), C6 is 0.49 mm from the east
board edge, and R10 is 0.25 mm from U8's package body. The larger placement weakness is
decoupling: U8's four VCC3V3 pins have no cap of their net within 10 mm (the five VCC3V3A caps
are on the digital-supply side of the chip), and the three AM26LS31 100 nF caps sit 10-15 mm
from the VCC pins they serve. Eleven moves (P1) fix all of the above; a further seven (P2/P3)
are offered for the integrator to take only if the affected nets are being re-routed anyway.

## Findings

Severity: **defect** = must change for r12; **should-fix** = recommended; **cosmetic** = note only.

### F1 (defect) C25 pad under the MagJack body
C25 (1206, CHASSIS–GND) at (145.19, 54.86, 90): pads span y 52.53..57.20; MagJack body ends at
y 53.60 → CHASSIS pad (145.19, 53.27) is 0.54 mm under the jack, body overlap 1.07 mm. Visible on
`render_E_esp_eth.png` (pad overlaps the hatched jack box). Remedy: rotate horizontal and lift:
**C25 → (145.19, 56.20, 0)**: body (142.85, 55.34)-(147.53, 57.07); 1.74 mm above the jack,
1.93 mm below the antenna keepout (y 59), x max 147.53 < 150 edge. CHASSIS pad to tab J10.14
becomes 10.0 mm (was 7.1) — acceptable for a 1 nF chassis cap. Lands on one GND top track + one
GND via (re-route GND stub only).

### F2 (defect) R6 / R7: 0.12 mm from the MagJack body, and swapped
R6 (LED_G, 122.12, 50.57) and R7 (LED_Y, 122.12, 23.39): body x max 123.28 vs jack body x 123.40
→ 0.12 mm. Worse, J10.12 = LED_G is at y 30.40 and J10.1 = LED_Y at y 43.62, so each resistor
sits next to the *other* LED's pin: LED_G pad → J10.12 = 20.2 mm, LED_Y pad → J10.1 = 20.3 mm,
two traces crossing the whole pin field. Remedy: swap and pull 1 mm west:
**R6 → (121.10, 23.39, 0)**, **R7 → (121.10, 50.57, 0)**: jack gap 1.14 mm; LED pad → pin
7.2 mm each. Nets: LED_G, LED_Y, VCC3V3 stubs; new bodies cross existing VCC3V3 / ESP_IO0 /
U0RXD top tracks (those runs move, not re-plan).

### F3 (defect) C6 0.49 mm from the east board edge
C6 (100 nF VCC5, 148.41, 158.17, 0): body x 147.31..149.51, edge at 150.00 → 0.49 mm (rule ≥ 2 mm
for non-edge parts). It is also 5.7 mm from U7.3 (VCC5 input) which it decouples. Remedy:
**C6 → (145.55, 160.47, 0)**: pads (144.85, 160.47) VCC5 / (146.25, 160.47) GND; U7.3 at
(142.45, 160.47) → 2.4 mm; 0.75 mm from the U7 pad column, 3.35 mm from the edge. Nets: VCC5, GND.

### F4 (defect by rule, small) R10 0.25 mm from U8's package body
R10 (EXRES 12.4 k, 108.78, 52.61, 90): body y max 53.76 vs U8 LQFP body y min 54.01 → 0.25 mm,
and 0.30 mm from pad 12's tip. Remedy: **R10 → (109.60, 52.40, 90)**: 0.69 mm to the package
corner, 0.95 mm to pad 13, 1.10 mm to C28/C29; EXRES pad → U8.10 = 3.1 mm (was 2.4). Nets: EXRES,
GND (three GND top tracks under the new body). This move also frees the strip needed by F6.

### F5 (should-fix, placement + net) U8 VCC3V3 pins have no cap within 10 mm
| U8 pin | net | nearest same-net cap | mm |
|---|---|---|---|
| 43 / 44 / 45 (left column) | VCC3V3 | C31 (pre-ferrite, at L2) | 10.0 / 10.4 / 10.7 |
| 28 (top row) | VCC3V3 | C31 | 13.3 |
| 4 / 8 / 11 (bottom row) | VCC3V3A | C26 / C28 | 6.4 / 5.7 / 4.2 |
| 15 / 17 / 21 (right column) | VCC3V3A | C28 | 2.9 / 3.7 / 5.4 |
| 20 TOCAP | — | C15 | 3.7 |
| 22 V1V2 | — | C12 | 1.8 |

C20/C21 (100 nF) and C22 (10 µF) are **VCC3V3A** caps placed 4.1 mm from the **VCC3V3** pins
43-45 and 8.7 mm from the nearest VCC3V3A pin; C26/C27 (VCC3V3A) sit at (98.6, 51/49) next to
pins 46-48 (NC/NC/GND). The cluster is on the wrong side of the chip for its net.
Remedy (two parts; the first needs stream B/C because it re-nets two caps):
- **Re-net C20, C21 → VCC3V3** (C22 10 µF may stay VCC3V3A as bulk, or go with them — C's call)
  and slide them to the pin column: **C20 → (98.62, 56.19, 180)**, **C21 → (98.62, 58.22, 180)**
  → pads (99.32, y) are 1.6 / 1.9 mm from U8.45 / U8.43. Column shared with C16 (60.25);
  1.13 mm between caps.
- **C26 → (106.10, 50.90, 270)**, **C27 → (107.60, 50.90, 270)** (VCC3V3A pad north): 1.7 mm
  from U8.8 / U8.11; 0.51 mm below the bottom pad tips, 0.60 mm apart, 1.12 mm from R10 (moved).
  C27's new body crosses the EXRES stub (re-routed with R10 anyway).
If C prefers not to re-net, skip the C20/C21 move (it buys nothing on the wrong net) and instead
add a VCC3V3 100 nF at (98.62, 56.19, 180) — same slot. Pin 28 (top row) remains 5+ mm from any
VCC3V3 cap; a slot exists at (109.60, 63.30, 0) if C wants one (0.82 mm from C23, 1.8 mm from the
U8 body).

### F6 (should-fix) AM26LS31 decoupling 10-15 mm from VCC
C8 / C9 / C10 (100 nF VCC5) sit 12 mm north of U3 / U4 / U5, above the row of pull-downs:
VCC pin 16 → cap 6.2 / 10.1 / 10.1 mm (pin 4, the enable tie, 9.8 / 14.9 / 14.9). Pin 16 is the
top-left pin (29.97, y). The slot directly above it, mirroring R49/R53/R57 at x 31.24, is free:
**C8 → (29.00, 40.06, 90)**, **C9 → (29.00, 105.49, 90)**, **C10 → (29.00, 170.92, 90)** (VCC5 pad
south). VCC5 pad → pin 16 = 2.3 mm; 0.71 mm above the SOIC pad row (same as the resistors),
1.36 mm from R49/R53/R57. Nothing under the new bodies. Nets: VCC5 trunk (currently a vertical
run at x ≈ 34.4 joining the three caps) and GND.

### F7 (should-fix) R44 is an 11 mm stub on the RX pair
R44 (49.9 R, ETH_RXN–ETH_RX_BIAS) at (102.76, 35.56) is 0.23 mm end-to-end from R42 (with a 0.6 mm
y offset — misaligned and crowded), 10.9 mm from C34 and 9.2 mm from its partner R45; the RXN
path detours through it (U8.5 → R44 → C34 ≈ 29 mm vs RXP ≈ 15 mm — pair skew, for stream C).
Remedy: **R44 → (107.84, 41.60, 180)** beside R45 (0.72 mm gap; BIAS pads both at x 108.59):
ETH_RXN pad → C34 = 6.0 mm. Lands on two ETH_RXP top tracks (re-route RX pair — coordinate
with C/D).

### F8 (should-fix, P2) U2 3V3 decoupling 4.6 mm
C17 (100 nF VCC3V3) at (141.62, 84.00, 90) → U2.2 (141.62, 78.75) = 4.55 mm. Slot between the
module and the resistor row is free: **C17 → (141.62, 81.20, 90)** → 1.75 mm; 0.60 mm above the
U2 pad row, 0.83 mm west of the antenna keepout (x 142.9), no overlap with R20 (x ≤ 139.51) or
C14 (x ≥ 143.50). Displaces the ESP_EN trace C14 → U2.3 (re-route via x 139.5-141.1 channel).
Note for C: the only cap on U2's 3V3 pin is 100 nF; no bulk (10 µF) within 60 mm.

### F9 (should-fix, P2) Buck input cap 9.5 mm from U6.1
C4 (100 nF VINRAW) at (105.73, 165.10) → U6.1 (108.44, 156.26) = 9.5 mm; C1 (100 µF) = 14 mm.
U6 is rotated so pin 1 is the bottom of the east column, under pins 2-4. Best cheap fix:
**C4 → (107.90, 154.20, 0)**: VINRAW pad (107.20, 154.20) → U6.1 = 2.4 mm; 1.33 mm below the U6
pads, 0.69 mm above D2. The SWNODE run U6.2 → D2 currently passes here (2 segments) and must
go east of the cap through the 1.4 mm channel to L1's pads. C's call whether the 150 kHz buck
needs it.

### F10 (should-fix, P2) USB ESD array 17 mm from the connector
D6 at (97.00, 25.00): 16.9 mm from J11's D+/D- pads; the unprotected stub is the whole run.
Free slot at **D6 → (96.00, 12.50, 0)**: 2.6 mm above the J11 body, 2.3 mm from D7, clear of the
(80, 10) mount (x ≥ 86). Displaces the "USB-C PROG / PWR" silk at (95, 10.5) (stream E) and
the USB_DP/USB_DN pair + USB_*_MCU pair (stream C/D). Requirement 4.1 says "before routing";
this is the cheapest time.

### F11 (cosmetic, P3) R42/R43 sit 18 mm off the TX path
R42/R43 (TX 49.9 R pull-ups) at (100.22, 34.96/32.51): the TX pair leaves U8.1/2 at y 53.3,
dives 18 mm south-west to them, then runs 25 mm north-east to J10.3/4 (y 39.6/40.6). A slot
exists on the direct path: **R42 → (117.90, 46.90, 180)**, **R43 → (117.90, 45.40, 180)**
(0.93 mm from C32, 0.64 mm apart, 4.3 mm from the jack body). Only worth it if C/D re-route the
TX pair; otherwise leave.

### F12 (cosmetic, P3) U9 resistor rows inconsistent with U10
U10's pattern is right: LED resistors R34/R35 3.75 mm below pins 1/3, pull-ups R40/R41 4.26 mm
above pins 16/14. U9 has both rows *below* the chip: R30-R33 (LED, to pins 1/3/5/7 on the bottom
row) 22 mm south; R36-R39 (pull-ups, to pins 16/14/12/10 on the *top* row) 14 mm south. To match
U10: **R30-R33 → y 75.69** (x unchanged), **R36-R39 → y 92.46**. Eight moves, eight alarm nets
plus the VINRAW and VCC3V3 feeds; new bodies cross M0S_I / M?_ALARM top tracks. Shortens the
VINRAW run by 18 mm. Optional.

### F13 (cosmetic) 0603 arrays at 1.27 mm pitch
R12-R16/R25, R20-R23, C35/R59: body gap 0.41 mm by pad-bbox, 0.47 mm by nominal body. Below the
0.5 mm rule but standard JLC-assembled practice; **waived, no change**. Same for C28/C29 (0.59),
R1/R60 (0.69), C30/L2 (0.75), J2/J9 (0.80, the blocks are designed to butt), C30/C31 (0.91),
C13/R45 (0.99), J1/J2 (1.09).

### F14 (cosmetic) mount-keepout margins ≈ 1 mm
J1 header east end 1.02 mm outside the (140, 10) r = 6 keepout; R2 1.02 mm outside (80, 10). An
M4 head/washer (r ≤ 4.5) clears both by > 2.5 mm; accept.

### F15 (cosmetic) small misalignments, no move proposed
C22 1.02 mm west of the C20/C21 column (moot after F5); C13 0.71 mm above the R45 row; D2/D3
0.64 mm y offset; C1/C2 1.4 mm; SW1/SW2 (y 14) vs R1/R60 (y 15); Y1 0.86 mm west of the XI/XO
pin centre while C23/C24 are centred on it. Decoupling column pitches around U8 vary
(3.05 / 2.03 / 2.79 mm) — harmless.

## Proposed changes

Priority: P1 = do for r12; P2 = do if the named nets are being re-routed; P3 = optional.

| P | ref | from (x, y, rot) | to (x, y, rot) | why | nets to re-route |
|---|---|---|---|---|---|
| 1 | C25 | (145.19, 54.86, 90) | (145.19, 56.20, 0) | pad under MagJack body (F1) | CHASSIS, GND (1 via) |
| 1 | R6 | (122.12, 50.57, 0) | (121.10, 23.39, 0) | 0.12 mm to jack; wrong LED pin (F2) | LED_G, VCC3V3; crosses ESP_IO0/U0RXD |
| 1 | R7 | (122.12, 23.39, 0) | (121.10, 50.57, 0) | as R6 (F2) | LED_Y, VCC3V3; crosses ESP_IO0 |
| 1 | C6 | (148.41, 158.17, 0) | (145.55, 160.47, 0) | 0.49 mm from edge; U7 input cap (F3) | VCC5, GND |
| 1 | R10 | (108.78, 52.61, 90) | (109.60, 52.40, 90) | 0.25 mm to U8 body (F4) | EXRES, GND |
| 1 | C26 | (98.62, 51.08, 0) | (106.10, 50.90, 270) | VCC3V3A cap to pins 8/11 (F5) | VCC3V3A, GND |
| 1 | C27 | (98.62, 49.05, 0) | (107.60, 50.90, 270) | as C26 (F5) | VCC3V3A, GND, EXRES |
| 1* | C20 | (96.08, 61.24, 180) | (98.62, 56.19, 180) | VCC3V3 cap at pins 43-45 (F5) — *needs re-net to VCC3V3* | VCC3V3, VCC3V3A, GND |
| 1* | C21 | (96.08, 58.19, 180) | (98.62, 58.22, 180) | as C20 (F5) — *needs re-net* | VCC3V3, VCC3V3A, GND |
| 1 | C8 | (34.42, 46.69, 0) | (29.00, 40.06, 90) | U3 VCC cap 6.2 → 2.3 mm (F6) | VCC5, GND |
| 1 | C9 | (34.42, 112.12, 0) | (29.00, 105.49, 90) | U4 VCC cap 10.1 → 2.3 mm (F6) | VCC5, GND |
| 1 | C10 | (34.42, 177.57, 0) | (29.00, 170.92, 90) | U5 VCC cap 10.1 → 2.3 mm (F6) | VCC5, GND |
| 1 | R44 | (102.76, 35.56, 180) | (107.84, 41.60, 180) | RX stub / skew, 0.23 mm to R42 (F7) | ETH_RXN, ETH_RX_BIAS, ETH_RXP |
| 2 | C17 | (141.62, 84.00, 90) | (141.62, 81.20, 90) | U2 3V3 cap 4.6 → 1.8 mm (F8) | VCC3V3, GND, ESP_EN |
| 2 | C4 | (105.73, 165.10, 0) | (107.90, 154.20, 0) | buck input cap 9.5 → 2.4 mm (F9) | VINRAW, GND, SWNODE |
| 2 | D6 | (97.00, 25.00, 0) | (96.00, 12.50, 0) | ESD array 17 → 4 mm from J11 (F10) | USB_DP, USB_DN, USB_DP_MCU, USB_DN_MCU, VBUS, GND; silk |
| 3 | R42 | (100.22, 34.96, 180) | (117.90, 46.90, 180) | TX pull-up onto the TX path (F11) | ETH_TXP, VCC3V3A, ACTLED |
| 3 | R43 | (100.22, 32.51, 180) | (117.90, 45.40, 180) | as R42 (F11) | ETH_TXN, VCC3V3A, ACTLED (1 GND via) |
| 3 | R30-R33 | (x, 57.40, 0) | (x, 75.69, 0) | match U10 pattern (F12) | M0-3_ALARM_LED, VINRAW |
| 3 | R36-R39 | (x, 74.17, 0) | (x, 92.46, 0) | match U10 pattern (F12) | M0-3_ALARM, VCC3V3 |

Every "to" position was re-checked (`verify_moves.py`): no body pair under 0.50 mm (closest:
C26/C27 0.60, C17/U2 pads 0.60, R42/R43 0.64), no body in the antenna keepout (C17 0.83 mm west
of it, C25 1.93 mm below it) or the MagJack body (R6/R7 1.14 mm), no edge margin under 2 mm,
no mount-keepout margin under 1.5 mm.

## Verified and found fine

- **Body clearance ≥ 1.0 mm for every other neighbouring pair** (full table of the 39 pairs
  under 1.0 mm is in F1-F4, F7, F13; nothing else is closer than 1.0 mm). U2 body to J10 body
  7.4 mm; R5 sits between them with 2.2 / 2.3 mm.
- **Antenna keepout (142.9..153, 59..81)** contains only U2. **MagJack body** contains only C25's
  pad (F1). Six mount keepouts: nothing inside; nearest bodies J1 and R2 at 1.02 mm (F14).
- **Edge connectors**: J11 face overhangs the edge 0.6 mm (correct); J10 face 0.5 mm inboard
  (normal for the HR961160C through a panel — F to confirm wall thickness); J9/J2 wire-entry
  faces at y 0.7 with the board edge as the clear zone; nothing above their bodies (y 9.2) closer
  than F1 at 13.44 (4.2 mm) after the r10 fix; SW1/SW2 3.3 mm above J2; J1 1.09 mm from J2's body.
- **Series resistors on U2 lines all sit on their pin coordinate**: R12-R16/R25 at y = U2.18-22/17
  (±0.01 mm), R20-R24/R17 at x = U2.4-7/12/10 (±0.01 mm). The gaps in the y = 84 row are the alarm
  input pins (8, 9, 11), not misalignment. Do not "tidy" this row.
- Alarm resistor rows R30-R33 / R36-R39 / R34-R35 / R40-R41: pitch 5.08 exact, same x per column.
- Pull-downs R46-R57: each directly under/over its input pin (pins 1, 7, 9, 15), 0.61 mm from the
  SOIC pad row, identical for U3/U4/U5.
- Power block on one line at y 158.17 (D1, U6, L1, C5, C7, U7); C2 220 µF beside L1's output,
  C3 100 µF 2 mm above U7's tab; R18/R19 and D4/D5 paired at y 151.6 / 148.4.
- Crystal: Y1 1.8 mm from U8.30/31, R58 0.63 mm from the crystal body, C23/C24 symmetric on the
  pin pair. RSTN: C16 1.6 mm from U8.37, R11 by L2. V1V2: C12 1.8 mm from U8.22.
- ESP_EN RC (R4/C14) 6 mm from U2.3 and outside the keepout (y > 81); ESP_IO0 pull-up R5 3.1 mm
  from U2.27. SW1/SW2 far from U2 by design (§3 bottom-edge buttons) — not a grouping fault.
- DB25 → AASD pinout, driver allocation, termination, NC pins (V2_REQUIREMENTS §6): untouched.
- Not in my scope but noted for E/F: the west strip x -20..0 holds only two mount holes; the DB25
  shells start at x 0.0.
