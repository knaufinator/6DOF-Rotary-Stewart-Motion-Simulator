# C — Electrical routing review, r11 baseline (for r12)

Reviewed 2026-09-05 from `data/state_r11.json` (883 tracks, 312 vias, 594 pads) with
`tools/C_*.py` (outputs in `tools/out_*.txt`). Units mm unless stated; widths in mil.
Copper model: 1 oz outer (1.378 mil), 0.5 oz inner, 1.72e-8 ohm.m, via 1.2 mohm. Gerber check:
Top/Bottom have **no copper pours** (0 G36 regions), Inner1 is the film plane, Inner2 carries
7 tracks — so the only GND reference is Inner1, and Bottom traces see it through the 1.065 mm
core (~1.3 mm), not the 0.2 mm prepreg that the 0.22/0.20 and 0.24/0.15 targets assume.

## 1. Verdict

Nothing in r11 is mis-wired and every power trunk is thermally fine (worst IPC-2221 rise 10.6 C
at the assumed 1.5 A on 20 mil). The problems are all *topology*: the buck converter's ground
(U6 pins 4-8, D2, C4) reaches the plane only through 8 mil top tracks 13-23 mm long; the USB ESD
array D6 has no ground via (18.5 mm of 10 mil to the connector shell); the W5500's digital VDD
pin 28 has no cap closer than 22 mm routed (pins 43-45 are PMODE straps, per B); the ESP32's 3V3
arrives by a 155 mm, 4-via, 8-mil-necked detour (137 mV at 0.5 A) with only 100 nF local; and
none of the four "controlled" pairs is actually routed as a pair (median intra-pair gap
0.6-1.2 mm against 0.15-0.20 targets, 12-40 mm of each on Bottom, asymmetric via counts). Two
items are defects (D1, D2), the rest are should-fix; all are fixable in r12 without schematic
change, and the biggest single lever is a placement move (power stage next to J9/J11, notice
to A). Where A already proposed coordinates (C4, C8-C10, C17, D6, R44, R6/R7) I adopt them.

## 2. Tables

### 2.1 Power-net widths vs assumed current (item 1)

| net | I assumed | min w (layer) | widths seen: layer:w = mm | total mm | vias | IPC dT at min w | neck? |
|---|---|---|---|---|---|---|---|
| VIN_J | 1.5 A | 20 (Top) | Top:20 = 10.1 | 10.1 | 0 | 10.6 C | no |
| VIN12 | 1.5 A | 20 (Top/Bot) | Top:20 = 82.9, Bot:20 = 91.6 | 174.5 | 10 | 10.6 C | no |
| VINRAW | 1.5 A | 50 on the U6 path | Top:50 = 175.0, Top:20 = 9.6 (opto-LED feed only), 22-32 = 1.6 | 186.1 | 0 | 2.4 C (50) | 20 mil piece carries ~60 mA (R30-R35), not the buck |
| VCC5 | 1.0 A | 30 on L1->drivers | Top:30 = 293.1, Top:12 = 47.8, Bot:12 = 15.0, 22-24 = 6.7 | 362.6 | 2 | 3.1 C (30); 12 mil pieces = FB sense + D3 feed at <=0.5 A -> 2.5 C | ok |
| VCC3V3 | 0.6 A | 8 (Top/Bot) | Top:8 = 167.3, Top:12 = 190.3, Bot:8 = 79.3, Bot:12 = 68.7 | 505.6 | 22 | 6.0 C | **yes: LDO output leaves U7.2 at 8 mil for 40 mm** |
| VCC3V3A | 0.15 A | 8 (In2) | Top:12 = 64.4, Top:10.6 = 4.3, Bot:12 = 11.0, In2:8 = 14.3 | 94.0 | 6 | 3.9 C | 8 mil In2 piece is the R42/R43 feed, fine |
| VBUS | 0.5 A | 12 (Top) | Top:20 = 89.6, Bot:20 = 116.8, Top:12 = 2.2 (J11 pad exits only) | 208.5 | 15 | 2.0 C | no |
| SWNODE | 1.5 A | 40 (Top) | Top:40 = 11.7 | 11.7 | 0 | 3.4 C | no; D2 5.8 mm, L1 1.6 mm from pin 2 |
| GND (buck return) | 1.5 A pk | 8 (Top) | U6.5-8 -> C4 -> C1 -> via: 23.5 mm of 8 mil; U6.4 -> via 26 mm; D2.2 -> via 23 mm | — | 66 GND vias total | 48 C if 1.5 A DC on 8 mil; ~14 C at the 0.87 A diode average | **see D1** |

IPC-2221: I = k dT^0.44 A^0.725, k = 0.048 external / 0.024 internal, A in mil^2.

### 2.2 Differential pairs (item 2)

| pair | leg | w mil (mm) | layers | vias | length | skew | gap median/min/max (coupled %) | target w/gap | Inner1 void crossing |
|---|---|---|---|---|---|---|---|---|---|
| ETH_TX U8.2/1 -> R42/R43 -> J10.3/4 | ETH_TXP | 8.66 (0.22), 10 near PHY | Top+Bot 6.9 + 4.7 | 4 | 59.94 | 1.68 | 0.80/0.17/21.5 (20 %) | 0.22/0.20 | MagJack 5.3 (own net) |
| | ETH_TXN | 8.66 (0.22), 10 near PHY | Top+Bot 7.2 | 2 | 58.26 | | 0.82/0.17/2.8 (22 %) | | MagJack 4.1 |
| ETH_RX U8.6/5 -> C33/C34 | ETH_RXP | 8.66/10 | Top+Bot 5.7 | 2 | 23.16 | 2.23 | 0.92/0.25/5.1 (14 %) | 0.22/0.20 | none |
| | ETH_RXN | 8.66/10 | Top+Bot 11.2 | 2 | 20.93 | | 1.94/0.25/6.8 (10 %) | | none |
| ETH_JRX C33/C34 -> J10.7/8 | ETH_JRXP | 8.66 | Top | 0 | 14.30 | 1.39 | 0.80/0.29/2.3 (16 %) | 0.22/0.20 | MagJack 3.3 (own net) |
| | ETH_JRXN | 8.66 | Top | 0 | 12.91 | | 0.64/0.29/2.3 (19 %) | | MagJack 3.3 |
| USB J11 -> D6 | USB_DP | 10 (0.25) | Top | 0 | 19.24 | 4.64 | 1.25/0.25/2.5 (10 %) | 0.24/0.15 | none |
| | USB_DN | 10 | Top+Bot 6.1 | 2 | 23.88 | | 1.25/0.25/1.6 (13 %) | | none |
| USB D6 -> U2.14/13 | USB_DP_MCU | 10 | Top+Bot 39.4 | 2 | 70.44 | 1.24 | 0.83/0.31/6.2 (0 %) | 0.24/0.15 | none |
| | USB_DN_MCU | 10 | Top+Bot 52.1 | 4 | 71.68 | | 0.83/0.31/20.2 (0 %) | | none |

"Coupled %" = fraction of the leg with edge gap < 2x target. Ethernet skew total (PHY to jack):
RXP 37.5 vs RXN 33.8 = 3.6 mm = ~22 ps on FR4, 0.3 % of the 8 ns MLT-3 UI — the +-1 mm target is
a balance/EMI target, not a functional one. Widths: Ethernet 0.22 mm matches the recorded
target; USB 0.25 vs 0.24 target (fine). Gaps: no pair meets its gap target anywhere except at
the component pads.

### 2.3 Long power runs (item 8), routed path, per-layer copper

| net | path | I | routed mm | vias | min w | R mohm | drop mV | verdict |
|---|---|---|---|---|---|---|---|---|
| VBUS | J11 B4A9 -> D3.2 | 0.5 A | 224.6 | 14 | 12 (1 mm at pad) | 213 | 107 | too much: USB-only mode gives VCC5 = 5.0 - 0.11 - 0.35 (D3) = ~4.5 V into an AMS1117 (1.1-1.3 V dropout) |
| VBUS | J11 -> D6.5 | 0.5 A | 25.4 | 2 | 12 | 25 | 12 | fine |
| VIN12 | F1.2 -> D1.2 | 1.5 A | 190.5 | 10 | 20 | 181 | 271 | 2.3 % of 12 V, 0.4 W spread along 190 mm: electrically acceptable, but see A-notice |
| VIN_J | J9.1 -> F1.1 | 1.5 A | 10.1 | 0 | 20 | 10 | 15 | fine |
| VINRAW | D1.1 -> U6.1 | 1.5 A | 18.5 (9.5 straight) | 0 | 50 | 7 | 11 | fine; C1 (100 uF) is 28 mm routed from U6.1 |
| VCC5 | L1.2 -> U7.3 | 0.6 A | 18.4 | 0 | 30 | 12 | 7 | fine |
| VCC5 | L1.2 -> U3.4 / U5.4 | 0.1 A | 289 / 144 | 0 | 30 | 219 / 125 | 22 / 13 | fine (drivers tolerate) |
| VCC5 | L1.2 -> D3.1 | 0.5 A | 53.8 | 2 | 12 | 53 | 26 | fine |
| VCC3V3 | U7.2 -> U2.2 | 0.5 A | 155.0 (88 straight) | 4 | 8 | 274 | 137 | **S4**: 3.3 V minus 137 mV minus GND-return drop under Wi-Fi burst |
| VCC3V3 | U7.2 -> U8.28 (via the 43-45 bus) | 0.1 A | 343.5 | 14 | 8 | 620 | 62 | poor topology (S0/S4) |
| VCC3V3 | U7.2 -> L2.1 | 0.15 A | 329.4 | 12 | 8 | 591 | 89 | poor topology (S4) |
| SWNODE | U6.2 -> L1.1 / D2.1 | 1.5 A | 1.6 / 5.8 | 0 | 40 | 1 / 3 | 1 / 4 | fine |

Resistance per mm, 1 oz: 8 mil 2.42 mohm, 12 mil 1.61, 20 mil 0.97, 30 mil 0.64, 50 mil 0.39.

## 3. Findings

### Defects

**D1 — Buck converter ground is a string of 8 mil top tracks, plane 13-23 mm away.**
Evidence: nearest GND via/THT to U6.4 17.9 mm, U6.5-8 13.1-15.6 mm, D2.2 23.1 mm, C4.2 14.4 mm
(C1.2 2.1 mm). Paths: U6.5-8 -> (104.3,160.1) -> C4.2 (106.4,165.1) -> 13.6 mm -> C1.2 -> via
(92.2,167.5): 23.5 mm of 8 mil = 57 mohm. U6.4 -> 22 mm east -> via (130.5,155.9): 26 mm = 63 mohm.
D2.2 (107.8,147.7) -> 23 mm east -> via (130.9,147.7) = 56 mohm. The catch-diode loop (D2 anode ->
plane -> C4/C1 negative) is ~47 mm of 8 mil plus ~40 mm of plane, ~115 mohm, enclosing roughly
20 x 40 mm; at the 0.87 A diode average (1.5 A out, D = 0.42) that is 100 mV of ground shift
between D2 and C4 and a 150 kHz loop antenna. Input cap C4 (100 nF) is 28 mm routed from U6.1;
the 100 uF C1 is 16 mm straight / 28 mm routed.
Remedy (D + A): (a) two 0.3/0.6 GND vias each at D2.2, at the U6.5-8 pad row (x 102.3, y
156-160), at C4.2 and C1.2; (b) a 40 mil Top GND bar joining U6.5-8 -> C4.2 -> D2.2 (D2.2 is
9 mm from U6.8: route the bar along x = 102-103 from y 147 to 165); (c) A: move D2 to abut U6
pins 5-8 (anode pad within 2 mm of U6.8); A's C4 -> (107.90,154.20,0) (2.4 mm from U6.1) is
adopted — its GND pad still needs the via; C1 within 5 mm.

**D2 — USB ESD array D6 has no ground via.** D6.2 (97.0,23.85) reaches the plane only through
18.5 mm of 10 mil Top track to J11's THT shell pad (99.3,6.7). At an IEC 61000-4-2 8 kV edge
(~30 A/ns) 18 mm of 10 mil is ~15 nH -> hundreds of volts of bounce at the array's reference,
which is exactly what it is meant to clamp to. D7 (VBUS TVS) GND pad is 7.6 mm from a via.
Remedy (D): two GND vias within 1 mm of D6.2 (at r11's position (97.0,22.6) and (97.0,25.3)
are clear of the USB_DP/DN pads at x 96.05/97.95; if A's move to (96.00,12.50) is taken, place
them at the new pad the same way) and one at D7.2 (103.6,13.0) + 20 mil stub; keep the track to
J11 as the shield return. A's D6 move also shortens the exposed J11->D6 pair from 19-24 mm to ~7.

### Should-fix

**S0 — W5500 digital VDD (pin 28) has no local decoupling.** Pin 28 (106.4,61.76) goes
3.0 mm to a via at (105.6,59.1) under the body, 12.2 mm west on Bottom at 8 mil, then up to
C31 (10 uF) / C30 (100 nF) at L2's input: 22.3 mm routed, 2 vias, nearest cap 13.3 mm straight.
(Pins 43-45 share that bus but are PMODE strap inputs — B's finding — so they need no cap.)
The core runs on the internal 1.2 V whose C12 is pin-first (2.1 mm), which is why this is not
a defect; the I/O ring (80 MHz SPI, LED drivers) is what pin 28 feeds. C18 (100 nF) and C19
(10 uF) on VCC3V3 at (85.4,44.3)/(85.4,51.5) are orphans — no IC pin within 10 mm.
Remedy (A + D): move C18 to A's slot (109.60,63.30,0) with its VCC3V3 pad toward pin 28
(3.2 mm straight stub on Top, 12 mil) and a GND via at the other pad; C19 goes to U2 (S4).
Keep C30/C31 at L2's input; that placement is right.

**S1 — Ethernet TX pair is not a pair.** 60 mm PHY->jack (R42/R43 pull-ups in line), median
gap 0.80 mm vs 0.20 target, only 20 % coupled; the two 9.4 mm horizontal runs sit 2.54 mm
apart (y 35.10 and 32.56); TXP has 4 vias vs TXN 2 — TXP dives to Bottom alone near the PHY
((99.77,43.09) -> (101.70,46.95), 4.7 mm) while TXN stays Top; both then take a 7 mm Bottom jog
from x 119.5 to 121.3 whose second via pair is inside the MagJack void (x > 121.1). Cause:
ACTLED (Top diagonal (124.4,41.8) -> (112.7,61.5)) and the JRX pair (Top, (117.7,41.5/38.9) ->
J10.7/8) cross the TX corridor. The TX/RX crossing is inherent: PHY TX pins 1-2 are west of RX
pins 5-6 but J10 wants TX (pins 3-4) north of RX (7-8).
Remedy (D, with A): route TX from U8.1/2 to R42/R43 to J10 as 0.22/0.20 on Top; move ACTLED to
Bottom in that corridor; do the one unavoidable crossing as a single <= 3 mm Bottom jog of both
legs together at x ~ 116 (outside the void) with a GND via within 1 mm of each signal via.
Alternative for A: place C33/C34/R44/R45 at y 33-36 so the RX chain runs *south* of TX, removing
the crossing entirely.

**S2 — Ethernet RX: 53 % of RXN on Bottom, skew 3.6 mm end to end, bias resistor on a stub.**
RXN: 11.2 of 20.9 mm on Bottom (reference 1.3 mm away); RXP 5.7 mm. PHY->caps skew 2.23 mm,
caps->jack 1.39 mm, both same sign. R44 hangs on a 7.7 mm stub ((108.33,38.91) -> (102.01,36.58))
while R45 is in line at the RXP path. Remedy (D): Top only, both legs, 0.22/0.20, match to
+-1 mm; put R44 beside R45 at the pair — A's R44 -> (107.84,41.60,180) adopted.

**S3 — USB pair spends 40 mm on Bottom and is uncoupled.** D6->U2: 70.4/71.7 mm, 39-52 mm on
Bottom, median gap 0.83 mm vs 0.15, 0 % coupled; DN_MCU has 4 vias vs DP_MCU 2 (extra detour
(126.7,77.15) -> (123.45,67.0) on Bottom). J11->D6: DN takes 2 vias and 6 mm of Bottom, DP none,
skew 4.64 mm (limit 5.0). Full-speed USB tolerates this; it is a should-fix, not a defect.
Remedy (D): both segments Top only, 0.24/0.15, both legs on the same layer at every point;
J11->D6 straight (the DN Bottom jog avoids VIN12 on Bottom at 0.47 mm — VIN12 is the thing to move).

**S4 — ESP32 3V3 feed: 155 mm, 4 vias, 8 mil at the source, 100 nF local.** U7.2 leaves at
8 mil for 40 mm west to (109.6,148.9), drops to Bottom, comes back at (91.4,148.9) and runs an
84.7 mm 12 mil diagonal NE to (145.4,86.6): 274 mohm, 137 mV at 0.5 A (Wi-Fi TX bursts are
350-500 mA). Local capacitance at U2.2 is C17 = 100 nF; the next capacitor is C7 10 uF at U7.
C3 (100 uF) sits 5.9 mm from U7.2 but is reached through 77 mm of copper and 2 vias.
Remedy (D + A): 20 mil Top from the U7 tab (136.5,158.2) north at x = 137.0 (corridor checked:
only the old 12 mil diagonal crosses it; R18/R19 edge at x 138.25, R22 at y 84.5) to y = 88,
east to x = 141.6, north to C17.1 and U2.2 — ~90 mm, 87 mohm, 44 mV. Move C19 (10 uF, orphan)
to (139.4,87.6) rot 0 with its VCC3V3 pad on that trunk and a GND via; tie C3 to U7.2 with a
1.5 mm 20 mil stub. Delete the 8 mil west run and the 84.7 mm diagonal.

**S5 — U2.2 -> C17 goes through two vias.** 4.55 mm straight, 9.94 mm routed, Top -> Bottom ->
Top, because ESP_EN (U2.3 -> R4/C14) runs diagonally over the direct path. Remedy (D): ESP_EN
from pin 3 goes Bottom (slow net, 2 vias); VCC3V3 U2.2 -> C17.1 straight north on Top, 20 mil,
0 vias — with A's C17 -> (141.62,81.20,90) that stub is 1.8 mm. Give C17 a fresh GND via at the
new position (the existing one at (140.8,84.6) is 3.5 mm from it).

**S6 — AM26LS31 decoupling is 7-18 mm from the pins.** C8/C9/C10 (100 nF) routed 10.5-18.3 mm
from pins 4/16 of U3/U4/U5, C11 (10 uF) 7.1-11.9 mm; cap GND pads 5.6-8.3 mm from any via.
Remedy (A + D): A's C8/C9/C10 -> (29.00, 40.06/105.49/170.92, 90) adopted (2.8 mm north of
pin 16, VCC5 pad toward the pin); each GND pad gets a via within 1 mm; C11 stays. The VCC5
trunk at x ~ 34.4 is re-routed around them (A's note).

**S7 — W5500 analogue VCC pins 4/8/11 share one bus to C28 at 7-9 mm.** Pins 4, 8, 11
(y 53.26) are bussed under the body along y 54.2 (10.6 mil) and reach C28 (111.6,53.1) after
7.3-8.8 mm; pin 4's nearest cap C26 is 17.5 mm routed with 2 vias. Pins 15/17/21 are fine
(2.8-8.7 mm to C28/C29, same layer). Remedy (A): add a 100 nF at (106.4,51.8) rot 90 (between
the RX pair at x <= 105.0 and R10 at 108.8; pad 1 to the pin 8/11 stubs, pad 2 GND with via at
(106.4,50.6)); pin 4 then sees ~5 mm via the bus. C13 stays where it is — it decouples the
R42/R43 pull-up node, which is correct.

**S8 — VCC3V3 runs inside the MagJack Inner1 void.** 33.5 mm (Bottom (121.9,24.3) ->
(121.9,49.7) plus Top stubs), 2 vias inside, because R6/R7 (LED pull-ups) sit at x = 122.1, one
millimetre past the void edge (121.1). The DC net is harmless to the isolation barrier but it
has no reference for 25 mm and it crowds the TX pair's via at (121.31,41.78) (0.2 mm edge gap).
Remedy (A + D): A proposes R6 -> (121.10,23.39), R7 -> (121.10,50.57) (un-swapping them).
With the centre on the void edge the east pad (x 121.85) is still inside it; ask A for
x <= 120.3 (e.g. 120.20) so both pads and the VCC3V3 feed on Bottom stay west of 121.1.
Everything else in the void is J10's own nets or CHASSIS (65 mm) — correct. Antenna keepout:
0 tracks, 0 vias, 0 pads — correct.

**S9 — Crystal circuit is 9 mm from the PHY, XI takes a Bottom detour, SPI at 0.25 mm.**
XI 23.0 mm, 2 vias, 5.4 mm on Bottom ((105.77,62.98) -> (100.83,61.81)) vs XO 11.1 mm Top;
loop hull 49 mm^2; Y1 centre 9.0 mm from pins 30/31. Same-layer neighbours: INTN 0.28 mm from
XI, SCSN 0.25 mm from XO, SCLK 0.75 mm from XO, VCC3V3 0.80 mm. Under it on Bottom: SCSN 9.8,
INTN 6.4, MISO 4.3, M0D_I 4.9 mm (Inner1 continuous there, so tolerable). C23.2 GND is 5.4 mm
from a via, Y1.2 3.2 mm. Remedy (A + D): Y1 within 3 mm of pins 30/31 with XI/XO pads facing
the IC (centre ~(105.15,64.6)), C23/C24 flanking it with GND vias at each GND pad, R58 north of
Y1; XI/XO Top only, 8 mil, <= 6 mm each; SPI/INTN >= 0.6 mm edge gap (3W) and no via inside the
crystal footprint. Moves the ACTLED/LINKLED/MISO/SCSN vias now at y 63.4-63.6.

**S10 — GND via count and pad-to-plane distances.** 66 GND vias on 340 cm^2 (0.19/cm^2), zero
free stitching vias, 62 of 90 20 mm cells empty; 49 of 95 SMD GND pads have no via/THT within
2 mm. Beyond D1/D2: C23.2 5.4, C30/C31 6.7-6.9, C3 7.0, C2 5.1, C16 5.4, C15 4.5, C13 4.4,
C12 4.4, C20 4.9, C29 5.2, C28 3.7, C27 3.1, U8 GND pins 9/19/23/29 2.3-4.0, U7.1 4.4 mm.
With a single reference plane a Top<->Bottom transition does not need a return via (both faces
reference Inner1), so this is about impedance of each pad's return, not layer changes.
Remedy (D): one via <= 1 mm from every decoupling-cap and IC GND pad listed (~30 vias), plus a
10 mm stitching grid in the U2/U8/J10/U6 quadrants and along the board edge.

**S11 — Power-stage placement (notice to A).** VBUS 225 mm / 14 vias / 107 mV and VIN12
190 mm / 10 vias / 271 mV exist only because J9 and J11 are at y 4-5 and U6/D3/U7 at y 150-160.
VIN12 is acceptable as is (2.3 %); VBUS is not comfortable: USB-only bring-up gives the AMS1117
~4.5 V in for ~0.3 A of ESP32 — inside its 1.1-1.3 V dropout band. Moving U6, L1, D1, D2, D3,
C1, C2, C4, C5, U7, C3, C6, C7 to y 15-40 beside J9/J11 removes ~350 mm of 20 mil, 24 vias,
both long-run drops, and fixes D1's geometry at the same time. Alternative: J9 to the south edge.

### Notes

- N1 RS-422 output pairs: all 24 legs 8 mil, no necks, <= 2 vias; skew max 9.4 mm (M2S), then
  4.1 (M3D, M5D), all far under 30 mm. Not routed as pairs: M0D median gap 12.8 mm, M4D 10-11,
  M1S/M3S/M5S 3.4-5.0; only M0S/M2S/M3D/M5D run together for 55-77 % of their length. Cable is
  the antenna anyway; low-priority pair routing for D.
- N2 No RS-422 output runs near Ethernet/USB. MCU-side D/S/ALARM escapes sit 0.45-0.55 mm from
  USB_DP_MCU at the ESP32 pad row (pitch-limited, fine); U0TXD parallels USB_DP_MCU at 0.32 mm
  and ESP_IO0 is 0.70 mm from ETH_TXN — both static/slow, acceptable; S1/S3 re-routes clear them.
- N3 3V3A island: nothing foreign touches its copper on Top (closest INTN 2.8 mm); USB_MCU pair
  (14.6 mm) and SPI (8-14 mm) pass under it on Bottom with Inner1 between — fine. L2.2 -> C20
  5.1 mm same layer, C22 10 uF at 1.4 mm from a GND via. 14.3 mm of the R42/R43 feed on Inner2.
- N4 ETH_TCT: C32/R59 are 12 mm from J10.5 and the net runs 19.6 mm on Inner2; ETH_RX_BIAS
  32.8 mm on Inner2. Move C32/R59/C35 to within 3 mm of J10.5/6 when A touches the jack area.
- N5 XL1509 pin map (1 VIN, 2 SW, 3 FB, 4-8 GND) matches the net assignment. AMS1117 output:
  C7 10 uF ceramic at 4.1 mm (datasheet wants >= 22 uF low-ESR; ceramic 10 uF is on the edge of
  its stability curve) — S4's C3 stub fixes this.
- N6 VCC3V3 via at (141.6,74.4) under the ESP32 body (Bottom diagonal from (129.6,56.7)): legal,
  disappears with S4.
- N7 Thermal: no net exceeds 11 C rise at assumed currents; GND 8 mil pieces only matter in D1.

## 4. What is correct

- Trunk widths: VIN_J/VIN12/VBUS 20 mil (10.6 C at 1.5 A), VINRAW 50 mil, VCC5 30 mil to all
  three drivers (0 vias, 289 mm max), SWNODE 40 mil and 11.7 mm total with L1 1.6 mm and D2
  5.8 mm from pin 2. VBUS's only 12 mil copper is the 1 mm exits from the J11 pads.
- Ethernet trace width 0.22 mm and USB 0.25 mm match the recorded stackup targets; USB skews
  (4.64 / 1.24 mm) inside +-5 mm; Ethernet skews electrically negligible (22 ps).
- Antenna keepout: no tracks, vias or pads on any layer. MagJack void: only jack nets and
  CHASSIS apart from S8.
- Every GND pad is in the one plane island (0 orphans); the VCC3V3A island is a single ferrite
  feed with C20/C21/C22 pin-first from L2 and nothing noisy across it.
- Pin-first caps: U8.22 V1V2 -> C12 2.1 mm, U8.20 TOCAP -> C15 2.9 mm, U8.15 -> C28 2.8 mm,
  L1.2 -> C5 1.0 mm, L2.1 -> C30 3.2 mm, U7.4 -> C7 4.1 mm; C17 GND via 0.85 mm; U2 pad-41
  GND (9 pads) within 2 mm of vias.
- RS-422: no leg longer than 9.4 mm over its partner, all 8 mil, at most 2 vias.

## 5. Proposed changes (summary for the integrator)

| id | to | change |
|---|---|---|
| D1 | D, A | GND vias at D2.2, U6.5-8, C4.2, C1.2; 40 mil GND bar U6 GND -> C4 -> D2; A: D2 adjacent to U6 pins 5-8 (C4 per A's (107.90,154.20)) |
| D2 | D | 2 GND vias at D6.2, 1 at D7.2 (at A's new D6 position if taken) |
| S0 | A, D | C18 -> A's slot (109.60,63.30,0) for U8 pin 28 + GND via |
| S1 | D, A | ETH TX as 0.22/0.20 Top pair, ACTLED to Bottom, one <= 3 mm paired jog at x ~ 116; or A moves C33/C34/R44/R45 south |
| S2 | D, A | ETH RX Top only, +-1 mm; R44 per A (107.84,41.60,180) |
| S3 | D | USB both segments Top only, 0.24/0.15 |
| S4 | D, A | VCC3V3 20 mil U7 tab -> x 137 -> y 88 -> C17 -> U2.2; C19 -> (139.4,87.6); C3 stub to U7.2 |
| S5 | D | ESP_EN to Bottom, U2.2 -> C17 (A's (141.62,81.20)) straight, new GND via at C17 |
| S6 | A, D | C8/C9/C10 per A (29.00, 40.06/105.49/170.92, 90) + GND vias |
| S7 | A | 100 nF at (106.4,51.8) for U8 pins 8/11 |
| S8 | A, D | R6/R7 per A but at x <= 120.3; VCC3V3 at x <= 120.3 |
| S9 | A, D | Y1/C23/C24/R58 within 3 mm of U8.30/31, Top only, GND vias |
| S10 | D | ~30 pad vias + stitching grid |
| S11 | A | power stage next to J9/J11 (or J9 south) |
