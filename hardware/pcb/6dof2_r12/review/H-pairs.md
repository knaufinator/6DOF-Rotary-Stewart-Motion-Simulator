# H — Differential-pair router (ETH TX/RX, USB, USB_MCU) for r12

**Stream H, 2026-09-05.** Read-only against the board. Baseline `data/state_prepairs2.json`
(880 tracks / 328 vias, the coordinator's re-export after the via-in-pad rework). Tools:
`tools/h_common.py` (exact clearance model = `v4_smooth` index with `v2_check_plan` semantics +
0.05 mil safety), `tools/h_pairs.py` (builds the routes), `tools/h_gate.py` (gates + metrics +
renders). Outputs in `tools/out_pairs/`. Units mm unless stated.

## Verdict

All four pairs are now routed **as pairs** — both legs on the same layer at every point, coupled
at the target gap, length-matched, every leg entering its pads outside-in and ending on pad/via
centres — and the plan passes the exact checker at **0 issues** against the baseline with the rip
applied (`state_minus.json`), with every one of the eight nets a single island. Two targets could
not be met literally and are stated with the reason: (1) the USB gap is **0.155 mm** (6.1 mil),
not 0.15 mm, because 0.15 mm = 5.9 mil is below the board's 6 mil clearance rule that the checker
enforces between the two legs; (2) "Top only" holds for the ETH RX pair and for 79 % of USB_MCU,
but the Top layer is sealed between y 54 and y 60 (no single-trace Top path D6→U2 exists — the
flood-fill in §3 proves it), so USB_MCU has one **paired** 13 mm Bottom section plus a paired 6–9 mm
Bottom hop at D6 (D6.5's VBUS Top trunk walls both MCU pads), and ETH TX has one paired 11 mm
Bottom crossing under JRX/C33/C34/ETH_TCT. Every layer change is a via pair ≤ 1 mm apart, both
legs together, with a GND via beside it (6 GND vias added). The plan also needs **two settled
traces moved** (M0_ALARM / M1_ALARM Top runs at U2's NW corner, §4) — without that there is no
Top approach to U2.13/14 at all; both are re-routed in the same plan and gated with it.

## 1. Per-pair results (`tools/out_pairs/pairs_report.json`)

| pair | leg | w mm | length mm | skew (target) | vias | Bottom mm | gap median / min (target) | coupled % | Inner1 void crossing |
|---|---|---|---|---|---|---|---|---|---|
| ETH_TX U8.1/2 → R42/R43 taps → J10.3/4 | TXP | 0.22 | 65.10 | −0.62 (±1) | 2 | 11.0 | 0.200 / 0.200 (0.20) | 91 | MagJack 8.3 (own net) |
| | TXN | 0.22 | 65.72 | | 2 | 11.4 | | | MagJack 9.0 (own net) |
| ETH_RX U8.5/6 → R44/R45 taps → C34/C33 | RXP | 0.22 | 19.26 | −0.84 (±1) | 0 | 0 | 0.280 / 0.200 (0.20) | 65 | none |
| | RXN | 0.22 | 20.10 | | 0 | 0 | | | none |
| USB J11 (2 pads each) → D6.1/D6.3 | DP | 0.24 | 4.85 | −4.34 (≤5) | 0 | 0 | 0.566 / 0.260 (0.155) | 11 | none |
| | DN | 0.24 | 9.19 | | 2 | 3.2 | | | none |
| USB_MCU D6.6/D6.4 → U2.14/13 | DP | 0.24 | 100.88 | −2.68 (≤5) | 4 | 20.4 | 0.155 / 0.155 (0.155) | 83 | none |
| | DN | 0.24 | 103.56 | | 4 | 24.6 | | | none |

"coupled %" = fraction of the P leg with edge gap < 2× target on the same layer (C's definition);
"gap median" is the edge gap sampled every 0.1 mm. Tap stubs: R42.2 0.76, R43.2 0.99, R44.2 1.20,
R45.2 0.79 mm (all ≤ 1.5). Pad entries: every final segment starts outside the pad and ends on its
centre (23 pad endpoints checked, 0 exceptions); 32 trace ends land exactly on via centres; 0
zero-length segments; 160 segments in 32 polylines (34 non-octilinear chords, no micro-jogs).
Antenna keepout: 0 mm of copper. Plan: 32 traces, 22 vias (ETH_TXP 2, ETH_TXN 2, USB_DN 2,
USB_DP_MCU 4, USB_DN_MCU 4, M0/M1_ALARM 1 each, GND 6).

Compared with r11 (C §2.2): ETH TX 0.80 → 0.20 mm median gap, 20 → 91 % coupled, 4/2 → 2/2 vias,
skew 1.68 → 0.62; RX 0.92/1.94 → 0.28 median, 2/2 → 0/0 vias, 11 mm Bottom → 0; USB_MCU 0 → 83 %
coupled, 39–52 mm Bottom → 20–25 mm, 2/4 → 4/4 vias but now always as a pair.

## 2. Gates (`tools/h_gate.py`, all PASS)

| gate | result |
|---|---|
| `v2_check_plan.py out_pairs/state_minus.json out_pairs/pairs_plan.json` | 32 traces / 160 segments / 22 vias, 613,924 checks, **ISSUES: 0**, nothing outside the board; tightest margin 0.101 mil (the USB legs against each other at 6.1 mil) |
| islands (`v3_islands.Net`, baseline vs state_minus + plan, every net) | open connections 9 → 2; the 2 are **ACTLED, LINKLED** (ripped by the integrator, routed after this plan); all eight pair nets + M0_ALARM + M1_ALARM + GND = 1 island; no net worse than baseline |
| rip identity (0.05 mil, net/layer/npts/head/tail against `state_prepairs2.json`) | 43 trace rows, 19 via rows, every row matches exactly one primitive (includes the two dead USB_DN_MCU vias at (4860.3,2637.5) / (4988.3,3037.5) mil) |
| pad-entry / via-end rules | 0 exceptions (see §1) |

## 3. Method

1. **Model.** `state_minus` = baseline minus every track/via of the eight nets minus the three
   settled M0/M1_ALARM primitives of §4. `h_common.Model` wraps `v4_smooth.Smoother`'s spatial
   index and reuses its verbatim copy of the checker's distance functions (`seg_seg`, `seg_rect`,
   pad rectangles, via r 12 + 6, keepout circles/rects, MagJack rule) plus a via rule mirroring
   `v2_check_plan` (incl. "no via inside its own SMD pad"). Every segment and via is checked as it
   is added, so the plan is proven segment by segment before the official checker runs.
2. **Ends by hand, corridors by A\*.** The constrained ends (0.5 mm-pitch U8 pins, the R42/R43 and
   R44/R45 taps, the J10 pocket, the USB-C mirrored pads, D6, U2.13/14) are explicit leg polylines.
   The long USB_MCU corridor is routed as ONE fat trace of width 2w+gap+0.03 (the pair envelope) on a
   4 mil `v2_router` grid restricted to one layer, reduced to the fewest chords whose envelope
   passes the model, then offset ±(w+gap)/2 with mitred joins to get the two legs.
3. **Paired layer changes** (`transition()`): both legs open from ±d to ±0.31 mm over 0.4 mm, vias
   staggered ±0.35 mm along the run (0.92 mm apart), so each leg's copper clears the other leg's
   via by 0.62 mm; a GND via is placed beside each pair. At corners (ETH TX) the same spread is
   built into the leg geometry.
4. **Length matching**: legs are compared and the short leg gets one chamfered bump (TXP 0.6 mm
   on its y 48.1 run; USB DP 1.3 mm east of the D6 hop; USB DP trombone 1.1 mm at J11).
5. **Topology facts found on the way** (all measured, not assumed):
   - The TX/RX crossing is inherent (PHY pin order vs J10), and after the via-in-pad rework the
     new ETH_TCT Top route (C32.1 → (120.49, 39.45..40.98) → J10.5) walls pins 3/4 from the west, so
     TX enters the J10.3/4 pocket from the north through the 0.82 mm slot between ETH_TCT's vertical
     and the ACTLED pad (legs at x 121.20 / 121.62, 0.45 / 0.25 mm clear). TX therefore crosses
     JRX, C33/C34 and TCT once, on Bottom, x 111.2–113.3, y 37.6 → 48.3.
   - The Inner2 ETH_RX_BIAS diagonal (109.1,43.7) → (127.4,37.5) and the GND stub/via at
     (110.49, 42.57) leave no via spot in the y 42–44 corridor, which is why the crossing lands
     north of C29's GND stub at y 47.7–48.3.
   - R44/R45 taps: heading south the RXN leg is west and its tap (R44.2) is south of RXP's — the
     only crossing-free topology is RXP passing between the two .2 pads (y 42.39) and dipping
     through the 0.69 mm R44.2/R44.1 gap, RXN passing under R44 (y 40.40); both taps are ≤ 1.2 mm.
   - USB-C: the mirrored pad order B7 A6 A7 B6 (DN DP DN DP) cannot be bridged on Top alone with
     the Bottom under the pads taken by VBUS (y 8.08); DN bridges B7–A7 south of the pads, DP
     bridges A6–B6 north of them, and DN hops 3.2 mm on Bottom north-east of CC2's Bottom
     diagonal to reach D6.3 (2 vias). Skew 4.3 mm is the price (limit 5).
   - USB_MCU pin order flips between D6 (DP west) and U2 (DP west but approached heading east) —
     an inherent crossover; it is folded into the D6 hop (DN's Bottom leg passes under DP's Top
     leg), so no extra vias.
   - Top flood-fill from D6 with the pair envelope stops at y 54–59 (x 93–105) and at y 41–45
     (x 104–119); from U2 it stops at y 44–47. Even a 10 mil single trace cannot cross. The
     shortest Bottom bridge is y 45.5 → 49.9 between x 99 and x 113 (north of the TX crossing).

## 4. Settled copper this plan moves (integrator decision)

U2.13/14 sit behind a via forest: M0_ALARM Top (104.85,81.28)→(122.94,78.64) + via, M1_ALARM Top
(106.27,81.28)→(124.36,79.86) + via (124.36,79.86), M3 via (126.39,80.67), M2 via (127.41,81.28),
and pin 15's pad leave a single 0.36 mm band — one trace, never a pair — and M0/M1 must cross
M5D_I (Bottom, y 79.93) on Top somewhere. The plan rips 3 primitives + 1 via (`extra: true` rows
in `pairs_rip.json`) and re-creates: M0 Top (104.85,81.28)→(107.49,78.64), via, Bottom lane y 78.64
to the kept via (122.94,78.64); M1 Top (106.27,81.28)→(108.30,79.25), via, Bottom lane y 79.25
merged into its existing Bottom polyline (re-created minus its first segment so no hanging tail).
Both nets stay one island; the pair then runs Top along y 79.365/79.80 into pins 14/13. If the
integrator refuses this, the alternative is r11's topology (DN_MCU 17 mm alone on Bottom).

## 5. Not met / caveats

- USB gap 0.155 mm instead of 0.15 (6 mil rule). USB J11→D6 is 11 % coupled (pad geometry).
- ETH RX 65 % coupled: the first 7 mm run at the 0.5 mm pin pitch (gap 0.28), and the tap→cap
  section is set by the 2.54 mm C33/C34 pitch.
- ETH TX Bottom crossing is 11 mm, not ≤ 3 mm: the wall it must cross (JRX + C33/C34 + the new TCT
  route) spans y 38.5–47; USB_MCU has 20–25 mm on Bottom (D6 hop + the y 54–60 seal), always paired.
- ACTLED (J10.2) must now escape between the TX legs' slot (x ≤ 121.7) and the jack: use a via in
  the void between pins 1 and 2, e.g. (123.5, 42.9), then Bottom — the Top west of the pad is taken.
- The dead ETH_TCT via (112.80,46.99) + stub (112.80,45.72→46.99) left by the rework was treated
  as an obstacle; it is dead copper for `v2_dead_stubs.py`.
- Outputs are valid only for `state_prepairs2.json`; re-run `h_pairs.py` then `h_gate.py` (25 s)
  after any further copper change before applying.

## 6. Integrator recipe

1. Rip: every row of `tools/out_pairs/pairs_rip.json` by identity (`v3_gen_riplan.py`; guards
   880 tracks / 328 vias). 43 traces + 19 vias incl. the 4 `extra` M0/M1_ALARM rows.
2. Create: `tools/out_pairs/pairs_plan.json` (32 traces, 22 vias, cover fills on the vias as usual).
3. Then ACTLED/LINKLED, then D's smoother (`--nets` on these only if wanted — the plan has no jogs).
4. Renders: `tools/out_pairs/render_{eth,usb}_{before,after}.png` (RSCALE 0.35).
