# D — Route geometry (r11 baseline → r12 smoother)

**Stream D, 2026-09-05.** Read-only against the board. Tools:
`review_r12/tools/v4_smooth.py` (measure + smoother) and `review_r12/tools/v4_gate.py` (proof).
Outputs in `review_r12/tools/out/`, logs in `review_r12/tools/logs/`.

## Verdict

The complaint is real and measurable: 1,362 45° micro-jogs and 1,995 segments shorter than
12 mil across 144 / 267 of the 883 traces, all of it router staircase (4/8 mil A* steps on
shallow diagonals). The offline smoother removes **every** micro-jog and 97% of the short
segments by replacing 256 traces with the fewest straight chords that pass the exact clearance
checker: **4,480 → 1,705 segments**, 77 mm less copper, all four proof gates green
(`v2_check_plan.py` 0 issues, islands 0 → 0 with identical per-net counts, 256/256 endpoints
unchanged, length −1.01%). The rip + create delta is ready for the integrator
(`tools/out/smooth_rip.json`, `tools/out/smooth_plan.json`) and must be regenerated (5 s) from
the live state after the placement moves of streams A/E, before it is applied.

## Part 1 — measurement (r11, `data/state_r11.json`)

Definitions (per polyline, mil): *segments*; *length*; *turns* = direction changes > 1°;
*short* = segments < 12 mil; *straightness* = end-to-end distance / path length; *jogs* =
micro-jogs, a segment ≤ 8 mil whose two neighbours are parallel to each other and not to it
(the aliasing artefact: `— / —` or `/ _ /`); *non-octilinear* = segments not at 0/45/90°.

### Board totals, before → after

| metric | before (r11) | after smoother | change |
|---|---|---|---|
| traces | 883 | 883 (256 re-shaped, 627 untouched) | — |
| segments | 4,480 | 1,705 | −62% |
| mean segments / trace | 5.07 | 1.93 | |
| direction changes | 3,591 | 820 | −77% |
| segments < 12 mil | 1,995 | 59 | −97% |
| 45° micro-jogs | 1,362 | 0 | −100% |
| traces with ≥ 1 jog | 144 | 0 | |
| traces with ≥ 1 short segment | 267 | 59 | |
| copper length | 301,617 mil = 7,661.1 mm | 298,580 mil = 7,583.9 mm | −3,037 mil (−1.01%) |
| straightness (length-weighted) | 0.9153 | 0.9247 | |
| non-octilinear segments | 198 | 506 | see limits |

### Per layer

| layer | traces | segments | turns | short | jogs | length mm | re-shaped |
|---|---|---|---|---|---|---|---|
| 1 Top | 701 | 2,755 → 1,257 | 2,050 → 555 | 1,108 → 53 | 735 → 0 | 4,479 → 4,437 | 175 |
| 2 Bottom | 172 | 1,697 → 420 | 1,523 → 247 | 887 → 6 | 627 → 0 | 2,918 → 2,883 | 81 |
| 15 Inner1 | 3 | 11 → 11 | 8 → 8 | 0 | 0 | 113 | 0 |
| 16 Inner2 | 7 | 17 → 17 | 10 → 10 | 0 | 0 | 151 | 0 |

The 59 remaining short segments: 42 are in untouched traces (33 single-segment stubs such as
pad-edge-to-via legs, 9 already minimal), 8 in the refused traces below, 9 in re-shaped traces
(8 of them the final ≤ 12 mil leg into a pad centre or via, which the endpoint rule fixes).

### Top-30 worst traces (ranked by jogs, then segments), before → after

| # | net | L | length mil | segments | jogs | straightness | result |
|---|---|---|---|---|---|---|---|
| 856 | M1S_I | 1 | 1,511 → 1,403 | 147 → 6 | 86 → 0 | 0.922 → 0.993 | re-shaped |
| 845 | M3S_I | 1 | 2,076 → 1,963 | 134 → 6 | 71 → 0 | 0.925 → 0.978 | re-shaped |
| 736 | M4D_I | 2 | 3,093 → 2,933 | 142 → 5 | 55 → 0 | 0.925 → 0.975 | re-shaped |
| 714 | U0TXD | 1 | 1,158 → 1,120 | 66 → 7 | 51 → 0 | 0.931 → 0.963 | re-shaped |
| 711 | U0RXD | 1 | 1,260 → 1,220 | 68 → 8 | 50 → 0 | 0.937 → 0.968 | re-shaped |
| 715 | U0TXD | 2 | 1,291 → 1,240 | 59 → 6 | 40 → 0 | 0.939 → 0.977 | re-shaped |
| 661 | SCSN | 2 | 1,398 → 1,347 | 57 → 6 | 31 → 0 | 0.768 → 0.797 | re-shaped |
| 712 | U0RXD | 2 | 1,387 → 1,347 | 56 → 11 | 31 → 0 | 0.858 → 0.884 | re-shaped |
| 758 | M2S_I | 1 | 2,038 → 1,955 | 80 → 6 | 29 → 0 | 0.915 → 0.954 | re-shaped |
| 675 | ESP_IO0 | 1 | 821 → 778 | 59 → 4 | 29 → 0 | 0.926 → 0.979 | re-shaped |
| 877 | M5S_I | 2 | 1,299 → 1,221 | 66 → 4 | 25 → 0 | 0.928 → 0.988 | re-shaped |
| 881 | VIN12 | 2 | 1,381 → 1,322 | 59 → 4 | 24 → 0 | 0.944 → 0.986 | re-shaped |
| 868 | VBUS | 2 | 620 → 574 | 54 → 1 | 23 → 0 | 0.926 → 1.000 | one chord |
| 767 | M5D_I | 2 | 2,011 → 1,972 | 52 → 6 | 23 → 0 | 0.930 → 0.949 | re-shaped |
| 672 | ESP_EN | 2 | 2,265 → 2,240 | 39 → 5 | 22 → 0 | 0.890 → 0.900 | re-shaped |
| 870 | VIN12 | 1 | 1,130 → 1,092 | 36 → 5 | 19 → 0 | 0.955 → 0.988 | re-shaped |
| 804 | M5_ALARM | 1 | 3,106 → 3,063 | 47 → 6 | 18 → 0 | 0.967 → 0.981 | re-shaped |
| 867 | VBUS | 1 | 1,435 → 1,411 | 38 → 7 | 17 → 0 | 0.934 → 0.950 | re-shaped |
| 674 | ESP_IO0 | 2 | 695 → 675 | 30 → 4 | 17 → 0 | 0.958 → 0.986 | re-shaped |
| 869 | VIN12 | 2 | 1,125 → 1,113 | 30 → 6 | 17 → 0 | 0.945 → 0.955 | re-shaped |
| 839 | ESTOP_MCU | 1 | 596 → 556 | 44 → 4 | 16 → 0 | 0.908 → 0.974 | re-shaped |
| 719 | U0TXD | 2 | 1,080 → 1,055 | 38 → 6 | 16 → 0 | 0.808 → 0.828 | re-shaped |
| 681 | ETH_TXN | 1 | 749 → 729 | 28 → 7 | 16 → 0 | 0.872 → 0.895 | re-shaped |
| 859 | VBUS | 1 | 510 → 491 | 25 → 3 | 16 → 0 | 0.929 → 0.963 | re-shaped |
| 735 | M3D_I | 1 | 1,807 → 1,781 | 29 → 7 | 14 → 0 | 0.851 → 0.863 | re-shaped |
| 743 | M0S_I | 1 | 403 → 385 | 26 → 4 | 14 → 0 | 0.925 → 0.968 | re-shaped |
| 836 | VCC3V3 | 2 | 205 → 191 | 22 → 2 | 14 → 0 | 0.928 → 0.998 | re-shaped |
| 862 | VBUS | 2 | 867 → 853 | 21 → 5 | 13 → 0 | 0.907 → 0.921 | re-shaped |
| 806 | M5_ALARM | 1 | 497 → 474 | 29 → 4 | 12 → 0 | 0.884 → 0.927 | re-shaped |
| 750 | M0S_I | 2 | 1,627 → 1,610 | 25 → 2 | 12 → 0 | 0.977 → 0.988 | re-shaped |

`#` is the index into `state_r11.json['tracks']`. Full per-trace rows: `tools/out/smooth_report.json`
(`traces[]`, each with `before`, `after`, `changed`, `reason`, `snapped`).

### Before / after renders (`v3_render.py`, RSCALE 0.3; after = state_minus + plan, plan copper bright yellow = Top, cyan = Bottom)

| area | window mm | before | after |
|---|---|---|---|
| A1 driver S_I / D_I fan-out east of U4/U5 (M1S_I, M2S_I, M3S_I, M5D_I) | x 88..140, y 82..104 | `reports/D-img/A1_driver_SI_before.png` | `reports/D-img/A1_driver_SI_after.png` |
| A2 UART + ESP_IO0/ESP_EN south of U2, USB-C escapes | x 112..136, y 2..34 | `reports/D-img/A2_uart_esp_before.png` | `reports/D-img/A2_uart_esp_after.png` |
| A3 west-centre power + motor runs past U10 (M4D_I, M5S_I, VIN12, VBUS) | x 60..102, y 106..152 | `reports/D-img/A3_west_power_before.png` | `reports/D-img/A3_west_power_after.png` |

In A1 the four parallel 147/134/80/52-segment staircases become 5-6 chords each; in A2 the
UART pair becomes clean straight runs to the USB-C pads; in A3 the M4D_I 142-segment run
across U10 becomes 5 segments. Nothing else in the windows changes (the untouched traces stay
in their thin red/blue rendering).

## Part 2 — how `v4_smooth.py` works

1. **Model.** The state is loaded into a 100-mil spatial index: track segments (per layer, with
   half width and owning track), vias (radius max(d, 24)/2, all layers), pads (`pad_extent`
   rectangles; through-hole on all layers), the six mount circles, and the keepouts the state
   does not contain — J8's two board-lock slots (r 72 at (2452, 5577) and (2462, 7415)), F1's
   under-body strip, the antenna region on layer 12 with the same-net-pad exemption for U2.1/U2.40,
   and the MagJack body rect that only J10's nets may enter. Rules are verbatim copies of
   `v2_check_plan.py` (same `seg_seg` / `seg_rect` functions, `half + half + 6`, via `half + 12 + 6`,
   pad rect `+ half + 6`, keepouts `+ half`) plus a **0.05 mil safety margin** on every rule.
2. **Per trace, fewest chords.** For every polyline with ≥ 3 vertices, the vertices are the
   nodes of a DAG; an edge i→j ("chord") is valid when (a) every original vertex between i and j
   lies within `eps = half width + 6 mil` of the chord, so the new copper stays in the envelope
   the old copper already owned; (b) the chord passes the clearance model against the *current*
   geometry of every other object — already-smoothed traces in their new shape, others in their
   original — which is exactly what the checker sees for `state_minus + plan`; (c) same-net
   topology is preserved: every same-net track / via / pad that touched the original polyline
   (v3_islands touch model) still touches the chord that replaces the segments it touched, and
   no same-net object that did not touch before touches now (so another polyline's end sitting on
   a T-junction vertex pins that vertex, and chords never merge two same-net runs); (d) if an
   endpoint sits on an SMD pad, the new final segment still starts outside the pad. A shortest
   path (segments, then length) over valid chords gives the fewest straight segments; the
   original segments are themselves chords, so the worst case is "unchanged".
3. **Octilinear nudge.** Each interior vertex whose two segments are not both at 0/45/90° is
   moved (≤ eps) to the intersection of octilinear lines from its neighbours when the moved pair
   passes (a)-(d) and the trace does not become longer than its original. 64 vertices took it.
4. **Endpoints never move; width and layer never change.** Traces that come out identical are
   not in the plan (430 single-segment, 174 already minimal).

Run: `python v4_smooth.py [--state F] [--out DIR] [--safety 0.05] [--no-snap] [--eps-extra 0]`;
whole board in ~5 s (76,896 chords tested, 1,780 rejected by clearance). Then
`python v4_gate.py [--state F] [--out DIR]` (~10 s) writes `out/gate_results.json`.

### Limits — what it refused and why

- **23 traces refused** (`smooth_report.json['refused']`, listed with their reasons in the log):
  M0D_N, M0S_I, M0S_N, M0S_P, M1D_P, M1_ALARM_DRV, M2S_N, M2S_P, M2_ALARM_DRV, M3D_P, M4D_N,
  M4S_P, M5D_I, M5D_P, M5_ALARM_DRV, VCC5 ×2, GND ×3, M3D_I, M1D_I, VCC3V3A — all in the
  west DB25 / driver column (J3-J8, U3-U5, U6, U8), all hand-routed before the router existed,
  none with a jog and none longer than 15 segments. In each one an *existing* segment already
  fails the offline pad model (e.g. `M0D_N L2 segment 1: pad J3.18() 0.00 < 10.00`,
  `M0S_I L1: pad U3.5(M0D_N) 5.91 < 10.00`), i.e. the board passes native DRC there but the
  checker's bounding-box pad rectangles and its treatment of no-net DB25 pads as foreign copper
  are stricter. The smoother will not emit copper the checker would reject, so it leaves them.
  Cost: 8 short segments and 0 jogs stay. Fixing them means either loosening the checker's DB25
  pad model or re-routing those 23 by hand; neither is a geometry-stream decision.
- **Chords are arbitrary-angle.** 441 of the 678 plan segments are clean straight diagonals
  rather than 0/45/90° (board total non-octilinear 198 → 506). That is what "fewest straight
  segments" gives on a shallow diagonal; an octilinear-only variant needs one extra vertex per
  shallow run and a dogleg that bulges off the chord by up to (|dx|−|dy|)/2, which the dense
  areas do not have room for. If the owner prefers strict 45/90 over straight, that is a
  different objective and a small addition (`--octilinear`), not done here.
- **No re-routing.** Vertices come from the original polyline (plus ≤ eps nudges), so the tool
  cannot shorten a detour (SCSN stays at straightness 0.80) or change corridors; that is the
  router's job after placement moves.
- **Inner layers**: the 10 Inner1/Inner2 traces are already minimal; untouched.
- Like every tool in the chain it models no component bodies; but because it never leaves the
  ±(half width + 6 mil) envelope of existing copper it cannot create a body clash that r11
  does not already have.

## Gate results (`tools/out/gate_results.json`, log `tools/logs/v4_gate_20260905_074852.log`)

| gate | result |
|---|---|
| 1 `v2_check_plan.py out/state_minus.json out/smooth_plan.json` | plan 256 traces / 678 segments / 0 vias — **ISSUES: 0**, nothing outside the board; tightest margin 0.067 mil over the rule, identical to the tightest margin of the original geometry of the same 256 traces (checked by running the originals as a plan against the same state_minus: 0 issues, tightest 0.067) |
| 2 islands (`v3_islands.Net`, original state vs state_minus + plan) | open connections **0 → 0**; per-net island counts identical for every net |
| 3 endpoints | 256/256 replaced traces: head and tail unchanged to < 0.01 mil; net / layer / width unchanged; 0 zero-length segments |
| 4 length | 301,616.7 → 298,579.6 mil (**−3,037.1 mil, −1.01%**) |
| 5 inventory | 883 state tracks = 627 kept + 256 plan; 256 rip rows, every identity row matches exactly one state track at 0.05 mil |

## Integrator recipe

Precondition: the outputs are valid **only** for the exact state they were computed from
(rip rows are matched by net, layer, point count and endpoints). PLAN step 2 (placement moves +
re-route) changes that state, so:

1. After A/E moves are applied and the board is re-exported (`v2_export_state.js`), run
   `python tools/v4_smooth.py --state <exported state> --out tools/out_r12` then
   `python tools/v4_gate.py --state <exported state> --out tools/out_r12`; require `GATES: PASS`
   and read the refused list (expect the same 23 unless the DB25 column was touched).
2. **Rip**: for every row `[net, layer, npts, [hx,hy], [tx,ty]]` in `out/smooth_rip.json`, delete
   the one track with that net + layer + point count whose first/last points match within
   0.05 mil. Generate it the way `v3_gen_riplan.py` does (inventory guard: 883 tracks / 312 vias
   before, exactly one match per row, 256 deletions, refuse otherwise). No vias and no cover
   fills are involved — the plan adds no vias and every endpoint stays on the same via / pad
   centre, so the existing cover fills remain correct.
3. **Create**: every entry of `out/smooth_plan.json['traces']` (`net`, `layer`, `widthMil`,
   `pointsMil`) through the same primitive path `v3_gen_delta.py` uses, in 40-primitive guarded
   batches (256 traces ≈ 7 batches).
4. Post-apply as in the handoff: `v2_export_state.js` → `v3_open_report.py` 0 →
   `v2_dead_stubs.py` 0/0 → pour rebuild → save/close/reopen DRC 0. Then re-run
   `v4_smooth.py --measure-only` on the new export and confirm jogs = 0, segments ≈ 1,705.
5. If a later re-route (step 4 of PLAN, electrical fixes) adds new router traces, run the
   smoother again on those only (`--nets A,B`); it is idempotent on already-smoothed copper.
