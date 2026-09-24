# G - Lid UX / port labelling (r12)

**Verdict: F's mechanical solution stands unchanged (every opening, position, standoff and wall is as F
built it); every label on the lid and on the board silk is replaced.** The owner's "estop 100?" is
explained by F's corner: the buttons carried the net names `EN` and `IO0` on one line, which reads
"EN 100", and the two 3 mm pin holes were punched through the O of `IO0` and the P of `ESTOP`. The
lid was rebuilt in Fusion with the new labels and saved as **`6DOF_2_enclosure_r12b`** (Default
Project); all seven interference checks are still 0.000 cm3. Screenshots `enclosure/shot_r12b_*.png`.

## 1. Motor numbering - decision: `MOTOR 0` .. `MOTOR 5` (firmware M0..M5)

Every surface the operator sees is 0-based; a 1-6 box would be off by one against the screen:

| where | evidence |
|---|---|
| Controller firmware serial protocol | `main.cpp:1089` `FREQTEST:ERR motor=0-5`; `M%d` status lines with `i = 0..5` (`main.cpp:871/987/1389/1521`, `StepDriver.h:164/287`) |
| Desktop app UI | `app/src/ui_panels.cpp:2171/2188` "Motor %d" / "M%d" for `k = 0..5`; `test_harness_panel.cpp:532/580/769/813` `M0..M5` |
| Docs / schematic | `pcb/README.md` pin table M0-M5; schematic section "AXIS 0-5 DB25 DRIVE CONNECTORS" (HANDOFF.md:532); board silk `M0..M5` |
| only 1-based items | the *macro names* `STEP_PIN_1..6`/`DIR_PIN_1..6` in `helpers.h:57-68` (internal; the `stepPins[6]` array is 0-based) and a stale comment in `docs/hardware/build_epro.js:459` ("J2 Motor 1") for the old board |

Mapping (unambiguous, all three columns move together): **J3 = MOTOR 0 = M0 (STEP GPIO4/DIR 10),
J4 = MOTOR 1 = M1 (5/11), J5 = MOTOR 2 = M2 (6/12), J6 = MOTOR 3 = M3 (7/13), J7 = MOTOR 4 = M4 (8/14),
J8 = MOTOR 5 = M5 (9/17)**; U3 drives 0/1, U4 2/3, U5 4/5 (B §4). If the owner prefers 1-6, set
`MOTOR_FIRST = 1` in the script (one line) *and* change the five app/firmware format strings above -
do not do one without the other.

## 2. Lid labels as built (case mm = PCB x + 20; rot 90 reads bottom-to-top; bold Arial, 0.7 deboss)

| serves | label text | cap | rot | centre | measured extent / clearance |
|---|---|---|---|---|---|
| J3 J5 J7 windows | `MOTOR 0` `MOTOR 2` `MOTOR 4` | 5.0 | 90 | (8.5, 35 / 100 / 165) | outboard strip x 6..11, 6.4 from the window |
| J4 J6 J8 windows | `MOTOR 1` `MOTOR 3` `MOTOR 5` | 5.0 | 90 | (66.5, 35 / 100 / 165) | centre rib x 64..69, 4.4 from the window |
| USB-C (front wall) | `USB-C` | 5.0 | 0 | (105.5, 16.0) | x 94.6..116.4, 3.2 from the MOTOR 1 window, 3.4 from RESET |
| J9 12 V screws | `+` `GND` | 3.0 | 0 | (127.0, -0.6) (133.5, -0.6) | front margin under the window, over each screw (127.5 / 132.5) |
| J9 | `12V IN` | 4.0 | 90 | (130.4, 32.5) | column over J9 (124.9..135.8), y 22..43 |
| J2 E-stop | `E-STOP` | 4.0 | 90 | (141.2, 32.5) | column over J2 (135.8..146.7), y 22..43, 6.5 above the SW1 hole |
| J1 console | (none) | - | - | - | removed 2026-09-05 per owner: J1 is a bench header with no external port, so the lid carries no text for it |
| SW1 (141,14) | `RESET` | 4.0 | 0 | (128.5, 14.0) | ends x 137.2 = 2.3 from the hole edge (139.5) |
| SW2 (148.5,14) | `BOOT` | 4.0 | 0 | (159.8, 14.0) | starts x 153.2 = 3.2 from the hole edge (150.0) |
| J10 (right wall) | `ETHERNET` | 5.0 | 90 | (167.5, 37.0) | beside the wall opening (y 28.5..45.6), x 165..170 |
| title | `6DOF 2` | 7.0 | 0 | (135, 187) | unchanged |

Owner feedback (2026-09-05, "too many words") applied as the final state above: the rear header,
the USB hint lines, `LOOP` / `CLOSED = RUN`, the flash recipe and `CONSOLE 3.3V` were dropped; the
three column headers keep their columns with bottoms aligned at y 22. The behaviour text now lives
only on the board silk (§4) and in this report. `6DOF_2_enclosure_r12b` was rebuilt in place and
re-saved; screenshots `shot_r12b_*.png` are of this final state.

Drawing: `enclosure/lid_ux.svg` (to scale, openings red, wall ports dashed blue, groups green,
legend). Its generator also checks every label against every opening (>= 1 mm; >= 2 mm to the
button holes) - the only hit is its width model over-predicting the header; measured on screen the
header ends at x 104. Nothing was moved on the lid except labels; no opening changed.

**Before -> after (F's corner):** `USB` `12V` `EN` `IO0` `ESTOP` `ETH` at 3-3.5 mm regular weight,
`EN`/`IO0` on one line 4.5 mm above the holes, holes through the letters; `M0..M5`; nothing said
what the E-stop does. Now: full words, one scale for port names (5 mm), one for column headers and
button names (4 mm), one for behaviour lines (3.5 / 3.0 mm), E-stop described as a loop with its
rule, USB marked program/bench only, console marked 3.3 V, buttons named for their function with the
flash recipe, polarity on the 12 V screws, and the group header for the motors.

## 3. Grouping, reading order, type scale

- **Motors** (left two thirds): each window's label sits immediately to its LEFT (outboard strip for
  the left column, centre rib for the right column) reading upward along the 57 mm window, numbers
  climbing away from the front wall in the physical order J3..J8; one header along the rear margin
  says what they are and where the cable goes (AASD CN2). No prefix letters, no net names.
- **Service cluster** (front-right): reads left-to-right in the physical order of the wall and
  window - USB-C (wall x 108.5..121.5), 12 V (window 124.9..135.8), E-STOP (135.8..146.7), CONSOLE
  (147.2..154.8), then ETHERNET on the right wall. Column headers stand directly above what they name
  and read upward, so a second line of a group is the next column to the right. Button names sit
  beside their own hole on the hole's row; the recipe stands above the BOOT hole.
- **Scale** (FDM, 0.4 nozzle, lid printed face-down so the deboss is on the bed): 7 title / 5 port
  names / 4 headers and buttons / 3.5 behaviour lines / 3.0 hints only where the strip is < 30 mm.
  Bold sans (Arial Bold) so no stroke is below ~0.6 mm at 3 mm cap; 0.7 mm deep (0.6-0.8 prints
  crisp; 0.5 fills with elephant-foot). Line pitch >= cap + 1 mm; >= 1 mm from any edge, >= 2 mm from
  a hole. Do not go below 3 mm cap or regular weight - F's 3 mm regular `EN`/`IO0` is what failed.

## 4. Board silk (PCB mm, top layer, 6-mil stroke; positions checked against `tools/E_bodies.json`)

| # | string | x | y | rot | mil | clearance / note |
|---|---|---|---|---|---|---|
| 1-6 | `MOTOR n : SERVO DRIVE CN2` (n = 0..5) | 17.0 / 47.0 | 35, 100, 165 | 90 | 70 | replaces `Mn SERVO DRIVE CN2`; box x 16.4..17.6 / 46.4..47.6, ~26 long; nearest body C11 x 24.4 (E: outside the 19 mm hood zones) |
| 7 | `USB-C` | 95.0 | 9.3 | 0 | 70 | J11 top 7.3 (1.1); R2 x <= 88.8 |
| 8 | `PROGRAM &` | 95.0 | 12.5 | 0 | 60 | 90.9..99.1: R2 2.1 W, D7 (100.1) 1.0 E |
| 9 | `BENCH ONLY` | 95.0 | 15.7 | 0 | 60 | 90.5..99.5: R3 1.7 W, F1 (105.86) clear; replaces B's "USB: PROG/BENCH ONLY" and `USB-C PROG / PWR` |
| 10 | `+12V` / `GND` | 107.46 / 112.54 | 9.95 | 0 | 60 | E's E1 row (J9 body top 9.4) |
| 11 | `12 V DC IN` | 110.0 | 12.0 | 0 | 60 | 105.5..114.5: F1 (13.75) 0.85 N, D7 1.6 W; replaces `12V IN` |
| 12 | `E-STOP LOOP` | 121.2 | 9.95 | 0 | 60 | 116.2..126.2 over J2 (top 9.4), J1 body 127.19 1.0 E; replaces the `ESTOP`/`GND` pin pair (the loop has no polarity) |
| 13 | `CLOSED = RUN, OPEN = STOP` | 121.0 | 20.4 | 0 | 60 | 109.75..132.25: C36 (<= 18.4) 1.5 S, R7 (>= 23.0, or A's 22.99) 2.1 N |
| 14 | `RESET` / `BOOT` | 121.0 / 128.5 | 17.6 | 0 | 60 | 1.7 above SW1/SW2 tops (15.5); replaces `RST`/`BOOT` at 30 mil |
| 15 | `UART 3.3V` | 131.1 | 11.6 | 0 | 60 | 127.05..135.15: J2 fp 0.4 W, (140,10) washer 0.65 E - tight, visible; replaces `CONSOLE UART 3V3` (under J2/washer) |
| 16 | `GND` `TX` `RX` | 128.46 / 131.0 / 133.54 | 7.2 | 90 | 60 | vertical (2.54 pitch cannot hold 60-mil text horizontally), y 5.85..8.55, J1 body top 5.27 |
| 17 | `J1` designator | 131.0 | 9.6 | 0 | 50 | between 16 and 15 (E's (131.0, 8.0) now collides with 16) |
| 18 | `ETHERNET` | 134.0 | 50.5 | 0 | 70 | E-verified position |
| 19 | `6DOF 2` (board name, optional) | 100.0 | 185.0 | 0 | 70 | free area: C1 y <= 171, mount (80,190) keepout x <= 86 |

Delete: the four old KF128 pin labels at y 8.13, `USB-C PROG / PWR`, `12V IN`, `CONSOLE UART 3V3`,
`RST`, the old `M0..M5 SERVO DRIVE CN2`. If A's optional D6 move to (96.00, 12.50) is adopted, rows
8-9 move to y 19.0 / 22.2 (the strip x 90..100 is free there once D6 leaves 23.6). Every position
still needs the render check (memory rule); pads/vias under text were not checked here.

## 5. What changed in `enclosure/6dof_enclosure_r12.py` (F's backup: session scratchpad)

- `MOTOR_FIRST`, `DB25_LABELS = ["MOTOR %d" ...]`; window report lines now print `MOTOR n`.
- `TEXT_H/H2/H3`, `TEXT_DEPTH 0.7`, `TEXT_FONT "Arial"`, `TEXT_BOLD`, `CHAR_ADV`, and the whole label
  set as one data table `LID_LABELS` (text, case x, y, cap, rot) with the clearance rationale inline.
- `deboss_labels()` rewritten: rotation via `SketchTextInput.angle` (Fusion rotates about the box
  centre, so rotated text gets a square box centred on the target), `fontName`, `textStyle =
  adsk.fusion.TextStyles.TextStyleBold` (the enum is in `adsk.fusion`, not `adsk.core`).
- Geometry untouched. py_compile OK; three live iterations in Fusion (box centring, bold, USB stack
  order / RESET-BOOT clearance), each verified with top + iso screenshots; final saved as
  `6DOF_2_enclosure_r12b`. F's `6DOF_2_enclosure_r12` is left as it was.
- Lid volume 62.99 -> 62.38 cm3 (bold, deeper deboss); Base unchanged 126.27 cm3.

## 6. Open questions

1. 0-5 vs 1-6 is the owner's call (section 1); the script and the silk table follow 0-5.
2. Wall-face labels: the USB-C / 12 V / E-STOP openings are in the front wall, the RJ45 in the right
   wall; the lid labels stand 6-16 mm behind each opening. If the owner wants text on the wall faces
   as well, that is a second sketch plane in the script (not done - untestable without an iteration).
3. With the hint lines gone from the lid, the E-stop rule (`CLOSED = RUN, OPEN = STOP`), the flash
   recipe and "USB = program/bench only" exist only on the board silk (§4 rows 8-9, 13) and in the
   docs; if a printed quick-reference card is wanted, the removed strings are in git history of
   `6dof_enclosure_r12.py`.
4. `UART 3.3V` on the board sits 0.4 mm from the J2 outline and 0.65 mm from the M4 washer; if the
   integrator moves J2 inboard (F says <= 1 mm is harmless) give this label the room instead.
5. Board silk row 13 assumes R7 stays at y >= 23.0 (A's move keeps it at 22.99).

## 8. r12c (integrator, 2026-09-05)

Owner: J1 has no external port, so the lid carries no `UART` text. Removed from `LID_LABELS`; rebuilt and saved as `6DOF_2_enclosure_r12c` (Lid 62.75 cm3, 15 strings). `shot_r12c_lid_top.png` / `shot_r12c_iso.png`. The service window over the terminal-block screws is unchanged. Board silk `UART` / `GND TX RX` stays for bench use.

Note (integrator, 2026-09-05): the Gerber shows EasyEDA's text size is not the cap height - 60 mil renders 0.90 mm, 70 mil 1.07 mm. All board strings were set to 70 mil for JLC's 1.0 mm minimum.
