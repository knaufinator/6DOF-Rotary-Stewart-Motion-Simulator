# FIELD_VGOOD: retaining the intended 12 V input

> Supporting design study. For the ordered 26 October revision, use
> [current decisions](DECISIONS.md), [review method](REVIEW_METHOD.md) and
> [hardware-test plan](BRINGUP.md). Dated proposals and open-status statements
> below retain their original context.

**Offline follow-up, 2026-10-17. No CAD change.** This supersedes the earlier 12 kΩ/1.5 kΩ proposal as a proposed solution for a 12 V ±5% input. That divider's high trip corner requires more protected-rail voltage than the existing input chain establishes. The prior file remains useful as a leakage/threshold calculation, not a selected design.

## Findings

1. The present 100 kΩ/15 kΩ nominal 9.522 V monitor still needs correction: it cannot establish 10 V at the collector.
2. The exact 12 V input chain loses appreciable voltage in D1 and F1. Those parts' available specifications do not give a guaranteed operating drop over the complete temperature/load envelope.
3. At J9 = 11.4 V, an improved monitor can be feasible with a tightly specified cable, current, temperature, and startup envelope. A resistor-only adjustment to the existing comparator cannot provide a general guarantee over the present unknown conditions.
4. At some allowed-looking combinations of total board load and field startup/fault load, the remote voltage is below 10 V even with an ideal monitor. The correct response there is inhibit/stop, not a lower threshold or an assertion that startup is guaranteed.

## Exact source and actual parts

Accepted archive `R14-B5ETHG1-refilled-saved-reopened-HOLD-1792242719318.epro2`, SHA256 `d83197ff86072c6bd6fac1b54dc165efd88877494d078b363fe7f7e3435bfa5b`.

`J9.1 → VIN_J → F1.1/F1.2 → VIN12 → D1.2/D1.1 → VINRAW → U17.1/U17.10 → FIELD_12V → J12.2`.

F1 and D1 carry **total board input current**, including the buck regulator. U17 carries only the field branch. The current through each cannot be substituted for the other.

| Part | Exact source identity | Applicable evidence | Limitation |
|---|---|---|---|
| F1 | BHFUSE BSMD1812-110-16V, C883148 | 0.045 Ω minimum initial resistance; **0.250 Ω maximum at 25°C one hour after trip or reflow**; 1.10 A hold at 25°C | 0.250 Ω is not a maximum while hot and operating near hold current; hold current is temperature dependent |
| D1 | **MDD** SS34, C8678, SMA | **0.55 V maximum at 3 A, 25°C** | Applying this at smaller currents uses monotonic diode behavior; it does not establish a cold-temperature bound or the actual board drop |
| U17 | TI TPS26620DRCR, C1848361 in source inventory | RON maximum 0.521 Ω at 25°C; 0.685 Ω at 85°C; 0.800 Ω across full specified temperature range | Table conditions and actual junction temperature apply; overload current limit is not normal load current |

Sources: [BHFUSE manufacturer specification, pp.1 and 3](https://atta.szlcsc.com/upload/public/pdf/source/20230721/D27B1B216F4F90D863936C428BEED739.pdf), [MDD SS32–SS3200 manufacturer datasheet](https://www.microdiode.com/uploadfiles/PDF/SS32-THRU-SS3200-SMA.pdf), [exact MDD JLC identity](https://jlcpcb.com/partdetail/SS34/C8678), [TI TPS2662 datasheet](https://www.ti.com/lit/ds/symlink/tps2662.pdf). Do not substitute a different manufacturer's generic SS34 specification.

### Actual routed copper screen

The read-only extractor found connected paths on the accepted source and charged each traversed trace its full length/width resistance, including via barrels. Nominal assumptions are copper resistivity 1.724e−8 Ωm, outer copper 35 µm, inner copper 17.5 µm, and via barrel plating 25 µm. Pads/fills are treated equipotential. This is a reproducible conservative path-length screen, not a field solver or guaranteed manufactured maximum.

| Path | Nominal 20°C resistance |
|---|---:|
| J9.1 → F1.1 | 0.002576 Ω |
| F1.2 → D1.2 | 0.013542 Ω |
| D1.1 → U17.1 | 0.040994 Ω |
| U17.10 → R80.1 monitor tap | 0.056521 Ω |
| R80.1 monitor tap → J12.2 | 0.180010 Ω |

The last two paths share some copper; they must not be interpreted as a unique series path decomposition. Charging both at aggregate field current intentionally overestimates that part of drop for this screen. The monitor-to-jack path matters because the comparator does not sense the remote connector directly. Ground-plane return, J9 terminal resistance, upstream supply leads, copper manufacturing tolerance, and ripple remain outside these numbers.

## Practical 12.0 V and 11.4 V screens

The following uses the explicit 25°C component limits above, nominal copper adjusted to 25°C, and a required 1.5 Ω maximum complete hot cable loop. Total board current is an **unmeasured scenario variable**, not a newly established load limit. Field steady current is the prior 170.524 mA conservative load screen plus 0.785 mA incremental controller-side monitor-divider load. The 257.257 mA case is the previously documented conditional eFuse table screen; it includes the entire field branch, with full current also conservatively assigned to the cable.

| J9 input | Total board current | Field case | Voltage at monitor tap | Remote voltage screen |
|---:|---:|---|---:|---:|
| 12.0 V | 0.30 A | Steady | 11.258 V | 10.971 V |
| 12.0 V | 0.50 A | Steady | 11.197 V | 10.910 V |
| 12.0 V | 0.50 A | Conditional limit | 11.147 V | 10.714 V |
| 11.4 V | 0.30 A | Steady | 10.658 V | 10.371 V |
| 11.4 V | 0.50 A | Steady | 10.597 V | 10.310 V |
| 11.4 V | 0.30 A | Conditional limit | 10.609 V | 10.176 V |
| 11.4 V | 0.50 A | Conditional limit | 10.547 V | 10.114 V |
| 11.4 V | 1.10 A | Conditional limit | 10.362 V | **9.929 V** |

These are conditional calculations, not guaranteed voltages. The 1.10 A row is especially not an assertion of acceptable hot F1 operation. At the last corner no electrical monitor can simultaneously declare usable power and assure a 10 V remote sensor supply, because the supply path itself is insufficient. A higher-resistance 28 AWG example makes the margin worse; its construction cannot be assumed for the owner's cable.

At 1.5 Ω cable resistance, a monitor's minimum falling threshold must be at least **10.287 V** for the steady screen or **10.433 V** for the conditional-limit screen, including the conservative monitor-to-jack copper allowance. The latter limit must remain above the remote-supply minimum during load increases, even if normal startup finishes at the smaller current.

For J9 = 11.4 V and total input current 0.50 A, the available steady threshold window is only **10.287–10.597 V**; allowing the conditional-limit case contracts it to approximately **10.433–10.547 V**. These windows have no extra reserve for the omitted losses, ripple, temperature, and response time.

### Why the existing reference is inadequate at the low corner

Even with a perfect comparator, zero hysteresis, zero resistor error, and no leakage, the TLV3012B's 1.223–1.260 V reference spread scales a required 10.433 V minimum threshold into at least a **10.749 V maximum threshold**. That is above the calculated 10.547–10.609 V monitor supply in the conditional-limit low-input cases. At 0.50 A total/steady current it also consumes essentially the entire available window before any real comparator error is added. This is an explicit impossibility for those screening corners, not merely a nominal-divider preference. [TI B-version reference limits](https://www.ti.com/lit/gpn/TLV3012B).

## Options that preserve a nominal 12 V supply

| Option | What changes | Feasibility and cost |
|---|---|---|
| **Qualified nominal-12 V prototype envelope** | Keep U18; lower-impedance precision divider/clamp correction; specify loaded J9 minimum tighter than 11.4 V, cable, temperature and total input current | Least PCB change, but it does **not** satisfy a general 12 V ±5% requirement. Trip values must be selected only after the actual loaded lower rail is established. The earlier 12 kΩ proposal is too high. |
| **Per-board threshold qualification** | Measure both thresholds at temperature and select precision resistor values for each of five boards | Can reduce lot-to-lot reference spread for those boards, but needs documented calibration/rework and drift limits. It does not resolve physical undervoltage or unspecified clamp leakage. Not a preferred automatically assembled, interchangeable production solution. |
| **Replace monitor with a precision supervisor** | New U18 footprint/pin mapping and pull-up; accurate divider; clamp/protection review | Potentially viable at 11.4 V with a tighter cable/current envelope. Needs maximum specifications including hysteresis and bias, not typical headline accuracy. No reviewed drop-in candidate yet. |
| **Reduce upstream loss while retaining 12 V** | Reviewed ideal-diode/reverse-polarity stage in place of D1, and possibly a better qualified fuse/path | Recovers up to much of the present diode-drop budget. This is a power-stage design change requiring owner review, fault/reverse-polarity validation and new layout. It may be more effective than forcing an exceptionally narrow monitor window. |
| **Lower-voltage compatible NC sensors** | Replace the selected 10–30 V sensor with a physically and electrically validated lower-voltage NC model | Removes the most restrictive 10 V floor while retaining the collector. This changes the sensor choice and needs owner review and actual manufacturer compatibility proof. No substitute has been selected. |

### Supervisor candidates checked, without claiming a solved replacement

- **TPS389001DSER / LCSC C1509297** is a concrete direction: 1% maximum threshold accuracy, 0.825% maximum hysteresis, WSON-6. A 12.1 kΩ/1.5 kΩ divider and the proposed 2.2 kΩ/5.6 V clamp give a conservative **10.301 V minimum falling / 10.672 V maximum rising** screen at room-temperature resistor tolerance and 1 µA clamp leakage; the full-temperature resistor/2 µA clamp case widens to **10.255–10.752 V**. It therefore still fails the broad 1.5 Ω/257 mA low-input requirement. It could support a reduced current or tighter-cable envelope, but requires more margin than the near-equalities in the table. It needs a new footprint and output pull-up; JLC allocation was not established. [TI specifications](https://www.ti.com/lit/ds/symlink/tps3890.pdf), [exact LCSC identity](https://www.lcsc.com/product-detail/C1509297.html).
- **TPS3700** is not an automatic improvement: its minimum falling threshold is 387 mV and maximum rising threshold 404 mV. Their ratio alone is approximately 1.0439 before divider/leakage effects, too broad for the tightest low-input window. [TI TPS3700](https://www.ti.com/lit/ds/symlink/tps3700.pdf).
- **TPS3899DL01DSER / C2864821** has a 5% typical hysteresis headline, which is already about 0.52 V referred to a 10.4 V threshold. Do not select it based only on “0.5%” typical accuracy. [TI part](https://www.ti.com/product/TPS3899/part-details/TPS3899DL01DSER), [JLC identity](https://jlcpcb.com/partdetail/TexasInstruments-TPS3899DL01DSER/C2864821).
- **TPS3842A011DRLR** offers a high-voltage SENSE input that can avoid the present low-voltage clamp problem, but its actual maximum threshold error is ±1.5% and its 1% hysteresis option extends to 1.5%. It is not a 0.5%-maximum monitor and does not solve the narrowest window. [TI electrical table](https://www.ti.com/lit/ds/symlink/tps3842.pdf).

A different monitor is justified only after specifying its required complete falling-to-rising envelope. At the 11.4 V / 0.50 A / 1.5 Ω / conditional-limit screen, that envelope is less than approximately 114 mV before reserves—roughly 1.1% of the sensed rail **including both threshold extremes, hysteresis, divider tolerance and leakage**. A conventional “±1% accurate” supervisor cannot meet that merely by changing the divider.

## Recommended next decision preparation

Preserve the intended 12 V supply requirement. First establish maximum total input current and the permitted ambient range, then qualify D1/F1 operating drop at that load, the actual 15 ft complete copper cable loop, and the sensor startup current. Record voltages at J9, VINRAW, U17 OUT, monitor tap, and remote collector. The minimum acceptable loaded supply can then be computed without the earlier illustrative 0.70 V upstream assumption.

If 11.4 V normal startup is mandatory across the broad current/temperature range, prepare a combined low-loss input/protection and precision-monitor option for owner review. If the requirement is nominal 12 V prototype operation with automatic undervoltage inhibit at its low tolerance corner, a qualified narrower envelope and less invasive monitor correction may be acceptable, but that is a distinct requirement and must be stated plainly. No CAD change or approval request is made here.

Prototype-order approval is not loaded-machine commissioning approval. An assembled board can be evaluated under a documented supply/cable envelope; it must not be labeled as guaranteeing 10 V at the collector for every unspecified 12 V source and Ethernet cable.

## Artifacts

- `checkpoints/R14-FIELD-INPUT-COPPER-PATH-RESISTANCE-SCREEN-20261017.json`: SHA256 `143c865427726504de25c96ebd602516b12f304e4811056f7a0efbf8327ba67b`.
- `checkpoints/R14-FIELD-MONITOR-TAP-RESISTANCE-SCREEN-20261017.json`: SHA256 `3fa62a50bd6700fde8668660589dc8fe0221e377600497370511363ded00c919`.
- `checkpoints/R14-12V-FIELD-MONITOR-FEASIBILITY-V2-20261017.json`: SHA256 `a0d40f77397da62b577ea59e92608550ce454f4952f4523a00b701d5b065a0d2`.
- Reproduction scripts: `tools/inspect_r14_field_input_resistance_20261017.py`, `tools/inspect_r14_field_monitor_taps_20261017.py`, `tools/review_r14_12v_field_feasibility_20261017.py`.

The earlier non-V2 feasibility JSON is a superseded intermediate: it added monitor current above the conditional total eFuse-limit screen. V2 correctly keeps total conditional field current at 257.257 mA.
