# r13 preparatory sourcing and compatibility

Prepared 2026-09-22 EDT (catalog captures after midnight are dated 2026-09-23 UTC).
Scope: hardware revision preparation, not a final manufacturing BOM. All required
parts remain REQUIRED and fully assembled; no DNI/hand-install workaround is approved.
No order, reservation, supplier contact, or EasyEDA write was performed by this task.

## Selected implementation shortlist (supersedes exploratory candidates below)

The root implementation now freezes: one TI SN74LVC1G74DCUR C70285; two TI
SN74LVC1G98DCKR C485081; one TI TPS389001DSER C1509297; U7 LDL1117S33R
C435835; D8 MDD SS34 C8678; C6 1 uF C15849; C15 and new C42 4.7 uF C1779;
C33/C34 C1631; added 100 nF bypass/timing capacitors reuse C14663; R69/R70
33 ohm C23140. The precision monitor divider uses Yageo **RT0603BRD0732KL
C861325 (32.0 kOhm)** and **RT0603BRD0710KL C95204 (10.0 kOhm)**, both
0.1%, 25 ppm/C, 0.1 W, 0603. These divider parts are REQUIRED and must not be
merged with ordinary 1% 10 kOhm pull-ups. Fresh indicators were 9,656/9,627 and
1,768,570/1,106,298 respectively (gross/allocation indicator), min purchase one.

The [Yageo RT family datasheet](https://yageogroup.com/content/datasheet/asset/file/pyu-rt_1-to-0-01_rohs_l)
defines B tolerance and D TCR suffixes; the
[exact 10 kOhm specification](https://yageogroup.com/component-documentation/download/specsheet/RT0603BRD0710KL)
confirms 1.6 x 0.8 x 0.45 mm body. Include differential TCR and voltage-reference
tolerance in the supervisor threshold proof. An exact 32 kOhm single-part PDF
did not load, but its identical family/size/tolerance/TCR and resistance-code
decoding are verified against the manufacturer series data.

The additional Schmitt buffer avoids relying on a slow open-drain supervisor
edge at the DFF clear pin. The Nexperia DFF, TPS3808 supervisor, ordinary NOR,
680 ohm and 1.5 kOhm alternatives in the research trail below are **not selected**.
The provisional final board population is 144; only a fresh exact source/BOM/CPL
comparison can confirm it. Candidate stock files include only changed/new SKUs,
not the entire 144-part board. Run the complete-BOM screen after implementation.

## Approved nominal replacements

| Reference | Exact selection | Compatibility assessment |
|---|---|---|
| C15; new C42 | Samsung CL21A475KAQNNNE / C1779 | 4.7 uF, 10%, X5R, 25 V, 0805; nonpolar two-terminal. At C15, same nominal value/dielectric/package as the existing Walsin 4.7 uF X5R 10 V part, with greater voltage rating. C42 is a new local LOGIC_5V/U7 input capacitor. Body 2.00 x 1.25 x 1.25 mm, each +/-0.15 mm. Preserve C0805 pads and inspect body clearances. |
| C33,C34 | FH 0603B682K500NT / C1631 | 6.8 nF, 10%, X7R, 50 V, 0603; nonpolar two-terminal. Same coupling function, capacitance/tolerance/dielectric/package; replaces Samsung 25 V X7R type with higher rating. Body 1.60 x 0.80 x 0.80 mm, each +/-0.10 mm. Preserve C0603 pads. |
| R46-R57 (subject to final inhibit contract) | UNI-ROYAL 0603WAF1001T5E / C21190 | 1 kOhm, 1%, 0.1 W, 0603. Geometry-compatible value change, NOT an equivalent electrical substitution. LS-TTL idle-input calculation and MCU sink/source load belong to the inhibit design proof. At 3.6 V, dissipation is 13.1 mW using 990 ohm minimum. |
| U7 | ST LDL1117S33R / C435835 | 3.3 V low-dropout SOT223. Pin 1 GND, pin 2 and tab VOUT, pin 3 VIN match the existing fixed 1117 topology. Must verify actual placed numbering/tab, merged metadata and thermal copper. This substitution is coupled to the revised two-source power architecture; do not change BOM only. |

C15 evidence: [Samsung exact series page](https://product.samsungsem.com/mlcc/CL21A475KAQNNN.do)
and [JLC exact code](https://jlcpcb.com/partdetail/C1779). Samsung's embedded typical
25 C / 1 kHz / 1 Vrms graph gives approximately 4.66 uF at 1.2 V and 3.94 uF at
3.3 V; `C1779_manufacturer_bias.json` contains the extracted curve and interpolation.
These are typical, not guaranteed minima. W5500's TOCAP pin requires a nominal
4.7 uF capacitor and a short connection; its datasheet does not specify TOCAP as
the 1.2 V rail. Do not confuse TOCAP pin 20 with 1V2O pin 22. The same-class 25 V
substitution is acceptable for the prototype; first-article TOCAP stability and
Ethernet testing remain required. [W5500 datasheet, page 8](https://docs.wiznet.io/img/products/w5500/W5500_ds_v110e.pdf)

C33/C34 evidence: [FH exact manufacturer page](https://www.fhcomp.com/zh-cn/product_info/?prod_code=0603B682K500NT)
and [JLC C1631](https://jlcpcb.com/partdetail/C1631). FH specifies X7R temperature
range -55 to 125 C and +/-15% temperature characteristic in its general MLCC
datasheet linked on that page. An exact part DC-bias curve was not captured; no
claim of zero bias dependence is made. The 50 V rating has ample margin for this
low-voltage PHY-side coupling role, not for Ethernet isolation/surge protection.
The existing required chassis capacitor C25 is unchanged and must not be merged
with either ordinary capacitor type.

## Power-part checks and limitations

[ST LDL1117 datasheet](https://www.st.com/resource/en/datasheet/ldl1117.pdf),
pages 3 and 5-7: pinout above; 600 mV maximum dropout at 1.2 A across the specified
temperature range; +/-3% output tolerance, 15 mV maximum load regulation; at least
1 uF input capacitance and ceramic output capacitors are supported, with 4.7 uF
suggested. Verify the actual retained input/output capacitors after DC-bias and
layout changes. The quoted 350 mV dropout is TYPICAL, not the guaranteed maximum.

Lower dropout does not eliminate dissipation. Using the datasheet's 120 C/W
junction-to-ambient value as a conservative screening condition, a 4.7-to-3.3 V
drop at 300 mA is 0.42 W / 50 C rise; 500 mA is 0.70 W / 84 C rise. Actual copper,
airflow, enclosure and waveform matter. Thermal/current-step testing remains a
prototype requirement; do not advertise 1.2 A continuous capability on this PCB.

The user requires USB-only programming. Separating buck VCC5 from the diode-OR
LOGIC_5V prevents deliberate USB backfeeding of the buck output. It does not by
itself establish USB current compliance, low-end cable-voltage headroom, startup
inrush, suspended-port behavior or thermal margin. Programming firmware must keep
unneeded radio/network load off where feasible, and first-article testing must
measure VBUS current/rail minima. Motor outputs must remain inhibited without
valid external drive power. See the hardware contract for the accepted monitor.

## Candidate logic/source identities

Selections below are supply candidates, not permission to swap footprints blindly.
The inhibit designer owns the final topology and exact chosen variants.

| Purpose | Genuine manufacturer part / JLC code | Catalog screen | Important restriction |
|---|---|---|---|
| Schmitt configurable NOR | TI SN74LVC1G98DCKR / C485081 | 9,764 gross / 9,583 allocation indicator | SC70-6, not SOT23; verify configuration and pin mapping from TI. |
| Initial DFF candidate | TI SN74LVC1G74DCUR / C70285 | 13,510 / 13,327 | VSSOP8; ordinary CMOS inputs require edge-rate checks. May be superseded in final contract. |
| Alternate DFF candidate | Nexperia 74LVC1G74DP,125 / C458768 | 1,761 / 1,726 | TSSOP8 SOT505-2, not the TI DCU footprint. Data sheet describes Schmitt inputs but still lists a transition-rate limit; final design must account for it. |
| Alternate smaller DFF | Nexperia 74LVC1G74DC,125 / C503431 | 366 / 352 | VSSOP8 SOT765-1; exact body/pad comparison required. |
| Adjustable supervisor | TI TPS3808G01DBVR / C19653 | 18,547 / 18,253 | SOT23-6; threshold and hysteresis tolerances require a full allowed-voltage calculation. |
| Precision supervisor candidate | TI TPS389001DSER / C1509297 | 591 / 578 | WSON6 1.5 x 1.5 mm; final threshold and divider tolerance proof required. |
| Additional diode-OR leg | MDD SS34 / C8678 | >4 million / >4 million | SMA, same public code as existing selected part; do not substitute an SMB/SMC SS34 variant. |
| 680 ohm pull-down | UNI-ROYAL 0603WAF6800T5E / C23228 | >2 million / >2 million | 1%, 0.1 W, 0603. Only if final enable contract selects 680 ohm. |
| 1.5 kOhm resistor | UNI-ROYAL 0603WAF1501T5E / C22843 | >5 million / >4 million | 1%, 0.1 W, 0603. Final references/quantity to follow source. |
| Logic bypass | Yageo CC0603KRX7R9BB104 / C14663 | >51 million / >40 million | Reuse existing 100 nF, 50 V, X7R, 0603 line rather than new feeder. |

Public datasheets: [TI 1G98](https://www.ti.com/lit/ds/symlink/sn74lvc1g98.pdf),
[TI 1G74](https://www.ti.com/lit/ds/symlink/sn74lvc1g74.pdf),
[Nexperia 1G74](https://assets.nexperia.com/documents/data-sheet/74LVC1G74.pdf),
[TPS3808](https://www.ti.com/lit/ds/symlink/tps3808.pdf),
[TPS3890](https://www.ti.com/lit/ds/symlink/tps3890.pdf),
[UNI-ROYAL resistor family](https://www.royalohm.com/assets/pdf/products/smd/1.pdf),
[Yageo exact bypass](https://www.yageogroup.com/download/specsheet/CC0603KRX7R9BB104).

Rejected for immediate prototype sourcing: TI SN74LVC1G02DBVR C16360 (gross 2,
allocation indicator -1,511); TI SN74LVC1G98DBVR C507246 (gross zero, indicator -5).
Do not replace these with similarly named third-party clones without review.

## Fresh stock gate and source synchronization

The later complete-source screen is
`full_source_preliminary_alias_checked_summary.json` (2026-09-23 00:37:50 UTC):
144 required component instances / 46 exact supplier SKUs, screened for five
boards. All 46 are CONDITIONAL with adequate nominal catalog indicators. The
six matching DB25s C77833 show 780 gross / 390 allocation indicator, against 30
nominal units. This is not an actual processed JLC order allocation or spare-parts
calculation. Source metadata, final saved design and actual processed BOM are
still separate gates. No upload BOM or manufacturing release was generated by
this preparatory screen.

J1 C49257 uses a strictly bounded reviewed localization: source MPN
`2.54-1*3P针` equals the MPN retained in that exact catalog record's `componentName`,
while `componentModelEn` is `2.54-1*3PPin`. The checker recognizes only this
explicit code/name/English-name combination, not arbitrary fuzzy matching. The
initial full-source screen remains preserved to show why the alias was reviewed.

`source_supplier_plan.json` contains exactly 106 legacy-token corrections. Each
entry's MPN and footprint matched both the current persisted schematic and the
frozen r12 BOM; public code matched the per-reference map plus approved DB25
substitution. All 38 new/substituted references already used public codes.
`planned_source_map_r13.json` captures all 144 references but is deliberately
marked PREPARATORY_NOT_FINAL_SOURCE. Re-capture after source metadata changes;
do not relabel this preparatory capture as final evidence.

The independent `new_ic_footprint_audit.md/.json` checks all 30 U7/U11-U14 pads
against the contract and manufacturer drawings. Pin numbering/nets pass. It
recommends larger TI-example U11/U12/U14 lands before final routing and confirms
that U13's asymmetric pin-1 land is intentional, not corruption.

`check_sourcing_r13.py` is new and revision-local; it leaves every catalog-only
success CONDITIONAL and returns nonzero. It rejects missing exact code/MPN,
missing allocation indicator, insufficient gross stock or insufficient allocation
indicator. The four positive/negative classifier checks passed. It does not pretend
to know JLC attrition or convert catalog minimums into actual order demand.

`selected_contract_candidates_raw.json` preserves the earlier selected-contract
read-only JLC responses; the older `proposed_candidates` and `selected_candidates`
files preserve the exploration trail and are superseded. R46-R57 plus R61-R65
now use 17 C21190 resistors in this changed-part screen; R66 uses the ordinary
10 kOhm C25804 (not the precision divider resistor). Candidate counts are
provisional and not a final full-board BOM. Run on the fresh normalized BOM:

```text
python check_sourcing_r13.py --bom <r13-release>/JLC_BOM.csv --boards 5 --output-prefix final_bom
```

Five is the historical screening quantity, not authorization for an order. Repeat
for the actual quantity and require JLC's processed BOM to show no unmatched or
shortfall rows including its actual spares. No current stock is reserved.

For authoritative source substitutions, update exact manufacturer, MPN, public
supplier code, value/rating and all secondary supplier properties in both schematic
and PCB. Snapshot first, preserve full property maps, save/close/reopen, compare
both documents and fresh raw BOM. A normalized BOM alone does not update EasyEDA
one-click ordering. Previous order pages are stale after source changes.

The old `build_jlc_release_r12.py` delegates to a shared normalizer that reads
mutable global `ref2lcsc.json` and `assembly_substitutions.json` and hardcodes
123/113/1/9 population. r13 must freeze its own mappings, final count, and DB25
orientation correction, then verify all required designator sets. Never alter an
old frozen release or drop required parts to make a stock check pass.

**Current state: preparatory sourcing only; DO NOT ORDER from these files.** The
coherent r13 fabrication release and actual processed-assembly BOM remain separate
gates. Nothing here certifies electrical function, firmware or machine safety.
