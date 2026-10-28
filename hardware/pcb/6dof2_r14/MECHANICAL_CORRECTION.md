# R14C1 mechanical correction — 27 October 2026

**Corrected EasyEDA master and replacement exports complete. Original order remains on an engineering hold pending factory disposition.** J12 is required and remains populated. No factory message or order-system change has been sent by the agent.

## J12 outline and root cause

J12 is **Amphenol RJHSE-5080 / C599441**: nominal housing **15.75 × 14.99 mm**, height **13.21 mm**, from the [manufacturer drawing](https://cdn.amphenol-cs.com/media/wysiwyg/files/drawing/p-rjhse-5080.pdf). Its native component outline consists of four `LINE` records on the component-body layer. The earlier placement screen read closed `POLY` outlines and silently omitted this body. Electrical DRC checked copper rules; the earlier STEP review checked model presence/validity and selected new power parts rather than every assembly pair. Those checks did not establish whole-board physical clearance.

The dimensions and geometry were available. This was a verification failure, not an unexplained JLC model scale error. The archived ordered STEP independently contains the same interference: **J12/U20 52.9387 mm³**, J12/D13 1.0220 mm³, J12/R87 0.1972 mm³ and J12/C53 0.0739 mm³. U20's case extends roughly **1.8 mm** into J12's housing in plan view, or about **3.6 mm** including the lead envelope. These are substantial overlaps, not merely touching silkscreen.

![Reviewed body envelopes before and after correction](images/r14c1-clearance-before-after.png)

## Corrected master

J12's position, footprint and pin assignment stay fixed. U20 moves down **5.6 mm** to Y=22 mm, giving approximately **2.02 mm** nominal STEP separation from J12. U19 and nearby Hall-bank passives move with rerouted copper. J1 is separated from J2; Q4 and nearby small packages receive tolerance margin; TP3/TP8 gain probe access. In total, **30 component positions** change, **62 designator labels** are reflowed, and the board carries **R14C1** identification. Every one of the 324 parts remains fitted. The master was saved, closed, reopened and exported through the EasyEDA bridge; this is not an export-only patch.

Final actual CAM checks additionally caught and removed six old vias under relocated U19 SMT lands, and a small C51 outline/R87 mask-opening collision. The final package has no such intersections. All six pours were regenerated after routing corrections.

## Acceptance evidence and limits

The [dated review](review/2026-10-27-clearance/README.md) records all 324 components / 91 supplier codes, 52,326 mechanical pairs, 1,235 native lands, 2,684 copper-register rows, 1,098 drills and 348 board-level text envelopes. Manufacturer maximum or conservative family bounds include dimensional tolerances, plus 0.15 mm placement allowance per edge. Minimum expanded body spacing is 0.5 mm for SMT parts and 1.0 mm when a connector is involved; foreign-land clearances are 0.25/0.5 mm respectively. All eleven test pads and six board mounting sites pass the specified probe/head checks. All 324 native bodies were independently reconstructed a second time across all 57 used footprints: 321 POLY bodies, two FILL bodies and J12’s four-LINE body. No missing, open, degenerate, self-crossing or unsupported body was found. [Per-component outline evidence](review/2026-10-27-clearance/every-component-outline.csv) and six regression tests cover the actual J12 case and malformed/missing geometry.

All 324 exported STEP models are valid, and no solid intersections remain. Source/STEP pose parity proves the export follows the corrected master. Native DRC and schematic/PCB comparison are empty; actual CAM has no missing named pads, shorts, opens or foreign-copper clearance findings. See the report for inherited mask-gap and signal-reference limits.

This qualifies internal board separation within the documented assumptions. The exact fitted D-sub nut-stack heights for J3–J8, mated plug/hood/cable swept volumes and enclosure fit remain unqualified. No R14 bench tests have run. The recovered R13 firmware and corrected R14 12 V sensor supply/voltage-monitor/pulse-clear circuit are unchanged and checked against the ordered electrical interfaces.

## Manufacturing disposition

Use the complete [R14C1 replacement package](exports/2026-10-27-clearance/README.md), including its Gerbers, BOM and CPL. **Do not remove J12.** The original 26 October package and tag remain immutable historical records and are not assembly-approved.

If fabrication has not started, the factory must accept the complete replacement set and perform its assembly review. If the boards are already fabricated, new fabrication is required: changing the CPL cannot move existing pads. The factory's current stage and acceptance of replacement files have not been confirmed here. A suggested reply is: “Please retain J12; it is required. We have corrected the PCB placement and routing and can provide a complete replacement manufacturing package. Please confirm whether fabrication has started and whether you can replace the complete file set before proceeding.”
