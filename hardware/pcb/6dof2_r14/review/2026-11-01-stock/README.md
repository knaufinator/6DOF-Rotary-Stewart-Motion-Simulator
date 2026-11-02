# R14C2 stock revision review

**Implementation and static checks complete; fabrication gates remain open.** This is the C2 revision started on 1 November and checked on 2 November 2026. It preserves the R14C1 J12 correction, the 11.046 V nominal falling sensor-monitor threshold and the recovered R13 firmware. It does not confirm a factory order or qualify powered motion.

## Sourcing decisions

| Positions | Old code | New selected part | Five-board demand | Public available order quantity |
| --- | --- | --- | --- | --- |
| R75, R143 | C13315 | C22766 / 0603WAF1204T5E, 1.2 MOhm, 1%, 0603 | 10 | 170,022 |
| J3-J8 | C77833 | C3110947 / Amphenol L77SDB25SOL2, black-insulator female DB25 | 30 | 220 |
| J10 | C55683 | C12074 / Hanrun HR911105A integrated MagJack | 5 | 54,513 |

The [public inventory screen](inventory/INVENTORY.csv) checked all 91 codes against five boards' net fitted demand, completing at `2026-11-02T10:45:43.516640+00:00` (UTC; this is a sourcing snapshot). All 324 parts remain required and fitted. This is not a reservation or a private order/attrition allocation. [Machine-readable inventory](inventory/INVENTORY.json) binds the exact native-derived BOM and CPL hashes.

The resistor keeps the exact MPN, resistance, tolerance and package. The DB25 sockets keep every numbered signal position and all six anchors/rotations; the manufacturer requires 1.10 mm signal drills and rates the signal contacts at 3 A. The HR911105A has integrated magnetics, separate TX/RX centre-tap roles and LEDs; its A/6 drawing drives the new footprint and pin-role map. No extra transformer is added.

## Persisted implementation

Both schematic and PCB contain exactly 324 production components, 91 supplier codes and 1,214 logical pins. All nine sourcing changes are saved in both documents. J10 is at `(235, 8.77) mm`, rotation 0; temporary J901/J902 parts are absent. All other 323 functional pin memberships and all other component poses match C1. Firmware is unchanged.

The Ethernet delta replaces 13 obsolete terminal polylines with 21 reviewed polylines and nine 20/12 mil vias. Existing unrelated copper, standalone test pads, slots and keepouts remain unchanged. The main GND-plane jack exclusion retains its width and ends at Y=22.3 mm, 3.005 mm behind the maximum drawing housing. All six native pours were rebuilt, saved, closed and reopened after the final route change. [Source contract](SOURCE_CONTRACT.json), [exact routing delta](STOCK_REVISION_CONTRACT.json).

The native J10 placement export uses Mid Y=10.765 mm and Ref Y=8.77 mm. The native midpoint compensates the catalog footprint origin: all 14 catalog contacts, including shields, register within 0.0051 mm. The six DB25s register within 0.00026 mm. [Native CPL registration](CONNECTOR_CPL_REGISTRATION.json). No manual rotation or coordinate patch is applied to the CSV.

## Measured checks

| Gate | Result | Evidence |
| --- | --- | --- |
| Native DRC / schematic-PCB comparison | Zero findings in fresh native calls | [DRC](../../exports/2026-11-01-stock/NATIVE_DRC.json), [comparison](../../exports/2026-11-01-stock/NATIVE_COMPARISON.json) |
| Native/BOM/CPL/STEP identities | 324 parts, 91 codes, 1,214 logical pins; no omissions or temporary parts | [Export validation](../../exports/2026-11-01-stock/EXPORT_VALIDATION.json) |
| Actual four-layer copper | 241 named nets, 1,080 named physical pads; no missing pads, opens, shorts, floating groups, foreign clearance or NPTH clearance findings; all 340 GND through objects join the actual main plane | [CAM review](ACTUAL_CAM_REVIEW.json), [ground joins](GROUND_JOINS.json) |
| Every body outline and internal body/access envelope | 324 parts, 52,326 pairs, 1,235 native lands; no body/access findings | [Body register](BODY_LIMITS.json), [internal clearances](INTERNAL_CLEARANCE.json), [outline audit](BODY_OUTLINES.json) |
| Fresh native STEP | All 324 named component shapes valid; 52,326 component pairs evaluated; zero solid intersections | [Mechanical report](MECHANICAL.json) |
| Actual drills | 1,121 unique features / 627,760 pairs; minimum analytic gap 0.302757 mm, above the recorded 0.302 mm gate | [Drill review](DRILL_REVIEW.json) |
| Via drill / SMT apertures | All 912 vias checked against 1,048 SMT mask apertures and actual paste; zero intersections | [Aperture review](APERTURE_REVIEW.json) |
| Labels / actual ink | All 348 board text envelopes pass; no actual ink on mask openings or outside the board; twelve DB25 digits rendered for visual reading | [Text envelopes](LABEL_ENVELOPES.json), [ink review](SILK_REVIEW.json), [digit image](DB25_DIGITS.png) |

## Ethernet review and release limits

The end-to-end native centreline graph includes retained PHY routes and the RX series-capacitor pad bridge. The first routing candidate was rejected for increased TX mismatch. The final new MDI bends use 45-degree chamfers and the planar paths measure approximately **60.4895 mm per TX line** and **55.1866 mm per RX line**, with less than 0.001 mm pair mismatch. [Path measurements](MDI_PATH_LENGTHS.json).

These are geometric lengths, excluding via vertical delay and component internals. TX uses three vias per line; RX has zero on P and one on N. [WIZnet's guide](https://docs.wiznet.io/Design-Guide/hardware_design_guide) calls for short, matched differential routing and discourages layer changes. This layout is longer than its preferred 25 mm; it retains layer transitions and a deliberate jack plane void. The longest new unsupported entry projection is **14.123 mm**. [Actual reference projections](SIGNAL_REFERENCES.json). Equal planar lengths and clean DRC do not prove 100-ohm impedance, EMI behaviour or Ethernet reliability. Exact stack-up and SI/EMI disposition remain a fabrication gate.

The internal XY checks use reviewed maximum/conservative bounds. The supplied Amphenol STEP assumes 11.65 mm seated height while an earlier manufacturer drawing gives 12.3 mm. Jackscrews/support and cable-hood/enclosure fit remain unqualified. The Hanrun A/6 body is nominally 21.20 x 16.00 x 13.30 mm; the native catalog STEP differs. Its conservative XY union is checked without stretching the model. The STEP reader's nine unresolved-reference diagnostics are disclosed in [qualification gates](QUALIFICATION_GATES.json); all 324 named valid component shapes were recovered.

The same 50 inherited reduced positive mask/paste gaps remain for assembler registration/stencil review. Public stock does not qualify through-hole processing or fixture access. The exact EasyEDA integrated order preview and factory allocation have not been exercised. Keep this package **DO NOT ORDER** until these release gates have an explicit disposition. No factory submission or confirmation occurred here.

C2 changes drilling and therefore requires matching new PCB fabrication. It cannot be introduced into an already fabricated original/C1 PCB by substituting only the BOM and CPL. The actual existing-order fabrication state and acceptance of replacement Gerbers require factory confirmation.

## Firmware and bench qualification

The recovered R13 pulse engine and all firmware interfaces remain unchanged. `MotionAdmission<false>` remains locked. The sensors still receive 12 V; the corrected falling monitor threshold is nominally 11.046 V. Measure the actual 15 ft Ethernet sensor cable and perform [BRINGUP.md](../../BRINGUP.md), including voltage margin/hysteresis, independent reset/fault/clear and pulse captures, then unloaded single-axis and six-axis testing. No R14 powered tests or safety-system certification are claimed.

## Sources and reproducibility

- [Hanrun A/6 drawing](../../references/HR911105A-HANRUN-REV-A6-20261101.pdf) and [manufacturer page](https://www.hanrun.com/rj45_100/154.html).
- [Amphenol drawing](../../references/L77SDB25SOL2-Amphenol-20261101.pdf), [manufacturer page](https://www.amphenol-cs.com/product/l77sdb25sol2.html), and the [supplied STEP provenance study](../2026-10-28-amphenol/README.md).
- [Selected resistor datasheet](../../references/0603WAF1204T5E-UNIROYAL-C22766-20261101.pdf).
- [Saved master and manufacturing package](../../exports/2026-11-01-stock/README.md); [PCB PDF](../../drawings/R14C2-PCB.pdf), [schematic PDF](../../drawings/R14C2-schematic.pdf).

The source/interface contract and complete mechanical checks run in CI. The manifest binds the actual exported files and curated evidence. Trial route files and API debugging history remain outside the repository. Frozen original-order and R14C1 packages are preserved.

The [README link review](README_LINK_REVIEW.json) covers all owned local targets/anchors and59unique external URLs. The Amphenol page returned403 and one ST datasheet request timed out; archived supplier PDFs remain accessible. Two inherited vendor-README WS2812B links return404 and remain recorded as dependency findings.

[Exact STEP pose evidence](STEP_POSE_PARITY.json) confirms317unchanged model bounds and places all seven new connector models inside reviewed XY envelopes. Connector bounds use OCCT `AddOptimal_s`; the full mechanical pair screen uses conservative `Add_s` boxes.
