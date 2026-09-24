# Buying fully assembled r13 prototypes

**r13 RC4 prototype-buying guide — NOT AN ORDER AUTHORIZATION.** Read
[README.md](README.md), the [exact RC4 release handoff](r13-rc4-2026-09-23/README.md)
and its [remaining gates](r13-rc4-2026-09-23/STATUS.md).
Nothing has been ordered or paid for. Locally verified files do not establish actual
JLC allocation, acceptance of every assembly process, or working-board validation.

The intended purchase is a fully populated PCB for development and bench testing.
It is **not yet a programmed, functionally validated, ready-to-plug servo controller**.
Firmware is deferred. The owner should not need to solder omitted board connectors.

## 1. Use one complete release

The final package must contain matching r13 Gerber/drill, BOM, CPL, source checkpoint,
STEP/mechanical evidence, PCB/schematic renders, validation results and a final
SHA-256 manifest. The release status must permit prototype manufacture and name any
remaining bench-test limitations. Do not use r12 files to fill gaps or mix exports.
Use the actual filenames and checksums in the RC4 release handoff; a general
buying guide is not a substitute for its manifest.

Required design checks include persisted schematic/PCB agreement, completed routing,
native DRC, correct outline/drills, copper/keepout review and actual component-body
fit. A passed schematic audit is not a routed PCB release. All of these remain
distinct from testing a manufactured board.

## 2. Request the right assembly scope

Use **Standard PCBA with complete SMT plus through-hole/hybrid assembly** of all
144 required components. The selected ESP32-S3-WROOM-1-N8R2 / C2913204 is listed as
**Standard Only**, so Standard is required for this BOM, not merely an optional
future programming service. [Exact JLC part page](https://jlcpcb.com/partdetail/3198302-ESP32_S3_WROOM_1N8R2/C2913204),
rechecked 2026-09-23. The reviewed population includes nine through-hole components
per board (six DB25s, J1, J2 and J9), or 45 across five boards. Confirm JLC accepts
their required wave/manual/hybrid processes and any fixtures; do not leave them
for the owner to solder. Programming and functional testing, if later supplied,
require a separately reviewed method/image and quote.
[Programming service](https://jlcpcb.com/help/article/pcba-programming-service),
[functional testing](https://jlcpcb.com/help/article/functional-test-service).

The following are required, not optional/DNI:

- **J3–J8: all six DB25 connectors on every board, DS1034-25FUNSI44 / C77833.**
  This identity persists in schematic, PCB and BOM. Do not revive the old C3110947
  substitution/shortfall problem or use an order page created before the source fix.
- **J10: the exact integrated-magnetics Ethernet connector (MagJack)**, including
  its applicable signal contacts and shield/mechanical solder joints.
- **J2/J9 terminal blocks, J1 service header, J11 USB-C, and all other BOM parts.**
  USB hybrid retention pins and connector shell tabs are part of assembly scope.

If the order page cannot schedule any connector for automatic placement, request
JLC's accepted DIP/through-hole, selective/wave or hand-assembly process as appropriate
to that exact component. A manually installed part **at JLC** meets the fully assembled
goal; a part omitted for the customer to solder does not. Obtain written confirmation
and the charge before paying. Do not change part metallurgy/footprint or add parts
after ordering without reviewed approval; JLC warns that late assembly additions may
be unsupported. [Additional assembly services](https://jlcpcb.com/help/article/jlcpcb-supported-personalized-services).

## 3. Reconcile quantities and sourcing

Current r13 planning basis: **144 component designators mapped to 46 supplier SKUs**.
These are working-design expectations, not a processed order. The final exported
BOM/CPL must reproduce the final source designator set with no duplicate or missing
part. If the final audited count changes, update this guide and record why; do not
silently remove a required reference to keep a quotation moving.

| Assembled boards | Nominal component placements | DB25 required | MagJacks required |
|---|---:|---:|---:|
| 2 | 288 | 12 | 2 |
| 5 | 720 | 30 | 5 |

The 46-SKU figure is the number of unique ordered part types, not 46 parts per board.
Actual purchased quantities can exceed placements due to JLC attrition/minimums.
For five assembled boards, seeing only 24 DB25 units allocated is still **six short**.
Enough gross catalog stock is not proof that the processed order has an allocation.

For each SKU check public supplier code, exact MPN/package/variant, all mapped
designators, nominal need, attrition quantity, available allocation and assembly
eligibility. Resolve a shortfall through accepted procurement/pre-order or a reviewed
source-and-layout-compatible substitution. Do not uncheck the part, mark it DNI or
accept partial connector population. Refresh sourcing immediately before purchase.

## 4. Upload three inputs, then inspect

1. Upload only the final **fabrication Gerber/drill ZIP** into the PCB field.
2. Upload the final **BOM** and **CPL** separately into their assembly fields.
   A wrapper ZIP containing those files is not the fabrication upload.
3. Set board dimensions, four-layer stack, thickness, finish and copper weights from
   the final release specification. Do not guess them from an old cart or rendering.
   Follow [STATUS.md](r13-rc4-2026-09-23/STATUS.md) for the accepted JLC fabrication
   stackup and dielectric spacing. The editor's 11 persisted physical-layer records
   prove source recovery, not a qualified fabrication stackup or controlled impedance.
   Have JLC's actual four-layer construction and nominal 1.6 mm thickness accepted
   before purchase; record its copper weights, dielectric thicknesses and material.
4. Confirm the assembled quantity separately from the bare-board quantity. Compare
   two versus five fully assembled prototypes using the same scope.
5. Check processed parts and all connector orientations in the assembly preview.
   For DB25s use physical pin-1/body orientation evidence; do not blindly add another
   180-degree correction to a CPL that may already contain it.
6. Review CAM outline/drills/layer registration, DFM questions, component bodies,
   mounting support and connector access. Keep the accepted preview and correspondence.

## 5. Confirm the total and order description

Compare the delivered total: PCB, parts, SMT/THT assembly, setup/fixtures, any special
processes, testing, shipping, duties and taxes. No historical quote is current approval.
The design-review price estimate is not an accepted JLC production quote.

Suggested order-scope note, to adapt after the release is complete:

> Fully assemble the released r13 BOM, including all six DB25 connectors per board,
> MagJack, terminal blocks, service header and USB-C retention/shield joints.
> No required part is to be omitted or left for customer soldering. Please confirm
> any through-hole/manual assembly process, shortages and orientation questions
> before production. These are unprogrammed engineering prototypes for customer
> bench validation; no motion-ready firmware or full functional test is supplied.

If programming/FCT is later requested, provide a validated image, flash method,
fixture and explicit pass/fail procedure; get separate written acceptance. Do not
select generic functional testing and assume it includes servo-system validation.
Assembly does not include the user's drives, mains wiring, cables, stop/brake system,
power supply or enclosure unless separately contracted.

Only the owner places/authorizes the actual purchase. Retain the accepted order ID,
revision, files/hashes, assembly scope and quote. Record any later design change as
a new release and never silently replace files already approved for manufacture.
