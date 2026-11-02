# R14C2 ordered checkpoint

**2 November 2026: ordered, awaiting physical testing.** The [order record](../../ORDER_RECORD_C2.json) records the confirmation and freezes the [fresh EasyEDA exports](../../exports/2026-11-02-ordered/README.md). No order was submitted by this repository update. The factory receipt, exact uploaded bytes, order identifier, quantity and private component allocation were not independently captured.

## Snapshot evidence

The [parity report](EXPORT_PARITY.json) proves every native non-header record matches the [reviewed C2 source](../2026-11-01-stock/README.md), including schematic, placed geometry and libraries. Aperture-resolved Gerber commands, raw and normalized BOM/CPL, and native netlists are unchanged. All 324 parts, 91 supplier codes and 1,214 logical pins remain present. Fresh native DRC and schematic/PCB comparison both contain zero findings.

The [internal-clearance receipt](INTERNAL_CLEARANCE.json) reruns all body/access checks against the fresh archive. It reuses the [reviewed bounds register](../2026-11-01-stock/BODY_LIMITS.json) only after exact source parity, and records both the register's hash and original source binding. The [fresh STEP review](MECHANICAL_REVIEW.json) separately checks all 324 component shapes and 52,326 component pairs. Native STEP exports renumber entities, so STEP text inequality is not treated as a geometry failure. The earlier C2 import disclosed nine unresolved-reference diagnostics; the fresh review again recovered all 324 expected component shapes as valid. The model result does not replace manufacturer bounds or mating checks.

The earlier [CAM, copper, pin, drill, mask and silk reports](../2026-11-01-stock/README.md) remain applicable through exact source/CAM parity. Earlier candidate status and manifests remain immutable historical records. The current order confirmation changes procurement status, not the technical qualification of those exceptions.

## Receipt and bench acceptance

| Remaining evidence | Required disposition |
| --- | --- |
| Delivered revision and sourcing | Match C2 drilling, 1.10 mm DB25 signal holes, Hanrun A/6 footprint, retained J12 clearance and all required fitted parts against the frozen source. Record actual receipt, quantities and substitutions. |
| Ethernet SI and fabrication stack-up | Confirm actual stack-up/impedance and reliable link/traffic operation. The retained TX layer transitions and RX via asymmetry, and the 14.123 mm maximum unsupported entry, are unqualified exceptions. Planar pair mismatch below 0.001 mm does not prove electrical delay, 100-ohm impedance or EMI performance. |
| Connector mating and enclosure | Check Amphenol seating-height discrepancy (supplied model 11.65 mm versus older drawing 12.3 mm), fasteners, supports, hood, panel and cable access. Hanrun catalog STEP differs from A/6 maximum dimensions; the conservative XY union was checked, not full external mating/height. |
| Assembly processing | Inspect the 50 inherited reduced positive drill-to-mask/paste gaps (minimum mask gap 0.032061 mm). No intersections were found, but assembler process acceptance was not independently recorded. |
| Sensor power and hardware fault paths | Measure nominal 12 V feed, falling/rising monitor thresholds (11.046/11.2 V nominal), the actual 15 ft Ethernet sensor cable, remote sensor voltage, reset/brownout/short behavior and independent pulse clear. |
| Firmware and motion | Follow [BRINGUP](../../BRINGUP.md) with exact image hashes, independent pulse captures and unloaded-axis tests. R13 one-axis evidence does not qualify R14. Keep `MotionAdmission<false>` until physical acceptance. |

No R14 powered test has run as part of this checkpoint. Firmware, motion permission and the monitor circuit were not changed by the export capture. Record future measurements as new evidence without rewriting the frozen manufacturing inputs.
