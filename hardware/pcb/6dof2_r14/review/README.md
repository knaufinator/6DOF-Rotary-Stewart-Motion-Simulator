# R14 design review

The [1 November R14C2 stock-revision report](2026-11-01-stock/README.md) is the current implementation review: all 324 required parts, native/CAM/STEP checks, sourcing and explicit fabrication gates. The [matching C2 package](../exports/2026-11-01-stock/README.md) contains the saved master and fresh exports. Firmware and safety interfaces are unchanged; physical qualification is pending.

R14 was ordered on 26 October 2026. Static review is
complete; manufactured-board qualification is pending. Start with the
[review method](../REVIEW_METHOD.md), [design decisions](../DECISIONS.md) and
[current section report](2026-10-26-10V/README.md).

The [27 October R14C1 review](2026-10-27-clearance/README.md) corrected the confirmed J12 collision; C2 retains that geometry. The [28 October Amphenol connector study](2026-10-28-amphenol/README.md) supplied six-position model evidence. C2 now applies those sockets with manufacturer-derived drills, while height/hardware qualification remains open.

| Circuit rationale and historical evidence | Scope |
| --- | --- |
| [Section report](2026-10-26-10V/README.md) | Circuit-by-circuit findings and qualification limits |
| [Components](2026-10-26-10V/every-component.csv) | 324 exact parts and suppliers |
| [Pins](2026-10-26-10V/every-pin.csv) | 1,214 logical memberships |
| [Copper](2026-10-26-10V/every-trace-via-fill.csv) | 2,667 trace/via/fill/test-pad objects |
| [Labels](2026-10-26-10V/every-label.csv) | 360 designators, board strings and digits |
| [Interface contract](2026-10-26-10V/interface-contract.csv) | 78 firmware/DB25 mappings |
| [R13 recovery](r13-recovered-source/README.md) | Exact tested-source provenance and evidence limits |
| [Q11 sourcing correction](2026-10-26-10V/Q11_STOCK_REFRESH.md) | Same MOSFET, alternate reel code, unchanged geometry |
| [1 November inventory check](2026-11-01-inventory/README.md) | All 91 current BOM codes; three public quantity shortfalls for five boards |
| [C2 final inventory](2026-11-01-stock/inventory/INVENTORY.csv) | All 91 selected C2 codes cover five boards' net fitted demand; allocation unverified |

The [26-page 25 October report](R14-complete-design-review-2026-10-25.pdf) is a
historical circuit-rationale appendix. It predates R13 recovery and the sensor
correction. Its old 319-part counts, missing-source finding and order hold are
superseded by the current report and [order record](../ORDER_RECORD.json).
Duplicate baseline registers and manufacturing sets are not published as current evidence.
