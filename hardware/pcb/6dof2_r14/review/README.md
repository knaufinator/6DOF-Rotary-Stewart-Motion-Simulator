# R14 design review

R14 was ordered on 26 October 2026, as confirmed by the owner. Static review is
complete; manufactured-board qualification is pending. Start with the
[review method](../REVIEW_METHOD.md), [design decisions](../DECISIONS.md) and
[current section report](2026-10-26-10V/README.md).

| Current evidence | Scope |
| --- | --- |
| [Section report](2026-10-26-10V/README.md) | Circuit-by-circuit findings and qualification limits |
| [Components](2026-10-26-10V/every-component.csv) | 324 exact parts and suppliers |
| [Pins](2026-10-26-10V/every-pin.csv) | 1,214 logical memberships |
| [Copper](2026-10-26-10V/every-trace-via-fill.csv) | 2,667 trace/via/fill/test-pad objects |
| [Labels](2026-10-26-10V/every-label.csv) | 360 designators, board strings and digits |
| [Interface contract](2026-10-26-10V/interface-contract.csv) | 78 firmware/DB25 mappings |
| [R13 recovery](r13-recovered-source/README.md) | Exact tested-source provenance and evidence limits |
| [Q11 sourcing correction](2026-10-26-10V/Q11_STOCK_REFRESH.md) | Same MOSFET, alternate reel code, unchanged geometry |

The [26-page 25 October report](R14-complete-design-review-2026-10-25.pdf) is a
historical circuit-rationale appendix. It predates R13 recovery and the sensor
correction. Its old 319-part counts, missing-source finding and order hold are
superseded by the current report and [order record](../ORDER_RECORD.json).
Duplicate baseline registers and manufacturing sets are not published as current evidence.
