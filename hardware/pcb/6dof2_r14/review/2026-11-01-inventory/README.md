# R14C1 inventory check — 1 November 2026

**88 of 91 BOM codes cover the net fitted demand for five boards in JLC's public Available Order Qty. Three codes have shortfalls.** This is a sourcing snapshot, not an order allocation or fabrication release.

The check ran from 21:21:53 to 21:22:27 UTC (16:21–16:22 EST) against the current [R14C1 BOM](../../exports/2026-10-27-clearance/R14-BOM-JLC.csv) and [CPL](../../exports/2026-10-27-clearance/R14-CPL-JLC.csv) at commit `671702d10a9232910f4c6d9a5750f699a1a23051`. All 324 required fitted references and 91 unique supplier codes resolved. BOM quantities match designator counts; the BOM and CPL contain the same unique references.

Five boards is the provisional screen based on the planned order quantity. The tables also show two-board demand because actual order quantity and JLC's processing losses must be checked in the order. No attrition allowance is invented here.

## Shortfalls

| References / supplier code | Part | Need: 2 boards | Need: 5 boards | Available Order Qty | Separate catalog stock |
| --- | --- | ---: | ---: | ---: | ---: |
| J3–J8 / [C77833](https://jlcpcb.com/partdetail/CONNFLYElec-DS103425FUNSI44/C77833) | Connfly DS1034-25FUNSI44 DB25 | 12 | 30 | **0** | 65 |
| J10 / [C55683](https://jlcpcb.com/partdetail/56710-HR961160C/C55683) | HANRUN HR961160C MagJack | 2 | 5 | **4** | 7 |
| R75, R143 / [C13315](https://jlcpcb.com/partdetail/13962-0603WAF1204T5E/C13315) | UNI-ROYAL 0603WAF1204T5E resistor | 4 | 10 | **0** | 283 |

J10 covers two boards' net demand, before any order-specific processing losses. The DB25 and resistor codes fail the public quantity screen for either batch size. For C13315, the catalog also returns a minimum purchase quantity of 4,749 and a loss field of 12; neither value establishes an executable assembly purchase or the actual order's attrition requirement.

Catalog stock and public orderable quantity are separate fields. The DB25's 65 catalog units and the resistors' 283 catalog units do **not** override zero Available Order Qty. Confirm sourcing and allocation in the actual JLC order before relying on these parts for a new build. No required part has been omitted or substituted.

## Other checked parts

| References / supplier code | Available Order Qty | Net need: 5 boards | Result |
| --- | ---: | ---: | --- |
| Q11 / [C155242](https://jlcpcb.com/partdetail/DiodesIncorporated-DMN6075S7/C155242) | 15,666 | 5 | Corrected reel code covers net demand |
| Y1 / [C9006](https://jlcpcb.com/partdetail/YXC_CrystalOscillators-X322525MOB4SI/C9006) | 348,188 | 5 | Covers net demand |
| U13 / [C1509297](https://jlcpcb.com/partdetail/TexasInstruments-TPS389001DSER/C1509297) | 50 | 5 | Covers net demand; relatively limited stock |
| U29, U30 / [C4471167](https://jlcpcb.com/partdetail/TexasInstruments-SN74HCS02BQAR/C4471167) | 102 | 10 | Covers net demand |
| J12 / [C599441](https://jlcpcb.com/partdetail/AmphenolICC-RJHSE5080/C599441) | 577 | 5 | Covers net demand |

The [complete CSV](INVENTORY.csv) lists every current BOM code, references, quantities for two and five boards, public orderable quantity, separate catalog stock, result, source URL and check time. The [JSON evidence](INVENTORY.json) retains selected supplier fields, response dates and page hashes. There are no unresolved current BOM codes in this snapshot.

## Unapplied connector candidates

| Candidate | Available Order Qty | Catalog stock | Assessment |
| --- | ---: | ---: | --- |
| Black Connfly [C19268332 / DS1034-25FBNSi44](https://jlcpcb.com/partdetail/CONNFLYElec-DS103425FBNSi44/C19268332) | 3 | 3 | Insufficient for all six connectors on either two or five boards |
| Black Amphenol [C3110947 / L77SDB25SOL2](https://jlcpcb.com/partdetail/AmphenolICC-L77SDB25SOL2/C3110947) | 220 | 246 | Quantity covers net demand; mechanical substitution is not approved |

The [Amphenol study](../2026-10-28-amphenol/README.md) still leaves installed height, mounting and footprint-hole compatibility unresolved. More stock does not qualify a replacement. Both candidates remain outside the current BOM and CAD master.

## Method and limits

The [read-only checker](../../../../../tools/check_jlc_inventory.py) queries JLC's public catalog by exact supplier code, then reads that exact product page's server data. It does not execute downloaded JavaScript. The product frontend maps `canPresaleNumber` to its orderable maximum (`maxInStock`), while `overseasStockCount` supplies the separate stock count. [Field-mapping evidence](QUANTITY_FIELD_EVIDENCE.json) records the official frontend source and relevant excerpts. Negative search-endpoint quantities are not used as availability. Missing, negative, inconsistent or unrecognized detail data produces an unknown result rather than a pass.

This check does not access account-private parts inventory, reserved stock or the existing order's allocation. A public shortage does not prove an already allocated order is short. No stock was purchased or reserved, and no supplier or factory was contacted. Public quantities can change after the recorded check.

The CAD, firmware, BOM, CPL and manufacturing exports are unchanged. The existing [R14C1 review and qualification limits](../2026-10-27-clearance/README.md) remain in force. This report does not certify assembly lead time, mechanical acceptance or powered-board operation.

Reproduce from the repository root with Python 3.12 or later and a trusted TLS certificate store (`certifi` is supported when installed):

```sh
python tools/check_jlc_inventory.py \
  --bom hardware/pcb/6dof2_r14/exports/2026-10-27-clearance/R14-BOM-JLC.csv \
  --cpl hardware/pcb/6dof2_r14/exports/2026-10-27-clearance/R14-CPL-JLC.csv \
  --boards 5 --workers 3 \
  --extra-code C19268332 --extra-code C3110947 \
  --output /path/to/new-inventory-snapshot
python tools/test_jlc_inventory.py
```

The nine offline regression tests check exact part identity, conflicting page data, unknown quantities, buyability, BOM validation and the distinction between orderable and catalog stock. CI runs these tests without making live inventory requests.

Input SHA-256:

```text
BOM 811d35d8d9af3b070002002a248d7f40eb37e9ed71548e28039766ab07c6f498
CPL ee8ab8b1bdd339f18d8c618fb4d4855c56c72a05dc87398e7becdd2d488633f2
```
