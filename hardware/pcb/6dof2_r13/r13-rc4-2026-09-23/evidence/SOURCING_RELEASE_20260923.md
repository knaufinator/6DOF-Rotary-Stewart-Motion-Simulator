# r13 sourcing and full-assembly release audit

Status: **DO NOT ORDER — sourcing CONDITIONAL pending the actual JLC assembly quote/allocation.**

Checked September 23, 2026, 13:30–13:34 UTC. Catalog screen completed 13:31:04 UTC; two rate-limited public process pages were successfully rechecked at 13:33:55–56 UTC. No orders, pre-orders, reservations, uploads, supplier messages, substitutions or EasyEDA edits were performed.

## Result

- All **144 required components / 46 exact SKUs** are preserved for **five boards: 720 nominal placements**. No optional, DNI or user-hand-installed parts.
- All 46 exact-code catalog records match the expected MPN, with one explicitly reviewed Chinese/English translation for J1 C49257. Public gross stock and allocation indicators cover both nominal demand and a deliberately generous provisional screen. There is **no catalog-screen shortfall**.
- The recurring DB25 shortage is not present in this source/export: J3–J8 are six matching **CONNFLY DS1034-25FUNSI44 / C77833**, not C3110947. Current catalog values are **780 gross / 390 allocation indicator**, versus **30 nominal** for five boards and a **60-piece provisional screen**.
- **Standard PCBA is required** by U2 C2913204. Its exact public product page says Standard Only. Do not omit the ESP32 or proceed with an Economic quote that rejects it. [JLC ESP32 module](https://jlcpcb.com/partdetail/3198302-ESP32_S3_WROOM_1N8R2/C2913204)
- Full assembly is publicly supported: **135 SMT and nine through-hole components per board**. J1, J2, J9 and J3–J8 account for 45 through-hole placements across five boards, and all three exact connector SKUs are listed as Wave Soldering. J10 MagJack and J11 USB-C are listed as SMT Assembly; do not classify either as a removable hand-install item. [DB25](https://jlcpcb.com/partdetail/CONNFLYElec-DS103425FUNSI44/C77833), [mixed-technology capability](https://jlcpcb.com/capabilities/pcb-assembly-capabilities)

The catalog's `stockCount` is not an allocated order quantity. The undocumented `canPresaleNumber` field is retained as an allocation **indicator**, not promoted to verified availability. Static public product HTML did not expose an Available Order Qty value. Neither LCSC retail inventory nor a nonzero JLC catalog record proves that the final job can reserve its parts. JLC documents shortages from minimum placement and loss allowances, and differences between LCSC and assembly stock. [BOM matching warnings](https://jlcpcb.com/help/article/common-bom-and-cpl-matching-issues-and-explanations), [assembly FAQ, part 2](https://jlcpcb.com/help/article/pcb-assembly-faqs-part-2)

## Exact persisted source and export binding

Identity audit PASS against `100_routed_labeled_capture.json` and `44_schematic_aligned_reopened.json`: every reference has the same public supplier code, MPN, footprint UUID and Unique ID. All schematic Add into BOM / Convert to PCB flags are yes. Native PCB inline overrides agree where present; other resolved identities inherit the project Device metadata. This does not claim that every property is duplicated inline.

Fresh native export 107 also passes:

| Artifact | Result | SHA-256 |
| --- | --- | --- |
| `manufacturing_release/2026-09-23-r13-rc1/6DOF2_r13_BOM_raw.csv` | 49 native rows, 46 SKUs, 144 unique required refs. Exact code/MPN/footprint match. | `e2852d8e078cec9d49f5cf6fb51ce91fac7dc3da106c0ba5bd31259f03600da1` |
| `manufacturing_release/2026-09-23-r13-rc1/6DOF2_r13_CPL_raw.csv` | 144 unique refs, all Top, footprint match; no missing/extra refs. | `8c927be3b9fb220facc03115b73be83ba109d11ab65bb8ddf2c8eff6dbd2ebcf` |
| PCB100 source text | Persisted source used for audit | `284e27148e25e48cc817f879fc999f346fe6762017167230dc45a71f904084fe` |

Original UTF-16 tab-separated native exports were read only, not modified. Raw CPL rotations are **not** approved here. The independent orientation/normalizer gate must preserve the established J3–J8 +180° library-zero correction and check all polarized/directional parts.

The normalized upload files were also independently bound at 13:43:35 UTC: `6DOF2_r13_BOM_JLC.csv` has 46 rows / 144 exact refs, matching code, MPN, footprint and group quantity; SHA-256 `abb2c6a7b8e7f5ca94942f3d153ba13d09623d36f68c50f65ef61399d72f423c`. `6DOF2_r13_CPL_JLC.csv` preserves all 144 unique required refs on Top; SHA-256 `43e663b34e681b8c1de7c1f2084aa8874206f02b15479567abcc260b41c70f4d`. Exact angle/mechanical orientation approval remains the independent orientation review.

## Five-board inventory screen

Every row below is REQUIRED and CONDITIONAL. Quantities are nominal, not JLC's actual demand including attrition. “Indicator” means `canPresaleNumber`, **not reserved or guaranteed available assembly quantity**. The JSON records each exact manufacturer, package, source footprint, timestamp, primary product/datasheet link, price tiers, minimums and process-page evidence.

The provisional screen is `max(2 × nominal, nominal + 20, catalog leastPatchNumber)`. This is an intentionally conservative project screening heuristic, **not a prediction of JLC attrition**. All 46 indicators exceed that screen. All catalog `minPurchaseNum` fields are 1; this does not waive minimum placement or pre-order quantities. Catalog `leastPatchNumber`, `lossNumber`, pack quantity and pre-order minimum are preserved separately in JSON and never mistaken for a final order requirement.

| Public SKU | References | Exact MPN | Nominal / 5 boards | Gross catalog | Indicator | Class | Public process |
| --- | --- | --- | ---: | ---: | ---: | --- | --- |
| [C1589](https://jlcpcb.com/partdetail/1941-CL10B103KB8NNNC/C1589) | C12,C35 | CL10B103KB8NNNC | 10 | 2,176,194 | 2,094,480 | Extended | SMT |
| [C1631](https://jlcpcb.com/partdetail/1983-0603B682K500NT/C1631) | C33,C34 | 0603B682K500NT | 10 | 278,346 | 249,794 | Basic | SMT |
| [C1647](https://jlcpcb.com/partdetail/1999-CL10C180JB8NNNC/C1647) | C23,C24 | CL10C180JB8NNNC | 10 | 212,205 | 147,514 | Basic | SMT |
| [C1779](https://jlcpcb.com/partdetail/2131-CL21A475KAQNNNE/C1779) | C15,C42 | CL21A475KAQNNNE | 10 | 2,865,452 | 2,613,342 | Basic | SMT |
| [C2297](https://jlcpcb.com/partdetail/Hubei_KENTOElec-KT0805G/C2297) | D4,D5 | KT-0805G | 10 | 1,259,904 | 1,106,305 | Basic | SMT |
| [C3338](https://jlcpcb.com/partdetail/HonorElec-RVT1E101M0607/C3338) | C1 | RVT1E101M0607 | 5 | 58,484 | 56,655 | Extended | SMT |
| [C4190](https://jlcpcb.com/partdetail/4597-0603WAF2201T5E/C4190) | R30,R31,R32,R33,R34,R35 | 0603WAF2201T5E | 30 | 8,055,074 | 7,627,414 | Basic | SMT |
| [C6179](https://jlcpcb.com/partdetail/TexasInstruments-AM26LS31CDR/C6179) | U3,U4,U5 | AM26LS31CDR | 15 | 48,516 | 47,775 | Extended | SMT |
| [C7519](https://jlcpcb.com/partdetail/STMicroelectronics-USBLC62SC6/C7519) | D6 | USBLC6-2SC6 | 5 | 39,152 | 36,335 | Extended | SMT |
| [C8678](https://jlcpcb.com/partdetail/MDD_Microdiode_Semiconductor-SS34/C8678) | D1,D2,D3,D8 | SS34 | 20 | 4,639,813 | 4,488,004 | Basic | SMT |
| [C9006](https://jlcpcb.com/partdetail/YXC_CrystalOscillators-X322525MOB4SI/C9006) | Y1 | X322525MOB4SI | 5 | 164,412 | 135,515 | Basic | SMT |
| [C14663](https://jlcpcb.com/partdetail/YAGEO-CC0603KRX7R9BB104/C14663) | C4,C5,C8,C9,C10,C13,C16,C17,C18,C20,C21,C26,C27,C28,C29,C30,C36,C37,C38,C39,C40,C41 | CC0603KRX7R9BB104 | 110 | 49,789,131 | 38,624,002 | Basic | SMT |
| [C15008](https://jlcpcb.com/partdetail/15681-CL31A107MQHNNNE/C15008) | C3 | CL31A107MQHNNNE | 5 | 2,117,061 | 2,002,334 | Basic | SMT |
| [C15849](https://jlcpcb.com/partdetail/16531-CL10A105KB8NNNC/C15849) | C6,C14 | CL10A105KB8NNNC | 10 | 6,979,746 | 5,239,901 | Basic | SMT |
| [C15850](https://jlcpcb.com/partdetail/16532-CL21A106KAYNNNE/C15850) | C7,C11,C19,C22,C31 | CL21A106KAYNNNE | 25 | 5,734,934 | 4,660,656 | Basic | SMT |
| [C21122](https://jlcpcb.com/partdetail/21834-CL10B223KB8NNNC/C21122) | C32 | CL10B223KB8NNNC | 5 | 984,392 | 860,326 | Basic | SMT |
| [C21190](https://jlcpcb.com/partdetail/21904-0603WAF1001T5E/C21190) | R6,R7,R18,R19,R46,R47,R48,R49,R50,R51,R52,R53,R54,R55,R56,R57,R60,R61,R62,R63,R64,R65 | 0603WAF1001T5E | 110 | 24,722,903 | 20,485,938 | Basic | SMT |
| [C22859](https://jlcpcb.com/partdetail/23586-0603WAF100JT5E/C22859) | R59 | 0603WAF100JT5E | 5 | 8,619,027 | 7,925,236 | Basic | SMT |
| [C22865](https://jlcpcb.com/partdetail/23592-0603WAF1242T5E/C22865) | R10 | 0603WAF1242T5E | 5 | 237,157 | 224,005 | Extended | SMT |
| [C22935](https://jlcpcb.com/partdetail/23662-0603WAF1004T5E/C22935) | R58 | 0603WAF1004T5E | 5 | 7,019,526 | 6,520,434 | Basic | SMT |
| [C23140](https://jlcpcb.com/partdetail/23867-0603WAF330JT5E/C23140) | R12,R13,R14,R15,R16,R17,R20,R21,R22,R23,R24,R25,R69,R70 | 0603WAF330JT5E | 70 | 5,934,403 | 5,562,390 | Basic | SMT |
| [C23186](https://jlcpcb.com/partdetail/23913-0603WAF5101T5E/C23186) | R2,R3 | 0603WAF5101T5E | 10 | 26,453,035 | 25,616,192 | Basic | SMT |
| [C23631](https://jlcpcb.com/partdetail/YAGEO-CC1206KKX7RDBB102/C23631) | C25 | CC1206KKX7RDBB102 | 5 | 1,440,377 | 1,398,896 | Extended | SMT |
| [C25804](https://jlcpcb.com/partdetail/26547-0603WAF1002T5E/C25804) | R1,R4,R5,R11,R36,R37,R38,R39,R40,R41,R66 | 0603WAF1002T5E | 55 | 24,459,513 | 18,204,110 | Basic | SMT |
| [C32843](https://jlcpcb.com/partdetail/WIZNET-W5500/C32843) | U8 | W5500 | 5 | 10,343 | 8,297 | Extended | SMT |
| [C49257](https://jlcpcb.com/partdetail/50265-2_54_13PPin/C49257) | J1 | 2.54-1*3P针 | 5 | 267,349 | 258,550 | Extended | Wave |
| [C55683](https://jlcpcb.com/partdetail/56710-HR961160C/C55683) | J10 | HR961160C | 5 | 909 | 662 | Extended | SMT |
| [C61063](https://jlcpcb.com/partdetail/XLSEMI-XL1509_50E1/C61063) | U6 | XL1509-5.0E1 | 5 | 463,124 | 444,136 | Basic | SMT |
| [C70285](https://jlcpcb.com/partdetail/TexasInstruments-SN74LVC1G74DCUR/C70285) | U11 | SN74LVC1G74DCUR | 5 | 12,965 | 12,782 | Extended | SMT |
| [C77833](https://jlcpcb.com/partdetail/CONNFLYElec-DS103425FUNSI44/C77833) | J3,J4,J5,J6,J7,J8 | DS1034-25FUNSI44 | 30 | 780 | 390 | Extended | Wave |
| [C95204](https://jlcpcb.com/partdetail/YAGEO-RT0603BRD0710KL/C95204) | R68 | RT0603BRD0710KL | 5 | 1,707,985 | 1,045,713 | Extended | SMT |
| [C108301](https://jlcpcb.com/partdetail/ChilisinElec-PBY160808T_601YN/C108301) | L2 | PBY160808T-601Y-N | 5 | 1,135,088 | 1,134,449 | Extended | SMT |
| [C114599](https://jlcpcb.com/partdetail/LiteOn-LTV847S/C114599) | U9,U10 | LTV-847S | 10 | 6,252 | 5,897 | Extended | SMT |
| [C114625](https://jlcpcb.com/partdetail/YAGEO-RC0603FR0749R9L/C114625) | R42,R43,R44,R45 | RC0603FR-0749R9L | 20 | 316,858 | 291,931 | Extended | SMT |
| [C165948](https://jlcpcb.com/partdetail/Korean_HropartsElec-TYPE_C_31_M12/C165948) | J11 | TYPE-C-31-M-12 | 5 | 99,004 | 89,878 | Extended | SMT |
| [C193402](https://jlcpcb.com/partdetail/MDD_Microdiode_Semiconductor-SMF50A/C193402) | D7 | SMF5.0A | 5 | 582,402 | 580,571 | Extended | SMT |
| [C435835](https://jlcpcb.com/partdetail/STMicroelectronics-LDL1117S33R/C435835) | U7 | LDL1117S33R | 5 | 25,035 | 24,536 | Extended | SMT |
| [C455280](https://jlcpcb.com/partdetail/XUNPU-TS_1088R02026/C455280) | SW1,SW2 | TS-1088R-02026 | 10 | 179,494 | 176,545 | Extended | SMT |
| [C474952](https://jlcpcb.com/partdetail/Cixi_KefaElec-KF128_5_08_2PAA/C474952) | J2,J9 | KF128-5.08-2P-AA | 10 | 52,044 | 49,878 | Extended | Wave |
| [C485081](https://jlcpcb.com/partdetail/TexasInstruments-SN74LVC1G98DCKR/C485081) | U12,U14 | SN74LVC1G98DCKR | 10 | 9,764 | 9,583 | Extended | SMT |
| [C861325](https://jlcpcb.com/partdetail/YAGEO-RT0603BRD0732KL/C861325) | R67 | RT0603BRD0732KL | 5 | 9,656 | 9,627 | Extended | SMT |
| [C883148](https://jlcpcb.com/partdetail/BHFUSE-BSMD1812_11016V/C883148) | F1 | BSMD1812-110-16V | 5 | 14,707 | 14,698 | Extended | SMT |
| [C1509297](https://jlcpcb.com/partdetail/TexasInstruments-TPS389001DSER/C1509297) | U13 | TPS389001DSER | 5 | 586 | 573 | Extended | SMT |
| [C2913204](https://jlcpcb.com/partdetail/3198302-ESP32_S3_WROOM_1N8R2/C2913204) | U2 | ESP32-S3-WROOM-1-N8R2 | 5 | 3,616 | 3,287 | Extended | SMT / Standard only |
| [C4747974](https://jlcpcb.com/partdetail/KNSCHA-RST220UF25V019/C4747974) | C2 | RST220UF25V019 | 5 | 211,425 | 185,766 | Extended | SMT |
| [C7431083](https://jlcpcb.com/partdetail/SHOUHAN-CYH12568UH/C7431083) | L1 | CYH125-68UH | 5 | 14,757 | 14,542 | Extended | SMT |

There are 19 Basic and 27 Extended SKUs, none flagged Preferred Extended. The exact 32.0 kΩ and 10.0 kΩ 0.1% divider resistors remain C861325 and C95204, not ordinary 1% substitutes. Required C25 high-voltage chassis capacitor and all six DB25s remain populated.

## Process, lead time and cost

Use **single-sided Standard PCBA, full BOM assembly**, with all through-hole connectors selected. JLC's catalog label Wave Soldering is a process classification; its FAQ states these listed parts may be soldered manually. The service is still factory assembly, not a request for user soldering. [JLC assembly FAQ](https://jlcpcb.com/help/article/pcb-assembly-faqs)

The Standard capability page currently lists edge rails/fiducials and an assembly build-time benchmark of at least four days. The exact paid-job schedule, procurement delays and shipping are not established. Confirm rails, depanelization and the connector overhangs in DFM; do not let a tooling change disturb the mounting supports, antenna void or connectors. Fixture need is conditional on JLC process review. If a rigid-board fixture is needed for five boards, the current 1–29 quantity tier calls for two fixtures. [Capabilities](https://jlcpcb.com/capabilities/pcb-assembly-capabilities), [fixtures](https://jlcpcb.com/help/article/pcb-assembly-fixtures)

U13's WSON has hidden joints. Budget for the system's X-ray determination; do not suppress that process to clear cost. Moisture-sensitive module/IC/LED handling, any baking, crystal assembly difficulty and the final solder process remain with JLC's quote/engineering review.

Current public catalog prices produce a **$114.33 nominal component-only subtotal for five boards**, before spares and minimum placement charges. It excludes PCB fabrication, assembly, setup, feeders, stencil, fixtures, X-ray, taxes and shipping. It is not a quote or guaranteed total.

| Main component cost | Five-board nominal subtotal |
| --- | ---: |
| ESP32-S3 module | $23.20 |
| Six DB25s per board | $23.18 |
| W5500 | $14.15 |
| MagJack | $13.75 |
| TPS3890 supervisor | $8.47 |
| Three line drivers per board | $6.83 |

Standard PCBA currently charges feeder loading for both Basic and Extended parts, so Basic class alone does not mean zero feeder cost in this build. The current price guide lists single-side setup $25.56, stencil $8.21 and $1.53 Basic/Extended feeder loading, plus manual soldering, X-ray and possible fixture charges. These are published fee examples, not a complete board quote, and differ from rounded figures in the general FAQ. Use the processed quote as authoritative. [JLC price guide](https://jlcpcb.com/help/article/pcb-assembly-price)

No redesign or connector deletion is proposed merely to reduce the quote. Public stock does not currently suggest a need to buy full reels or pre-order. If a later real shortage appears, halt and obtain an explicit exact-substitute or procurement decision. Pre-order/global sourcing is a separate purchase and was not performed; it can add MOQ, attrition, handling and lead-time constraints. [Pre-order service](https://jlcpcb.com/help/article/what-is-jlcpcb-parts-pre-order-service), [inventory combination restrictions](https://jlcpcb.com/help/article/how-to-use-my-own-parts-for-pcb-assembly-order)

## Late C3 package check

The proposed 1210 mismatch was disproved by the exact Samsung page: C15008 / CL31A107MQHNNNE is **1206 (3216)**, 3.20±0.20 × 1.60±0.20 × 1.60±0.20 mm, 100 µF ±20%, 6.3 V X5R. At C3's 3.3 V operating rail, Samsung's typical 25°C DC-bias curve interpolates to about 44.73 µF, not the nominal 100 µF. This is not a guaranteed minimum across temperature, tolerance and aging. [Exact Samsung part](https://product.samsungsem.com/mlcc/CL31A107MQHNNN.do)

The retained 1206 pads are not identical to Samsung's board-dependent land recommendation. PCB100 has gap 1.69926 mm, individual pad length 1.4859 mm and width 1.7272 mm. Samsung's 3216 ±0.20 mm table gives gap 1.64–1.76, pad length 1.19–1.31 and width 1.74–1.86 mm. The gap agrees; lands extend farther outward and are 0.0128 mm narrower than the table's lower bound. This is **not a 1210 package error**, and no geometry was changed. The design owner accepted retaining the 1206 prototype lands because this difference from the generic guide does not establish a defect. Normal assembler land/paste DFM remains appropriate. Page 48 was rendered and visually inspected. Mechanical/model clearance must use the exact part's **1.8 mm maximum height**, not an assumed 1.3 mm generic model. [Samsung MLCC catalog](https://product.samsungsem.com/resources/file/product-catalog/MLCC_2512.pdf)

Evidence: `C3_manufacturer_package_bias_20260923.json` and the archived manufacturer PDF. Do not enlarge C3 to 1210 based on the disproved package assumption.

## Remaining sourcing acceptance gate

1. Process the final r13 Gerber, normalized BOM and normalized CPL for **five boards** in a fresh Standard PCBA quote. Do not reuse an old EasyEDA-created order page.
2. Confirm **all 144 required refs / 46 SKUs**, including all six matching DB25s per board, are selected for assembly.
3. Capture each actual required quantity including spares/minimum placement and actual available/allocated quantity. Require zero unmatched, missing, Standard Only conflicts or shortfalls.
4. Check the final process/lead-time/fixture/X-ray/manual-soldering charges and orientation preview. Do not accept “do not place” for any required part.
5. Refresh inventory immediately before order/payment, under separate authorization.

The PCB remains **untested**. Factory assembly does not promise flashed firmware, a functional servo-motion test or verified cable/servo compatibility. Firmware is deliberately deferred, so do not describe the delivered prototype as a validated plug-and-run system. JLC programming is an optional reviewed service requiring the interface and programming file; neither was commissioned here. [JLC programming requirements](https://jlcpcb.com/capabilities/pcb-assembly-capabilities)

Machine-readable evidence: `SOURCING_RELEASE_20260923.json`. The companion `audit_sourcing_release_20260923.py` refreshes exact source/export and public catalog/process evidence read-only. It cannot replace the actual processed-BOM gate. No order has been submitted.
