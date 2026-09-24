# r13 RC4 sourcing evidence rebind

Status: **PASS for exact source/export identity; DO NOT ORDER pending actual JLC allocation and complete-assembly acceptance.**

This additive report preserves the original RC1 catalog audit. It binds its exact supplier identities to PCB202, schematic189 and the actual saved RC4 raw/normalized BOM/CPL. **495 checks pass**: all 144 references, all 46 supplier codes, every MPN and required-part classification are unchanged. Relative to the original source, only J3-J8 footprint associations changed to the guarded 3.20 mm flange-hole correction; their selected C77833 parts did not change. The RC3-to-RC4 PCB delta is only the board display cache, not electrical geometry or metadata.

## What is and is not current

The original complete catalog/process screen ran September 23, 2026 at 13:30-13:34 UTC. Those recorded stock/availability indicators remain timestamped evidence, **not a fresh stock query, reservation, actual assembly allocation or guaranteed lead time**. This rebind does not silently refresh their timestamps. Recheck the actual five-board processed BOM immediately before any purchase.

Standard PCBA remains required for ESP32-S3-WROOM-1-N8R2 / C2913204; its primary product page was also reopened during final documentation review on September 23. [Exact JLC ESP32 page](https://jlcpcb.com/partdetail/3198302-ESP32_S3_WROOM_1N8R2/C2913204).

All 144 components are required: 135 SMT and nine THT per board, or 720 nominal placements and 45 THT placements for five boards. Six C77833 DB25s per board require 30 nominal connectors before actual JLC attrition. [Exact C77833 JLC page](https://jlcpcb.com/partdetail/CONNFLYElec-DS103425FUNSI44/C77833). No DNI, optional removal, customer hand-install or automatic substitution is approved.

## Exact final bindings

| Artifact | SHA-256 |
| --- | --- |
| `202_final_pcb_capture.json` | `4bcbd7b23f445253f7220278fb7b193aefef3ee5d6346630237d9ce98210919f` |
| `189_schematic_rc3_capture.json` | `957dde997969d646cb0445e567bfcc4813728e97eac44e4f4219773319581589` |
| `6DOF2_r13_BOM_raw.csv` | `dbd461331c67ed3d60fe169285e394695a01cfa82c8c1f69030078e83ecb4a60` |
| `6DOF2_r13_CPL_raw.csv` | `7e244c74fb5a56a9ce55da0b6bd23070444537a424daf0370836e02ca9a72950` |
| `schematic_contract.json` | `f76b27d5453ba416d50dfb66c89edc524c12c891f1df09d4fc0ce3941c5da6fb` |
| `204_rc4_archive_independent_audit.json` | `01dc529047afe9a03e85d28fce7f638c4a063a6c5028585f34c4c647bc039434` |
| `6DOF2_r13_BOM_JLC.csv` | `5c7ce157d1b4507a3196eff41fd2579f1ca568a7fc468eeb57c241228d70fffc` |
| `6DOF2_r13_CPL_JLC.csv` | `43e663b34e681b8c1de7c1f2084aa8874206f02b15479567abcc260b41c70f4d` |

The six DB25 +180 degree CPL offsets are applied exactly once in the normalized CPL. Both normalized CSV hashes are unchanged from RC3: the final display-cache repair does not change part identity, physical placement or rotation. Actual JLC assembly preview remains a separate gate.

## Minimal evidence to include

- `SOURCING_RC4_REBIND_20260923.json` and this report: all 144 / 46 identities and current hashes, with explicit no-allocation limitations.
- `SOURCING_RELEASE_20260923.json` and `.md`: original primary catalog/process URLs, timestamps, raw records and quantities for all 46 SKUs.
- `r13_rc4_cpl_normalization_contract.json`, `rc4_jlc_csv_normalization.json`, `rc4_saved_csv_independent_audit.json`: final source/raw/normalized identity and orientation proof.
- `172_db25_source_preservation_audit.json`, `189_rc3_presentation_independent_audit.json`, `202_rc4_board_display_cache_independent_audit.json` and `204_rc4_archive_independent_audit.json`: exact final footprint/source scope.

## Remaining order gates

1. Process the final BOM/CPL for five fully assembled boards: all 144 references / 46 SKUs selected, zero unmatched parts or shortfalls including actual attrition/minimums.
2. Confirm Standard PCBA accepts every required THT/hybrid/shield joint and any fixture/X-ray requirements. No omitted connectors for the owner to solder.
3. Inspect actual JLC pin-1/body orientation, especially all six DB25s; do not apply a second 180 degree correction.
4. Obtain owner authorization for the purchase. Nothing has been ordered, reserved, paid for or communicated to suppliers by this audit.

This is an untested prototype. Factory assembly does not implement the deferred firmware, commission servo motion, or certify an E-stop/STO/brake system.
