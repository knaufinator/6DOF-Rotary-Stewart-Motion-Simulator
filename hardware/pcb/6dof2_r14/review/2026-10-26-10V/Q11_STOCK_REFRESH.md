# Q11 stock correction — 2026-10-26

The owner's processed JLC order reported Q11 C3280148 / DMN6075S-13 with a
seven-piece shortfall. Both saved/reopened EasyEDA documents now specify
**C155242 / DMN6075S-7**. Q11 remains required and fitted.

The [manufacturer datasheet](https://www.diodes.com/datasheet/download/DMN6075S.pdf)
DS37023 Rev7-2 lists both variants as the same SOT23 device; the suffix changes
reel size (7 versus 13 inches) and packing quantity (3,000 versus 10,000).
The selected 60 V, ±20 V gate, on-resistance, pinout, land pattern, model and
rotation are retained. No circuit, routing or footprint change was made.

The [JLC product page](https://jlcpcb.com/partdetail/DiodesIncorporated-DMN6075S7/C155242)
returned available-order quantity **16,242**, versus seven requested in the
owner's actual order. This is JLC `canPresaleNumber`, mapped by its public page
to `maxInStock` / “Available Order Qty”; it is not LCSC retail inventory.
The part is Extended, SMT, Economic/Standard eligible. The retained
[sourcing receipt](Q11_SOURCING_PREFLIGHT.json) records time and raw values.
Stock remains volatile; reprocess/select C155242 in the live order and confirm
that its seven-piece request is allocated before confirmation/payment.

The first combined metadata-save call timed out after 30 seconds. A read-only
reconciliation found the new identity in both documents; separate confirmed
saves, closes and reopens then proved persistence. No metadata write was retried.
Native comparison and DRC are both empty after the change.

The [exact source/CAM delta](Q11_SOURCE_CAM_PARITY.json) proves only Q11 metadata
changed: four schematic attributes and one PCB component metadata map. Every
physical/net/model/pose record is unchanged, and all 15 aperture-resolved
manufacturing streams are identical. Thus the earlier complete CAM/drill/mask/
paste/silk review remains applicable, bound to the new exports by this proof.
The before-change project and Gerber are retained beside this note for provenance;
they are not the upload files. A fresh STEP also passes OCCT with the same solids,
faces, volume and bounds. All exports and the manifest were refreshed together.

Y1 C9006 was reported by the owner as an additional placement/assembly charge;
the owner instructed retaining it. Its source, supplier identity and placement
were not changed.
