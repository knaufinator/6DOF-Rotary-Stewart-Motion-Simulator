# R14C1 replacement exports — 27 October 2026

**Prepared and statically reviewed; factory acceptance and physical testing pending.** This complete replacement set corrects the ordered J12 housing interference while retaining all 324 parts and the existing circuit/firmware interfaces. See the [correction](../../MECHANICAL_CORRECTION.md), [final review](../../review/2026-10-27-clearance/README.md) and [hash manifest](RELEASE_MANIFEST.json).

| File | Use |
| --- | --- |
| [Gerber/drill ZIP](R14-Gerber-FS36.zip) | Fabrication authority: corrected four copper layers, outline, mask, paste, silkscreen and drills |
| [BOM CSV](R14-BOM-JLC.csv) | 91 exact supplier groups, all 324 required components; no manual omission of J12 |
| [CPL CSV](R14-CPL-JLC.csv) | Matching corrected placements; use with these Gerbers |
| [EasyEDA Pro project](R14-project.epro2) | Authoritative editable PCB and schematic snapshot; native sources are embedded |
| [Assembly STEP ZIP](R14-assembly.step.zip) | Full 324-part corrected assembly, all-pair interference checked |
| [Native schematic SVG](R14-schematic.svg) | Fresh circuit drawing; unchanged electrical design |
| [Export validation](EXPORT_VALIDATION.json) | Identity/pin/BOM/CPL/STEP inventory coherence |

Native BOM/CPL TSV files retain EasyEDA's original export bytes. The JLC CSVs normalize columns/units and group by supplier code without changing component identities. PCB/schematic `.net` files and native DRC/comparison JSON are review evidence. The EasyEDA archive contains the full native documents, so duplicate standalone source dumps are omitted from this curated package. Filenames retain the R14 family prefix; the manifest and board marking identify R14C1.

The factory must accept and review this complete set before proceeding. If original boards are already fabricated, changing the CPL cannot relocate their pads; new fabrication is required. No factory upload, hold or message was performed here. The [original ordered package](../2026-10-26/README.md) remains frozen and is superseded for assembly.

Readable [four-layer PCB PDF](../../drawings/R14C1-PCB.pdf), [top SVG](../../drawings/R14C1-PCB-top.svg) and [schematic PDF](../../drawings/R14-schematic.pdf) accompany the native exports. Internal board spacing passes the documented limits. External connector/enclosure fit, process exceptions and physical R14 qualification remain as described in the report.
