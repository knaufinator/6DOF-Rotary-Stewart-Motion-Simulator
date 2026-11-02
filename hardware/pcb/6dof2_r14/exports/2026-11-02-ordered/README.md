# R14C2 ordered snapshot — 2 November 2026

**Ordered; awaiting physical testing.** Order placement was confirmed in the conversation. This fresh capture of the unchanged EasyEDA master records the prepared design; the factory's exact uploaded files, order identifier, quantity and allocation were not independently captured. See the [order record](../../ORDER_RECORD_C2.json) and [snapshot review](../../review/2026-11-02-ordered/README.md).

| File | Purpose |
| --- | --- |
| [Gerber/drill ZIP](R14-Gerber-FS36.zip) | Fabrication input; four layers, 300 × 101.6 mm, native metric 3.6 coordinates |
| [JLC BOM](R14-BOM-JLC.csv) | Assembly input; 324 required fitted parts, 91 supplier codes |
| [JLC CPL](R14-CPL-JLC.csv) | Assembly input; native Mid coordinates, rotations and sides |
| [EasyEDA project](R14-project.epro2) | Editable PCB, schematic and cached libraries |
| [Assembly STEP ZIP](R14-assembly.step.zip) | One complete native STEP with all 324 component identities |
| [PCB PDF](R14-PCB.pdf) | Four copper layers, rendered from this Gerber ZIP |
| [Schematic PDF](R14-schematic.pdf) | Three circuit detail pages followed by the complete native sheet |
| [Native schematic SVG](R14-schematic.svg) | Full vector sheet; inherited source label placement retained |
| [Raw BOM](R14-BOM-native.tsv) / [raw CPL](R14-CPL-native.tsv) | Unmodified native assembly evidence |
| [Schematic netlist](R14-schematic.net) / [PCB netlist](R14-PCB.net) | 1,214 logical pins in both documents |
| [DRC](NATIVE_DRC.json) / [comparison](NATIVE_COMPARISON.json) | Fresh native results, both empty |
| [Export validation](EXPORT_VALIDATION.json) / [SHA-256 manifest](RELEASE_MANIFEST.json) | Inventory and file integrity |

Gerber, BOM and CPL are separate JLC upload inputs. The project, STEP and PDFs are reference files. No identity, CPL rotation or position patch was applied: R75/R143 use C22766, J3–J8 C3110947 / L77SDB25SOL2, and J10 C12074 / HR911105A in the master.

Exact non-header source records, aperture-resolved manufacturing commands, netlists and assembly tables match the [reviewed C2 candidate](../2026-11-01-stock/README.md). Container timestamps and STEP entity numbering can differ. The new STEP receives a separate validity/interference review. The existing [PCB preview](../../images/r14c2-pcb-top.png) remains representative because the manufacturing commands are identical.

The sensor monitor, retained J12 correction and recovered R13 firmware are unchanged. Normal R14 motion admission remains locked. Ethernet signal integrity, external connector fit, assembly processing and powered fault behavior still need the [physical acceptance checks](../../BRINGUP.md). Neither this order record nor clean static checks establish hardware qualification. Preserve these frozen inputs; use a new dated snapshot for subsequent changes.
