# R14C2 stock revision - matching engineering package

**DO NOT ORDER: static engineering checks pass; fabrication gates remain open.** Read the [review and qualification limits](../../review/2026-11-01-stock/README.md) before using this 1 November 2026 candidate. It has not been submitted to or confirmed by JLCPCB.

| File | Purpose |
| --- | --- |
| [R14-Gerber-FS36.zip](R14-Gerber-FS36.zip) | Fabrication-only four-layer Gerbers/drills, metric 3.6 format; 300 x 101.6 mm board |
| [R14-BOM-JLC.csv](R14-BOM-JLC.csv) | All 324 required components grouped into 91 supplier codes |
| [R14-CPL-JLC.csv](R14-CPL-JLC.csv) | Native Mid X/Y, side and rotation; matching 324-reference set |
| [R14-project.epro2](R14-project.epro2) | Saved/reopened editable PCB and schematic master snapshot |
| [R14-assembly.step.zip](R14-assembly.step.zip) | Native assembly STEP; all 324 component shapes checked, model limits disclosed |
| [R14-schematic.svg](R14-schematic.svg) | Fresh native full schematic sheet |
| [R14-BOM-native.tsv](R14-BOM-native.tsv), [R14-CPL-native.tsv](R14-CPL-native.tsv) | Original EasyEDA export bytes |
| [R14-schematic.net](R14-schematic.net), [R14-PCB.net](R14-PCB.net) | Fresh native netlists |
| [NATIVE_DRC.json](NATIVE_DRC.json), [NATIVE_COMPARISON.json](NATIVE_COMPARISON.json) | Empty native finding sets from matching source |
| [EXPORT_VALIDATION.json](EXPORT_VALIDATION.json) | Identity, safety-interface and package-coherence checks |
| [RELEASE_MANIFEST.json](RELEASE_MANIFEST.json) | SHA-256 binding of exports and curated review evidence |

C2 changes the Ethernet drill pattern and DB25 signal drill diameters. It requires matching C2 PCB fabrication; these BOM/CPL files cannot retrofit an already fabricated original or C1 board. Confirm factory disposition before applying C2 to an existing order.

For manual JLC assembly upload, the Gerber ZIP, BOM and CPL are **three separate inputs**. The project/STEP ZIPs are not fabrication uploads. Use a complete matching set; discard previews from older exports. For EasyEDA's integrated order route, generate a fresh preview from the saved C2 source and verify the processed BOM, allocation and placement.

The revision replaces R75/R143 with C22766, J3-J8 with C3110947 and J10 with C12074, including manufacturer-derived connector geometry and native sourcing metadata. No required part is omitted. Both source documents retain all 324 parts and 1,214 logical pins. Firmware and the corrected voltage-monitor circuit are unchanged; normal motion admission remains locked pending physical qualification.

The public inventory check covers five boards' net fitted demand, not private allocation, attrition or through-hole process approval. Ethernet stack-up/impedance and routing exceptions, exact external mechanical interfaces, inherited reduced mask gaps and the unobserved integrated-order preview remain explicit gates. See [QUALIFICATION_GATES.json](../../review/2026-11-01-stock/QUALIFICATION_GATES.json).

[R14C1 clearance files](../2026-10-27-clearance/README.md) and the [original ordered package](../2026-10-26/README.md) remain archived unchanged. Do not combine their placements, BOM or Gerbers with C2.
