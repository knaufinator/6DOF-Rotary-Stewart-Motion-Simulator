# R14 six-axis controller

**R14C2 stock revision implemented. Static checks pass; fabrication gates remain open.** The saved master now contains better-stock resistors, six Amphenol DB25 sockets and a Hanrun Ethernet jack. Read the [review and qualification gates](review/2026-11-01-stock/README.md) before ordering. No new factory submission or confirmation occurred here.

The ESP32-S3 controller provides Ethernet, six differential STEP/DIR outputs, independent servo-on latches, protected 12 V Hall-sensor power and direct hardware pulse inhibition. C2 preserves the R14C1 J12 clearance correction, the corrected 11.046 V nominal falling sensor-monitor threshold, Q11 **DMN6075S-7 / C155242**, every firmware interface and the recovered R13 pulse engine. Normal motion admission remains locked pending physical qualification.

[Design decisions](DECISIONS.md) · [Design philosophy](DESIGN_PHILOSOPHY.md) · [Review method](REVIEW_METHOD.md) · [Firmware evidence](FIRMWARE_STATUS.md) · [Bring-up](BRINGUP.md) · [Original order record](ORDER_RECORD.json)

## PCB and schematic

![R14C2 top copper and silkscreen from the final Gerbers](images/r14c2-pcb-top.png)

[PCB drawing: four copper layers (PDF)](drawings/R14C2-PCB.pdf) · [Top-layer SVG](drawings/R14C2-PCB-top.svg) · [Editable PCB and schematic](exports/2026-11-01-stock/R14-project.epro2)

The drawing renders actual C2 Gerbers, including drills, slots and ink. Both native documents contain all 324 production components and 1,214 logical pins. The source has been saved, closed, reopened and freshly exported; temporary staging parts are absent.

![R14C2 native schematic: sensor monitor, pulse clear and input bypass](images/r14c2-schematic-details.png)

[Schematic PDF: details and full sheet](drawings/R14C2-schematic.pdf) · [Fresh native full-sheet SVG](exports/2026-11-01-stock/R14-schematic.svg) · [Schematic netlist](exports/2026-11-01-stock/R14-schematic.net)

The crops retain native label placement and named-net stubs. Use the vector full sheet and netlist for continuations; the inherited compact blocks contain overlapping labels. C2 changes connector sourcing and J10 pin roles; the monitor, hardware clear and input bypass circuits are unchanged.

## Matching C2 engineering package

| File | Purpose |
| --- | --- |
| [Gerber/drill ZIP](exports/2026-11-01-stock/R14-Gerber-FS36.zip) | Four-layer fabrication geometry, 300 × 101.6 mm, metric 3.6 format |
| [Assembly BOM](exports/2026-11-01-stock/R14-BOM-JLC.csv) | All 324 required fitted parts grouped into 91 supplier codes |
| [Placement CPL](exports/2026-11-01-stock/R14-CPL-JLC.csv) | Native Mid coordinates/rotations; matching reference set |
| [EasyEDA project](exports/2026-11-01-stock/R14-project.epro2) | Saved/reopened master snapshot, PCB and schematic |
| [Assembly STEP](exports/2026-11-01-stock/R14-assembly.step.zip) | All 324 named component shapes checked; model limits disclosed |
| [Export guide and manifest](exports/2026-11-01-stock/README.md) | File roles, SHA-256 hashes and release gates |

R75/R143 use **C22766**, J3–J8 use **C3110947 / L77SDB25SOL2**, and J10 uses **C12074 / HR911105A**. The [public inventory screen](review/2026-11-01-stock/inventory/INVENTORY.csv) records five-board net fitted demand across all 91 codes; private allocation, attrition and through-hole assembly handling remain unverified. Amphenol signal contacts are rated 3 A. Its required 1.10 mm signal drills and the Hanrun A/6 grid are persisted in the master.

Manual JLC PCBA upload needs the Gerber ZIP, BOM and CPL as separate inputs. Use the complete C2 set and a fresh order preview. Public stock and clean DRC do not approve manufacture. Ethernet stack-up/impedance and entry-void/layer-transition exceptions, external mounting/hood/panel fit and inherited reduced mask gaps remain [explicit gates](review/2026-11-01-stock/QUALIFICATION_GATES.json).

## Review and qualification

The [C2 report](review/2026-11-01-stock/README.md) covers all 324 body outlines, 52,326 component pairs, 1,121 drilled features, 348 board-text envelopes, actual mask/paste/ink and four-layer copper. Fresh native DRC and schematic/PCB comparison are empty. Final measured planar MDI pair mismatch is below 0.001 mm; this does not establish electrical delay matching, 100-ohm impedance or SI/EMI qualification.

The recovered tested R13 laptop firmware remains integrated; retained physical evidence covers one axis. Firmware is unchanged by C2. `MotionAdmission<false>` stays locked. No R14 powered tests have run. Follow [BRINGUP.md](BRINGUP.md), including the actual 15 ft sensor cable, voltage/hysteresis measurements, independent reset/fault/clear and pulse captures, then unloaded axis tests.

## Revision history

The [R14C1 mechanical correction](MECHANICAL_CORRECTION.md) fixed the confirmed J12 collision without removing J12. Its [complete files](exports/2026-10-27-clearance/README.md) and [review](review/2026-10-27-clearance/README.md) remain archived unchanged. The [28 October Amphenol study](review/2026-10-28-amphenol/README.md) supplied six-position model evidence; C2 now applies the replacement with new drills and sourcing metadata, while retaining its height/hardware qualification limits.

The [original ordered package](exports/2026-10-26/README.md) and `r14-ordered-2026-10-26` tag are preserved. Its confirmed J12 collisions still prevent approving that original placement. Trial routing and API debugging files remain outside the curated repository.
