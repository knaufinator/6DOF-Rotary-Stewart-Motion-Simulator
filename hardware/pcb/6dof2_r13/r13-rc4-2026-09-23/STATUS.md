# 6DOF2 r13 RC4 — verified prototype files

**DO NOT ORDER. UNTESTED PCB. No order has been submitted.**

The routed design and fresh manufacturing exports passed the local checks below. This is a prototype file package, not a validated, programmed or ready-to-operate motion controller. The user's explicit no-order instruction remains in force.

Frozen source: PCB202 / schematic189; native exports204–206; independently audited import212. Board intent: **170 × 200 mm, nominal 1.6 mm, approximately 1 oz copper on all four layers**. Layer order: **Top / Inner1 GND / Inner2 signal / Bottom**.

The saved CAD records specify about 35 µm for each copper layer, but only one 1.510 mm FR4 dielectric record—not all three dielectric spacings of a qualified four-layer stack. Copper plus FR4 sums to 1.6500 mm; masks bring the exported model to about 1.67005 mm. Restored layer records prove persistence, **not** a valid JLC fabrication or controlled-impedance stack. Obtain the accepted JLC stack/copper construction and review actual reference-layer spacing/impedance before ordering.

| Check | Verified result |
| --- | --- |
| Saved PCB | 144 components, 654 pads, 967 tracks, 398 vias, 689 static fills, 8 regions and one native GND plane. |
| DRC and recovery | Zero native errors at203 and after RC4 import212. Imported active PCB/schematic records, 11 physical-layer records, 125 rule selectors and the 144-component/640-logical-pin netlist match exactly. |
| Outline/display | Stale editor `BOARD` background corrected without changing other source geometry. Fresh Gerber/STEP remain correct. Exact JLC menu preflight210 reached confirmation without the missing-outline warning, then was cancelled. |
| Fabrication | Strict CAM passes: 11 Gerbers, metric3.6 format, one closed outline and 581 source-matched unique drills/slots. All twelve DB25 mounting holes export as 3.20 mm NPTH. |
| Assembly files | 46 BOM rows cover all 144 required components; 144 Top-side CPL rows. All six matching Connfly DB25s retained. Saved-file identity/orientation checks pass. |
| Actual 3D | All 144 real component assemblies/861 component solids present; 435 model/placement checks pass. Fresh top/bottom/isometric views inspected. PCB and component geometric inventory matches RC3 in 147 checks. |

Fresh RC4 painted Gerber geometry/drills exactly match RC3; all 11 CAM PNG witnesses are byte-identical. Source/cache/plane checks also pass. See `evidence/cam_step/` and `mechanical/CAM_STEP_MECHANICAL_REVIEW.md`; the final manifest binds every delivered file.

## Remaining gates

- **JLC stackup, allocation and assembly review:** Confirm the actual four-layer dielectric/copper stack and data-pair impedance as above. Recheck stock for the actual quantity—five boards require 30 DB25s. Allocate every required SKU and confirm JLC assembly of SMT **and** all through-hole connectors, including DB25s and MagJack. Review the separately uploaded Gerber, BOM and CPL, placement/polarity and manufacturer DFM feedback. The local outline preflight is not an order, allocation or assembly-preview approval.
- **Firmware and bench testing:** Firmware is deferred. USB-alone programming is intended but untested; 12 V is not required for programming. Verify power isolation/rails, USB enumeration, reset/output-disable latch, GPIO47 inhibit, GPIO48 fresh-arm-edge behavior, servo signals, communications and temperatures using current-limited bench power before any powered platform connection. No certified E-stop/STO claim is made.
- **Mechanical qualification:** Use all six board mounts and insulating local support during cable mating, especially the middle DB25 row. Connector hardware/hood fit, strain relief, insertion force, MagJack plug/latch access, antenna clearance and relocated-U7 ventilation remain untested. The legacy DB25 model does not certify the exact Connfly hardware. C3 may be 1.8 mm high although its generic model is 1.3 mm. The old r12 enclosure is not qualified for this revision.
- **Prototype validation:** No assembled board exists from this release. Signal integrity, EMC/ESD, sustained thermal/current behavior and real servo compatibility are not proven by DRC or model clearance.

Use only this RC4 package after the remaining gates and explicit order authorization. Earlier RC1–RC3 exports or browser order pages are not substitutes.
