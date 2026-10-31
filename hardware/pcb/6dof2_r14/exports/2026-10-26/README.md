> **27 October correction:** the original placement has confirmed J12 housing interference. See [R14C1 correction](../../MECHANICAL_CORRECTION.md) and [current review](../../review/2026-10-27-clearance/README.md). Original ordered technical artifacts stay frozen; the earlier geometry review does not approve assembly.

# R14 ordered manufacturing package — 26 October 2026

**Ordered. Hardware testing is pending.** The selected
Q11 is DMN6075S-7 / C155242; Y1 remains C9006. This package is the ordered design
baseline. Future changes belong in a new revision/export directory.

| File | Use |
| --- | --- |
| [Gerber/drill ZIP](R14-Gerber-FS36.zip) | PCB fabrication |
| [BOM](R14-BOM-JLC.csv) | Assembly identities and grouped quantities |
| [CPL](R14-CPL-JLC.csv) | Placement coordinates, side and rotation |
| [EasyEDA project](R14-project.epro2) | Editable PCB and schematic |
| [Schematic SVG](R14-schematic.svg) | Full native schematic drawing |
| [Assembly STEP](R14-assembly.step.zip) | CAD mechanical review |
| [Release manifest](RELEASE_MANIFEST.json) | File hashes and review state |

[PCB PDF](../../drawings/R14-PCB.pdf) · [Schematic PDF](../../drawings/R14-schematic.pdf) ·
[Design decisions](../../DECISIONS.md) · [Review method](../../REVIEW_METHOD.md) ·
[Order record](../../ORDER_RECORD.json)

Gerber, BOM and CPL are separate factory inputs. `JLC_UNZIP_BEFORE_UPLOAD.zip`
is only a transfer envelope; extract it first. It contains the intact three inputs,
this guide and the manifest. The EasyEDA project and STEP are not Gerber uploads.

## Source and evidence

The 324-component / 91-group / 1,214-logical-pin master was saved and reopened.
Native DRC and schematic/PCB comparison are empty. Raw UTF-16LE BOM/CPL and both netlists are retained as review/reproduction inputs.
The EPRO2 contains the native PCB and schematic; duplicate standalone document
source dumps remain in the local working archive. The normalized CSVs preserve every required reference and selected supplier code.
C78 and D9 are intentionally absent. Required THT/hybrid connectors remain fitted.

The Q11 reel change fixed the reported seven-piece shortfall. Its
[source/CAM proof](../../review/2026-10-26-10V/Q11_STOCK_REFRESH.md) establishes the
same device/package, unchanged physical geometry and identical plotted manufacturing
streams. Stock checks are dated evidence, not a promise of future availability.

The [section report](../../review/2026-10-26-10V/README.md) records actual copper,
drill, mask/paste, silk and STEP checks. Factory allocation, placement/rotation,
11 non-SMT/hybrid connector processes, J11 shell slots/paste, J12 centroid and the
inherited stencil/via questions were not independently captured through the review
API. The [order record](../../ORDER_RECORD.json) documents order confirmation;
factory acceptance has not been independently verified. No manufactured R14
qualification is claimed.

Use [BRINGUP.md](../../BRINGUP.md) when boards arrive. The supply/cable and dynamic
behavior assumptions remain acceptance items even though the PCB is ordered.
