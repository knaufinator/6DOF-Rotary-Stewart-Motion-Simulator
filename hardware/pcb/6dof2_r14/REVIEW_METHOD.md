# R14 review method and evidence limits

## Current R14C1 acceptance — 27 October 2026

Use the [R14C1 final report](review/2026-10-27-clearance/README.md) for current physical geometry: 324 components, 91 codes, 1,214 unchanged logical pins, 2,684 copper-register rows, 903 vias and 1,098 holes. All 348 board-level text envelopes and twelve internal DB25 glyph regions are checked. The earlier 26 October quantities below describe the frozen ordered baseline, not the corrected layout.

The original review missed J12's four-LINE body. Whole-assembly collision testing and manufacturer-bound screening are now mandatory separate gates; missing/open body geometry fails explicitly. The source-bound register checks dimensional maxima/conservative family bounds with 0.15 mm placement allowance, expanded body/foreign-land separation, eleven 2 mm probe shafts and six 6.1 mm mounting-head envelopes. All 324 exported STEP shapes are valid, and all 52,326 pairs are screened, with exact solid distances/intersections for nearby candidates. Source-to-STEP pose parity prevents accepting a stale export. Six regression cases cover J12’s actual closed outline, broken/missing geometry, duplicate edges and self-crossing polygons.

Rebuild all pours after copper edits, save/close/reopen the master, export a complete dated package, compare all selected parts/pin nets and inspect actual Gerber copper, drill, mask/paste and ink geometry. The R14C1 reports bind source/CAM hashes. Original manufacturing inputs remain frozen and their verifier checks integrity only. Exact D-sub nut heights and external mating/enclosure swept volumes remain unqualified; bench acceptance is still required.

## Earlier ordered-baseline electrical methodology

The review compares three representations: the intended circuit and firmware
contract, the saved/reopened EasyEDA master, and the actual manufacturing output.
A check in one representation does not replace the others. The ordered revision
is recorded in [ORDER_RECORD.json](ORDER_RECORD.json); hardware tests are pending.

## Circuit and firmware contract

Each section was checked against the selected manufacturer's pinout/reference
circuit, the actual supplier variant and the current firmware interface. The
[section report](review/2026-10-26-10V/README.md) records the findings and limits.
The registers cover 324 exact parts, 1,214 logical pins and 78 GPIO/DB25 mappings.
The review includes startup/off-state behavior, voltage limits, NC input meaning,
alarm polarity, torque clearing and the distinction between bus voltage and current.

The tested R13 laptop source was recovered with 352 verified hashes, source-ZIP
CRCs and three Git bundles. Ten exact source/host files are retained in
[R13 provenance](review/r13-recovered-source/README.md). Final-image tests and older
soaks are distinguished; only axis 0 has retained physical evidence. Later R14
changes and build variants are documented in [firmware status](FIRMWARE_STATUS.md).

## Persistence and source coherence

Changes were guarded by exact project/document identities and pre-change source
snapshots. Saves were confirmed, documents closed/reopened and fresh exports read.
Timed-out writes were reconciled before continuing; they were not blindly repeated.
Schematic/PCB supplier identities and pin/net memberships agree. Native DRC and
native comparison both return empty arrays. The source-freeze receipt excludes
only generated DOCHEAD client/time/version fields from the content comparison.

For the Q11 reel substitution, an exact record delta proves only purchasing
metadata changed. All physical/net/model/pose records and all 15 aperture-resolved
manufacturing streams match the pre-substitution audit. This binds the full
geometry review to the refreshed files without pretending unrelated tests were rerun.

## Actual copper and reference planes

Gerbers were interpreted with their declared metric 3.6 format, aperture geometry
and dark/clear polarity. Copper islands were joined across layers only through
actual plated objects. No blanket GND union was used. Every native trace/via/fill/
test-pad row has an actual-copper attribution: 2,667 rows, including 1,757 traces
and 891 vias. All 241 named physical nets / 1,080 named pads passed the connectivity
checks. No shorts, missing named-pad contacts, unintended NC contacts, unqualified
floating copper or reviewed edge/NPTH/foreign-gap violations were found.

All 344 ground through-objects join the actual Inner1 reference. High-speed trace
projections and return vias were assessed separately. Eight inherited MDI entry
segments cross intentional reference voids, longest individual unsupported length
5.37 mm. They are disclosed, unchanged by the sensor correction, and do not
constitute a full SI/EMI or controlled-impedance qualification.

## Drill, mask, paste and labels

Analytic drill review covers 1,086 unique holes including four slots and 589,155
pairs. Minimum edge gap is 0.302756582 mm against the 0.302 mm gate. The separate
via drill file duplicates 891 PTH operations; it is not an additional hole set.

All 891 via drills were checked against 1,062 SMT mask apertures and actual paste,
with zero intersections. Fifty-six inherited positive gaps fall below a 0.125 mm
review band; that band is not a claimed factory acceptance rule. Minimum actual
mask gap is 0.032061 mm. All 27 new vias exceed the band. Factory registration,
stencil/EP/paste and via-treatment acceptance remain separate process evidence.
No resin fill, capping or tenting is inferred from nullable source fields.

All 360 labels are indexed: 324 designators, 24 board strings and 12 DB25 digits.
Actual exported ink has zero mask overlap and zero off-board area under the
documented tolerance. Changed-label crops and all DB25 digits were visually read;
there is no OCR claim. The STEP has 324 component references and valid OCCT
geometry, but generic package models and unknown mated hardware limit fit claims.

## Repository artifacts and reproduction

The repository keeps one current manufacturing package, the current registers,
selected machine receipts and the explanatory documents. The earlier 26-page
report is a dated reference appendix. Duplicate baseline export sets, speculative
checkpoints, downloaded web bundles and transient solver output are not part of
the release commit. Large research intermediates remain in the local working archive.

From the repository root:

```text
python -m pip install -r tools/requirements-review.txt -r tools/requirements-docs.txt
python tools/verify_r14_order_lock.py
python tools/validate_r14_exports.py --directory hardware/pcb/6dof2_r14/exports/2026-10-26 --profile sensor-10v
python tools/render_r14_drawings.py
python tools/audit_readme_links.py --external --output readme-link-audit.json
```

The lock verifier is read-only. The export validator checks/normalizes BOM and
CPL from retained raw exports; rendering regenerates documentation drawings only.
The complete independent geometric analysis has already produced the retained
receipts; the lightweight validator does not repeat that analysis. CAD changes
require a new geometry review and new revision, not reuse of old PASS receipts.

PCB previews/PDFs are rendered from the ordered Gerbers with Gerbonara. Schematic
PDF/PNG views are rendered from the native SVG with MuPDF. The documentation
renderer explicitly handles EasyEDA's metric XYG85XY slots, which the general
Excellon reader rejected; it does not rewrite the production drill file.

## Completed checks and next measurements

Retained software checks passed 85 host C++ tests, 10 bench-host Python tests,
nine source-contract checks, separate R14/R13 ESP-IDF builds, Windows Release and
documentation-safety builds, and documentation transport guards. These establish
source/build behavior within their scope. No new R14 flash, motor command or
hardware measurement was performed during repository preparation.

The owner's order confirmation establishes that fabrication was requested.
It does not independently establish final factory allocation/orientation/process
approval or manufactured-board behavior. [BRINGUP.md](BRINGUP.md) is the next-stage
acceptance plan; record measurements, exact firmware hashes, cable/drive identity
and PASS/FAIL results there or in a dated test report.
