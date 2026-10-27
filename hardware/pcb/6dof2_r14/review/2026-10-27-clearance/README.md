# R14C1 final clearance review — 27 October 2026

**Internal board geometry and exported electrical checks pass. Factory disposition and physical qualification remain pending.** This supplements the [26 October section-by-section circuit/firmware review](../2026-10-26-10V/README.md); the electrical design, supplier identities, 1,214 logical pin memberships and firmware are unchanged. The original placement is superseded because of the confirmed J12 collisions.

![Physical-body correction](../../images/r14c1-clearance-before-after.png)

## Results

| Scope | Coverage and result | Evidence |
| --- | --- | --- |
| Every selected part | 324 parts, 91 codes; reviewed maximum/conservative dimensions, source links and qualifications | [Component register](every-component-dimensions.csv), [bounds](BODY_LIMITS.json) |
| Every native outline, double checked | 324 parts / 57 used footprints independently reconstructed: 321 POLY, two FILL, one four-LINE body; zero missing, open, degenerate, self-crossing or unsupported bodies | [Outline audit](ALL_BODY_OUTLINES_REVIEW.json), [every component outline](every-component-outline.csv) |
| Native/STEP mechanical | All native outlines covered, all 324 STEP solids valid, 52,326 pairs screened, zero intersections | [Mechanical](MECHANICAL_REVIEW.json), [original failures](ORDERED_COLLISION_FINDINGS.json) |
| Tolerances and access | Expanded body/foreign-land spacing; all 11 test pads and six mounting sites pass | [Internal clearance](INTERNAL_CLEARANCE_REVIEW.json) |
| Master/export persistence | 30 moved / 294 unchanged STEP poses match the native master within 0.0001 mm | [STEP parity](STEP_POSE_PARITY.json), [placement changes](placement-changes.csv) |
| Circuit/firmware preservation | Same parts, footprints, sides, rotations and logical pin nets; controller diff against ordered tag empty | [Electrical/firmware parity](ELECTRICAL_FIRMWARE_PARITY.json) |
| Actual copper | 241 nets / 1,080 named physical pads; no missing pads, shorts, opens, unqualified floating groups or foreign-gap findings | [CAM](ACTUAL_CAM_REVIEW.json), [2,684 source copper rows](every-trace-via-fill.csv) |
| Ground planes | All 340 ground through-objects globally joined and directly connected to main Inner 1; six pours | [Ground proof](GROUND_JOINS.json) |
| Holes and slots | 1,098 unique holes, 602,253 analytic pairs; minimum gap 0.3027566 mm, unchanged inherited minimum | [Drill proof](DRILL_REVIEW.json) |
| Mask/paste | All 903 vias against 1,062 SMT lands; zero opening intersections | [Aperture review](APERTURE_REVIEW.json) |
| Labels and ink | 348 conservative board-text envelopes pass; 62 relocated designators checked against source; complete actual ink has zero mask overlap/offboard ink; twelve DB25 glyphs visually read | [Text envelopes](LABEL_ENVELOPE_REVIEW.json), [label register](board-labels.csv), [actual ink](SILK_REVIEW.json), [glyph image](DB25_DIGITS.png), [visual review](VISUAL_REVIEW.json) |

## Section continuity

Power entry/protection, 12 V Hall-sensor feed and voltage monitor, hardware pulse inhibition, six differential STEP/DIR outputs, six servo-on latches, alarms, Hall conditioning, MCU/Ethernet, UART and auxiliary regulation retain the ordered circuit. [Pin-by-pin coverage](../2026-10-26-10V/every-pin.csv), [interface contracts](../2026-10-26-10V/interface-contract.csv) and [voltage analysis](../2026-10-26-10V/VOLTAGE_CONTRACT.json) remain applicable because all component pin nets and selected parts are proven unchanged. Routing/placement evidence in this folder supersedes the earlier geometry evidence.

Sensors still receive **12 V**. Their 10 V minimum is the remote operating limit; the corrected monitor falls at nominal 11.046 V, allowing conditional cable/board loss. The earlier analysis assumes hot cable loop ≤2 Ω and board path ≤0.4 Ω; the owner's 15 ft Ethernet cable gauge/resistance remains unmeasured. This is not a measured R14 power guarantee. Motion admission remains locked pending [bench qualification](../../BRINGUP.md).

## Material limits

The exact D-sub nut-stack heights for J3–J8, mated hoods/plugs, cable routing and enclosure swept volumes are not qualified. Internal XY separation does not depend on those unresolved heights. The LTV-847S supplier package is checked from its drawing; its generic STEP marking alone does not establish part identity.

Fifty inherited vias have positive mask/paste gaps below the 0.125 mm review band (minimum mask 0.032061 mm, paste 0.032569 mm). None intersects an opening; this is not a new stencil/process approval. Eight inherited Ethernet MDI entry segments have reference-plane limitations (maximum about 5.37 mm); unchanged source geometry and [reference evidence](SIGNAL_REFERENCES.json) do not establish SI/EMI qualification. Physical fit, soldering, temperature, cable loss, reset/fault behavior and pulse timing still require hardware measurements.

## Reproduction and provenance

The repository tools `audit_pcb_outline_coverage.py`, `verify_pcb_body_bounds.py`, `audit_pcb_mechanical.py`, `validate_r14_exports.py` and `test_pcb_body_coverage.py` reproduce the maximum-body, exact STEP, export-identity and LINE-loop regression checks. Install `tools/requirements-review.txt`; unzip the assembly STEP or pass its ZIP directly to the mechanical tool. Run from repository root, pointing `--source` to the R14C1 project, `--bounds` to this folder's BODY_LIMITS.json, and pass the resulting receipt as `--internal-clearance` to the STEP audit. The actual-CAM receipts record script/source hashes; their full research pipeline and downloaded datasheets remain in the local working archive, so those scripts are not represented as a portable repo command. Manufacturer links and reviewed document hashes are in the bounds register. A blocked download is not a fresh datasheet verification.

The [replacement export manifest](../../exports/2026-10-27-clearance/RELEASE_MANIFEST.json) binds the artifacts. The original ordered tag remains fixed. No factory submission or R14 hardware-test result is implied by these static checks.

## Repository link check

The final audit checked 30 READMEs, 360 link occurrences and 44 unique external URLs. Owned local links and anchors pass. One obsolete Worldsemi URL returns 404 in two vendor-managed LED-strip READMEs; those vendor files are unchanged. The stewart-core GitHub URL and ST LDL1117 PDF could not be freshly verified because of a network error/timeout. These exceptions are retained in the [link audit](README_LINK_AUDIT.json), rather than reported as successful checks.

For an independent outline audit, run `tools/audit_pcb_outline_coverage.py --source hardware/pcb/6dof2_r14/exports/2026-10-27-clearance/R14-project.epro2 --output outline-review.json --csv component-outlines.csv`. It reconstructs the actual original LINE segments with polygonize_full, checks explicit polygon closure/validity/nonzero area, and enumerates every fitted component. Six regression tests include the actual J12 body and malformed or missing geometry.
