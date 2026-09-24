# Copper migration: saved offline plan, NOT APPLIED

Paused for the user's computer shutdown request. No live EasyEDA calls or copper mutations were made by the sourcing agent.

`copper_migration_plan.json` enumerates 60 track, 25 via and 48 fill deletions with exact IDs, captured geometry and reasons. Four replacement prefixes preserve established long USB and VCC5 paths while removing their old direct contacts. Validate every ID, net and geometry against the live post-placement PCB before applying; reconcile differences instead of blindly retrying.

`copper_migration_contact_audit.json` passes an independent continuous-geometry overlap check: after the planned deletions and prefix reuse, no remaining tracks, vias or top fills physically contact the eight fixed retagged U2/U3/U4/U5 pads or the old pad sites of D3/U7/C3/C6/C7. This does not model regenerated native planes or prove full clearance, connectivity or routing completion.

Key findings:

- U5 pin4 was a VCC5 junction for two long trunks. Removing only its short stub would leave VCC5 shorted to DRIVE_ENABLE. The plan shortens both trunks by 150 mil near that pad, preserving their remaining geometry for reconnection to the drive supply.
- U3/U4/U5 pin12 GND ties are removed only to their first shared ground junction. Shared ground vias and unrelated routes are preserved.
- All six old U7 thermal vias and both old rectangular VCC3V3 heat-spreader fills are removed. Explicit circular via covers are independent objects; every captured cover associated with a removed via is included. Some vias have zero or two explicit covers rather than four, which is preserved in the evidence.
- The long D6-to-old-D3 VBUS branch is removed; seven local USB connector/ESD tracks and three local vias are explicitly preserved.
- Only the last MCU-approach records of the USB pair are replaced. Prefixes retain established upstream geometry; the remaining approach must go through R69/R70, followed by short separate MCU-pin nets.

Unfinished, required next work:

1. Guarded live application of the deletion/prefix plan, only after a fresh checkpoint and comparison.
2. Reroute all moved/new components and changed nets. Stationary R18.1 (VCC5) and R19.1 (VCC3V3) lose old shared regulator spurs and specifically need reconnection.
3. Reconnect the shortened VCC5 trunks to U5.16 or another drive-supply point, never U5.4.
4. Finish the USB pair via R69/R70 and check pair geometry/length/skew and native DRC.
5. Build thermal copper at the new U7 location, rebuild planes and validate actual copper coverage/clearance after save/reopen.
6. Full connectivity, native DRC, source persistence, visuals and manufacturing-release gates. No files from this working directory are order-ready.

Reproduction scripts: `build_copper_migration_plan.py` and `validate_copper_migration.py`. The latter requires Shapely and was run using `uv run --with shapely python validate_copper_migration.py`. The plan is tied to the SHA256 hashes of `09_capture_pcb.json` and `placement_plan_r13.json` recorded inside it.
