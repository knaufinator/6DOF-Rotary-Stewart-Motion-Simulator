# Legacy hardware files (superseded)

Everything in this folder predates the current controller board, the EasyEDA Pro design
**"6DOF 2"** (r13 RC4 untested prototype handoff; see [`../6DOF2_BOARD.md`](../6DOF2_BOARD.md)):

- `pcbv2_mainboard_bom.md`, `pcbv2_easyeda_missing_parts.md` - the earlier through-hole carrier
  concept (DevKitC-1 + W5500 module + SN75174N + LM7805). Not built.
- `ProPrj_6dof2_*.epro`, `*.zip`, `*.eext` - EasyEDA Pro project snapshots and extension from the
  first automation attempts (2026-02).
- `build_*.js`, `wire_*.js`, `eda_bridge*.{js,py}`, `fix_*.js`, `layout_v2.js`, `gen_nets.py`,
  `pin_map.json`, `*.ps1` - bridge/automation scripts of that era. The live toolchain now lives
  with the board in `G:\My Drive\projects\6dof\pcb\easyeda\`.

Kept for history only; nothing here is a current manufacturing input. No r13 order
has been placed. See the [hardware overview](../../README.md) for current files.

## Git preservation policy

Legacy design snapshots, extracted EasyEDA source, BOM notes and automation
scripts are eligible for tracking; obsolete does not mean disposable. The root
ignore rules no longer hide this entire directory. These files remain history,
not current manufacturing inputs.

Only `easyeda-mcp/` and `pro-api-sdk/` remain explicitly excluded here. These
separate embedded Git checkouts contain local modifications that are not backed
up by a parent-repository commit. Preserve those changes independently before
moving or cleaning the checkouts; do not delete the directories or accidentally
add unconfigured Git submodules. The `node_modules/`,
`dist/` and packaged `build/dist/` outputs are generated, but
`build/packaged.ts` is source, not a disposable build artifact.
