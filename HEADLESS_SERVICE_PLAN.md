# 6DOF Platform — Headless Service Architecture & Phase‑3 Completion Plan

Status: **DRAFT / IN PROGRESS** · Owner: Chris · Started 2026‑07‑19 · Branch: `phoenix`

This plan captures a large, multi‑codebase change: split the desktop app into a
**headless backend service** (motion engine + full control API + MCP, zero GPU)
and a **detachable UI client**, and finish the half‑implemented Phase‑3 on‑device
demo/session system. It also records the smaller bridge/firmware fixes that
surfaced during live hardware testing on 2026‑07‑19.

---

## 1. Motivation

Live HW testing exposed several structural problems:

1. **Hard GPU dependency.** The app is one process where the motion/update loop
   runs on the OpenGL render thread. When a game (the real motion source, e.g.
   Assetto Corsa) saturates the RTX 4070, the app's GL context stack‑overflows
   (`nvoglv64.dll`, `0xc00000fd`) and the app dies ~7–13 s after launch. In
   production this machine runs the game **and** the motion feeder at once, so
   the feeder must not depend on the GPU.
2. **Monolithic engine + UI.** State (entities, motion, HIL streaming, cueing)
   lives in the GUI process. You can't run the motion pipeline without the whole
   GUI, and you can't attach a second view without a second engine (which would
   fight over the HIL device).
3. **Phase‑3 on‑device demo/session system is stubbed.** DEMO playback reports
   playing but produces no servo motion; burned files don't become the active
   sequence; `LOOP` is unimplemented; device file listing isn't surfaced.
4. **Tuning can't reach the device.** In raw‑LIVE the mini runs the cue engine,
   but the app's `mca_set`/`intensity`/`tilt` only write local config and the
   bridge never relays `MCA:` commands — so changing motion cue does nothing.
5. **Friction:** LIVE needed a manual Play; duplicate HIL entities from
   malformed reconnects; args‑nesting client bug (all fixed 2026‑07‑19).

## 2. Target architecture

```
                 ┌──────────────────────────────────────────┐
                 │  stewartd  (headless backend service)     │
                 │  ── motion engine (plugins → cueing →      │
                 │      HIL/SIL streaming), no GL, no window  │
                 │  ── control API  (TCP 8770, line‑JSON)     │
                 │  ── MCP server   (stewart_* tools)         │
                 │  ── runs as a Windows service / tray app   │
                 └───────────────┬──────────────────────────┘
                                 │  same TCP/JSON API
                 ┌───────────────┴───────────┐   ┌──────────────┐
                 │  UI client (ImGui/GL)      │   │  MCP / agent │
                 │  attaches on demand, thin  │   │  (Claude)    │
                 │  view+control, GPU only    │   └──────────────┘
                 │  while open                │
                 └────────────────────────────┘
```

- **One engine, many views.** The service owns all state and the single HIL
  connection. UIs (and the MCP/agent) are clients over the existing 8770 API.
- **Zero GPU when headless.** During gaming, only `stewartd` runs.
- **Full API + MCP on the service**, not the UI — the UI becomes optional.

## 3. Workstreams & phases

### Phase 1 — Bridge unblock (DONE, pending deploy) ✅
Small, no firmware flash. Gets live tuning working today.
- **1a** `control_api.py`: LIVE sends `PLAY:START` (not `PLAY:STOP`) so
  "enabled" alone moves the rig. *(done)*
- **1b** `control_api.py`: new whitelisted `cmd` verb → relays `MCA:` / `SERVO:`
  / `CONFIG:` / query strings to the mini so tuning reaches the on‑device cue
  engine. *(done)*
- **1c** commit+push → redeploy `/tmp/bridge` on Voron (192.168.1.168) → restart
  → verify one‑click LIVE + `MCA:` reaches the mini.
- **1d** live feel‑tuning pass (trim heave, lift surge/sway + tilt) via `MCA:`
  with camera as ground truth.

### Phase 2 — Firmware: finish on‑device DEMO/session (build + flash)
`mini/main/main.cpp` (ESP‑IDF). Requires build + flash (via bridge `flash_firmware`
or esptool over USB on the Voron).
- **2a** DEMO plays `laps123_moderate` (69367 samples) but rig is physically
  static — investigate PlaybackTask: is it advancing? is servo output gated by
  a state that DEMO doesn't set? is the TEL tap on the cue path (bypassed during
  baked playback)? Fix so baked playback drives servos.
- **2b** Burned file must become selectable/active: `SELECT:<name>` +
  device‑side file store; DEMO plays the selected file, not only the embedded one.
- **2c** Implement `PLAY:LOOP` (`PLAY:ERR unknown 'LOOP'` today).
- **2d** Surface device file list + `mem` as RESP events the bridge/app parse.

### Phase 3 — App: Device Demos list UI + sessions
`app/src/ui_panels.cpp`, `control_server.cpp`.
- **3a** Replace the single `sequence.m6p` text box with a **list**: local
  available sessions (recordings + `.m6p`), each with bake/upload actions.
- **3b** Show the **device's internal file list** (from Phase 2d) with
  size/active markers; select / delete.
- **3c** **Robust upload** with real progress + success/fail feedback (chunked
  handshake, CRC verify, retry).
- **3d** Generate the **3 Nürburgring session files** (currently only
  `laps123_moderate.m6p` exists — 3 laps combined). Produce gentle / moderate /
  aggressive variants (or per‑lap split) as first‑class selectable sessions.
- **3e** Resolve "Geometry out of sync with device" (push/confirm geometry).

### Phase 4 — Headless service + thin‑client UI (the large change)
The core architectural refactor. Sub‑phased to stay shippable throughout.
- **4a** `--headless` flag: run engine + control server + MCP with **no window /
  no GL**. The update/motion loop moves off the render thread into a plain loop.
  Immediate zero‑GPU backend for production gaming. *(ship this first)*
- **4b** Extract the engine into a service target (`stewartd`) that always runs
  headless; package as a Windows service / tray launcher with autostart.
- **4c** Make the ImGui app a **thin client**: it renders state fetched from the
  service over the 8770 API and sends commands, instead of owning the engine.
  Attach/detach at will; only consumes GPU while open.
- **4d** Interim GPU mitigation (do now, reversible): pin the app to the
  integrated AMD GPU via Windows per‑app graphics preference
  (`HKCU\...\DirectX\UserGpuPreferences`, `GpuPreference=1`) so the 4070 stays
  100 % for the game even before 4a lands.
- **4e** Robustness: handle GL device‑loss/reset gracefully; optional Mesa
  `llvmpipe` software‑GL fallback for a UI with no usable GPU.

## 4. Sequencing & rationale
1. **Phase 1** now — tiny, unblocks the original goal (tuning) today.
2. **Phase 4a + 4d** early — kills the GPU crash risk during any further testing
   (headless backend + iGPU pin) with low effort.
3. **Phase 2** — firmware is the gate for the demo actually working; needs a
   build/flash cycle, so batch 2a–2d together.
4. **Phase 3** — UI on top of the now‑working device file system.
5. **Phase 4b/4c** — the full service/thin‑client split, once the engine is
   proven headless.

## 5. Risks
- **Firmware flash on the live rig** (Phase 2) — keep a known‑good binary; flash
  via bridge or USB with recovery path. Verify servo safety (home/limits) first.
- **Headless engine loop** (4a) — the motion/update currently assumes render‑
  thread cadence; must re‑time it (fixed‑rate loop) without the GL vsync clock.
- **Thin‑client state sync** (4c) — 8770 API must expose enough state at a good
  rate; may need a streaming/subscribe channel beyond request/response.
- **Deploy discipline** — all changes via GitHub commit+push; bridge redeployed
  from a pushed commit (no ad‑hoc edits on the Voron).

## 6. Live status tracker
| Phase | Item | State |
|---|---|---|
| 1a | LIVE→PLAY:START | ✅ coded |
| 1b | `cmd` MCA passthrough | ✅ coded |
| 1c | deploy + verify bridge | ⏳ next |
| 1d | feel‑tuning pass | ⛔ blocked on 1c |
| 2a–2d | firmware DEMO/session | ⛔ not started |
| 3a–3e | Device Demos UI + sessions | ⛔ not started |
| 4a | `--headless` flag | ⛔ not started |
| 4b–4c | service + thin client | ⛔ not started |
| 4d | pin to iGPU | ⛔ quick, not started |

_Verified findings, decisions, and the control‑API arg contract live in the repo
history and `~/.claude` memory (`stewart-control-api-args`)._
