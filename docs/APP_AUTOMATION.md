# Desktop App Automation and Safe Documentation Captures

The repository-local MCP bridge is [`app/tools/stewart_mcp.py`](../app/tools/stewart_mcp.py). It exposes the native application's loopback JSON control API over MCP stdio. It is separate from the optional Linux HIL bridge in `app/bridge/`.

**Verified host: Windows.** The Python bridge uses portable paths, but the desktop control server is currently implemented only for Windows (`ControlServer::start` returns false on other platforms). A POSIX Python environment alone does not make this app-control workflow available on Linux/macOS.

The tools are for inspecting and driving the actual app, not generating mock screenshots. The r13 PCB/firmware combination remains **untested**; documentation captures are not hardware or commissioning evidence. See the [app guide](APP_GUIDE.md) and [firmware gates](../hardware/pcb/6DOF2_FIRMWARE_TODO.md).

## Setup

Use Python 3.10 or newer. Install the pinned requirements into a project-local virtual environment, not the system Python. These are setup instructions; documentation editing does not require running an installation.

From the repository root, when ready to install:

```powershell
python -m venv app/tools/.venv
app/tools/.venv/Scripts/python.exe -m pip install -r app/tools/requirements.txt
```

This `.venv/` location is already ignored by Git. The current requirements file pins the MCP SDK; use that file as the dependency source of truth. On POSIX, the corresponding Python path would be `app/tools/.venv/bin/python`, but the desktop control-server limitation above still applies.

The app must be built with the current automation changes. An optional separate build directory keeps documentation compilation separate from an existing build:

```powershell
cmake -S app -B app/build-docs
cmake --build app/build-docs --config Release
```

A first configure may need network access for the app's CMake dependencies. Follow [BUILD.md](BUILD.md) for toolchain prerequisites. Do not reuse a cache from an old repository location.

### MCP client configuration

For clients using the common `mcpServers` JSON format, adapt this entry to your checkout. Replace the placeholder paths, and make `command` the virtual environment's Python executable if it is not already the active `python`.

```json
{
  "mcpServers": {
    "stewart-platform": {
      "command": "python",
      "args": ["C:/path/to/6DOF-Rotary-Stewart-Motion-Simulator/app/tools/stewart_mcp.py"],
      "env": {
        "STEWART_APP_EXE": "C:/path/to/6DOF-Rotary-Stewart-Motion-Simulator/app/build-docs/Release/stewart-platform.exe",
        "STEWART_CTRL_PORT": "8772"
      }
    }
  }
}
```

The server command and environment are portable; the enclosing configuration syntax depends on the MCP client. Register it using that client's supported format. Starting the MCP server alone does not launch the app: call `stewart_launch` afterward.

| Environment variable | Meaning / default |
|----------------------|-------------------|
| `STEWART_APP_EXE` | App executable; defaults to `app/build/Release/stewart-platform.exe` relative to the bridge's inferred app directory |
| `STEWART_CTRL_PORT` | Loopback control port, default `8770`; use a spare port such as `8772` to avoid a normal running app |
| `STEWART_APP_DIR` | Working directory for an explicitly requested normal-mode launch; defaults to repository `app/` |
| `STEWART_SESSION_DIR` | Parent for unique empty documentation sessions; defaults to `stewart-docs-sessions` under the OS temporary directory |
| `STEWART_SHOT_DIR` | Default screenshot destination directory; defaults to the OS temporary directory |

No personal settings, recordings, plugins or workspaces are copied into a documentation session.

## Documentation mode is a launch policy

`stewart_launch()` defaults to `documentation_mode: true`. It sets `STEWART_DOCUMENTATION_MODE=1` **before** creating the process and gives it a unique empty working directory. The app reads that policy once; there is intentionally no runtime switch back to hardware-enabled operation.

In this mode:

- Serial enumeration/opening and hardware UDP transport startup are blocked before device/network I/O; HIL entities remain offline.
- HIL connection/command/streaming and geometry/MCA push operations are rejected. Live plugin activation and parameter changes are blocked.
- Normal user settings, recordings, presets and workspaces are not loaded or persisted. The app does not scan user plugins or start its hardware TX thread.
- ImGui layout persistence and secondary viewports are disabled so the app framebuffer represents the docked documentation window.
- Sequence-file export is blocked. Explicit screenshots and runtime diagnostics can still be written. Screenshot targets require a `.png` suffix, and existing non-PNG files cannot be overwritten. This is hardware-disabled operation, not a filesystem sandbox; choose output paths deliberately.

The ordinary app is **not** documentation-safe merely because the MCP bridge exists. A manually launched app without `STEWART_DOCUMENTATION_MODE=1` is normal mode. `stewart_launch(documentation_mode=false)` is an explicit opt-in to normal operation and can expose commands that move connected hardware; it is not part of this capture workflow.

When a service already occupies the selected port, the bridge checks the app identity and mode. It refuses to attach a documentation request to a normal/older app, and does not kill or restart any process automatically. Use a separate port or close the conflicting app yourself.

The control API binds to `127.0.0.1`, not a remote interface. Treat it as a trusted-local-process interface; do not forward the port to an untrusted network.

## Inspect, configure, then capture

Typical MCP tool sequence:

1. `stewart_launch(documentation_mode=true)`; require `running: true` and `documentation_mode: true`.
2. `stewart_capabilities()`; require `hardware_io_allowed: false`. Discover command and UI enum names from the response.
3. `stewart_docs_fixture()`; this creates labeled in-memory synthetic recordings, not hardware measurements.
4. `stewart_recordings_list()` and `stewart_entities()`; use returned names and IDs rather than guessing.
5. `stewart_play(recording="SYNTHETIC - Six-axis demo", loop=true, speed=1.0)` to process the synthetic samples through the real playback/filter/MCA/IK pipeline.
6. `stewart_ui_set(configuration=...)`, followed by frame progression and `stewart_ui_get()` until the requested layout is rendered.
7. `stewart_screenshot(path="C:/absolute/existing-directory/overview.png")`; inspect the actual image and returned capture metadata.
8. Stop playback with `stewart_stop()`. `stewart_quit()` requests a clean exit only when the caller is finished with this instance.

Example presentation-only configuration, using an ID obtained from `stewart_entities()`:

```json
{
  "layout": "geometry",
  "entity": 0,
  "entity_tab": "Platform",
  "platform_tab": "Geometry",
  "panels": {"console": false, "data": false},
  "input_expanded": false,
  "camera": {"azimuth": 0.8, "elevation": 0.5, "distance": 1500, "split": 0.6}
}
```

Entity `0` is illustrative, not guaranteed. Camera angles are radians; the split value allocates viewport/readout space. Supported layout, tab, panel and range values must be checked against `stewart_ui_get()` and the current API.

For the home-height measurement view, `stewart_ui_set(configuration={"entity": 0, "measure_step": 4})` selects **Platform → Measure → Home Height**. Replace the entity ID. `measure_step` accepts `0..6`; `ui_get().supported.measure_steps` reports the names. Conflicting explicit tab selections are rejected.

After changing live configuration through the API, `stewart_ui_set(configuration={"entity": 0, "dynamics_reload": true})` explicitly discards that entity's staged Dynamics edits and reloads its live values, like **Revert**. Use it only when discarding the staging buffer is intended. This is not an automatic reset or an **Apply** operation; merely changing tabs must preserve pending user edits. The action is advertised by `ui_get` and accepts `true`, not a toggle.

### Atomic requests and rendered state

`ui_set` validates the whole configuration before changing presentation state. Unknown keys, unknown entity IDs, unsupported tabs, non-finite numbers, or invalid bounds reject the request without partially applying the UI update. This atomicity applies to `ui_set`, not every legacy raw command.

Tabs, layouts and scrolls are consumed by the real ImGui rendering path. A successful response does not mean the next screenshot already shows the result. Inspect:

- `revision`: latest accepted UI configuration revision;
- `rendered_revision`: revision settled by the renderer;
- `pending`: layout/tab/scroll work still waiting for frames;
- `frame` and `bounds`: rendered frame and visible panel geometry, in main-viewport logical pixels.

Use `stewart_wait_frames(frames=3)` and then read `stewart_ui_get()` again. Before a capture, require `pending: false` and the intended rendered revision. A fixed delay alone is not proof that a hidden panel or sub-tab consumed its request. If pending state persists, inspect the visible panel/tab combination instead of capturing an unrelated screen.

`stewart_window_set` changes the client window size within the advertised bounds. `stewart_window_get` reports logical window dimensions; the PNG dimensions returned by `stewart_screenshot` are the actual framebuffer size and can differ on scaled displays. Screenshot paths must be absolute `.png` filenames in an existing directory.

## Raw command workflow

`stewart_raw(cmd, args)` exposes commands reported by `stewart_capabilities`; it does not bypass documentation-mode guards. Discover current capabilities before using raw commands.

For example, the MCP calls `stewart_raw("ping")` and `stewart_raw("ui_get")` correspond to these newline-delimited requests on the selected loopback TCP port:

```json
{"id":1,"cmd":"ping","args":{}}
{"id":2,"cmd":"ui_get","args":{}}
```

Use one request/response transaction per connection as the bridge does. Replies contain matching `id`, Boolean `ok`, and either an object `result` or an `error`. Read through the newline; a TCP read can contain only part of a response. The bridge validates framing and rejects truncated/invalid responses rather than treating them as successful commands.

Do not run hardware commands in normal mode as part of documentation capture. Reading capabilities is not authorization to connect, stream, push settings, or move a platform.

## Batch screenshot helper

[`app/tools/capture_docs.py`](../app/tools/capture_docs.py) captures the actual application over an MCP stdio client session. It uses synthetic SIL data and disconnected HIL views; no hardware is required.

```powershell
app/tools/.venv/Scripts/python.exe app/tools/capture_docs.py --exe app/build-docs/Release/stewart-platform.exe --port 8772 --output-dir docs/images/app --width 1920 --height 1080 --hold-open
```

The interface requires `--exe`. Optional settings include `--port` (default `8772`), `--output-dir` (default repository `docs/images/app`), `--report-dir` (default `app/build-docs/capture-reports`), `--width` and `--height` (default `1920` × `1080`). It launches/attaches only in documentation mode. The 11 views cover the overview, entity, dynamics, geometry, home-height measurement, recording, time-series, spectrogram, offline HIL settings, console and multiple entities. Geometry, Measure and offline HIL settings use focused `1440 × 900` captures.

Only the PNGs belong in the published documentation. The capture manifest and live-validation JSON are local run reports written to the ignored report directory, separate from the images. They record source/executable hashes, UI state, image dimensions/hashes, safety checks and protected-user-file hashes. Keep them locally when investigating a capture, not as user guides.

`--hold-open` keeps the MCP client alive for inspection until the app is closed. Without it, some execution hosts terminate child applications when the capture process ends; do not assume the app will remain open. The helper intentionally marks captures `CAPTURED_PENDING_VISUAL_QA`: files existing and API checks passing do not replace inspecting the final images.

Before publishing, inspect every PNG for readable labels, clipping, useful framing and clear synthetic/offline state. Confirm local image links resolve and that personal settings and recordings were preserved. A completed capture does not establish board operation.

## Coverage and limitations

The automation layer covers named layouts, panel visibility, entity/platform/data tabs, the seven Measure steps, camera framing, input strip height, selected scroll positions, window size, actual framebuffer capture, and selected existing app commands. It is **not** an implementation of every mouse/keyboard interaction. Do not assume every collapse section, modal, other wizard control, plugin-specific widget or Dynamics staging control has a callable automation equivalent; inspect capabilities and validate the visible result.

The Recording tab currently shows capture metadata and management controls, not a saved-waveform preview. Use the real **Time-Series** view while replaying the synthetic capture to show waveforms; do not fabricate a preview panel for documentation.

Long Dynamics and Data Streams panels are scrollable by design. The screenshots show representative real states, not every interactive control at once.

Pipeline setters such as `stewart_mca_set` change API-controlled configuration directly. They are not evidence that a user pressed the Dynamics **Apply** button or that the staged editor was exercised. The playback API's accepted speed range also differs from the UI slider: the raw API clamps to `0.05..5`, while the documented UI is `10–200%`.

Synthetic fixtures produce genuine runtime plots and IK output but do not prove timing accuracy, mechanical limits, actuator calibration, transport correctness, or board operation. Label captures accordingly. HIL screenshots must visibly remain offline; do not fabricate telemetry.

## Regression checks

With the MCP virtual environment already installed:

```powershell
app/tools/.venv/Scripts/python.exe -m unittest discover -s app/tools
ctest --test-dir app/build-docs -C Release --output-on-failure
```

The Python bridge tests use mocks and do not launch the app or access hardware. On Windows the CMake test targets include documentation transport guards and the existing recording-precision tests. These checks complement, but do not replace, real MCP/schema testing, visual QA of every capture, and separate hardware commissioning.
