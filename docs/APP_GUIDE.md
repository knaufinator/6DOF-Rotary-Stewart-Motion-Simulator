# Stewart Platform — Desktop App Guide

> A walkthrough for the native C++/OpenGL/ImGui desktop application that controls, tunes, and visualises your 6-DOF Stewart motion simulator.

**See also:** [BUILD.md](../BUILD.md) for build instructions · [ARCHITECTURE_ROADMAP.md](ARCHITECTURE_ROADMAP.md) for internal design · [README.md](../README.md) for project overview

---

## Table of Contents

- [First Launch](#first-launch)
- [UI Overview](#ui-overview)
- [Toolbar](#toolbar)
- [Entities (SIL & HIL)](#entities-sil--hil)
- [Input Sources](#input-sources)
- [Dynamics (Motion Cueing)](#dynamics-motion-cueing)
- [Platform Setup (Geometry)](#platform-setup-geometry)
- [HIL Mode — Connecting an ESP32](#hil-mode--connecting-an-esp32)
- [Recording & Playback](#recording--playback)
- [Data Streams & Spectrogram](#data-streams--spectrogram)
- [Console](#console)
- [Plugins](#plugins)
- [Settings & Persistence](#settings--persistence)
- [Keyboard Shortcuts](#keyboard-shortcuts)
- [Tips & Troubleshooting](#tips--troubleshooting)

---

## First Launch

```
cd app
.\build\Release\stewart-platform.exe
```

On first launch the app creates two files in the working directory:

| File | Purpose |
|------|---------|
| `stewart_settings.json` | All entity configs, input state, MCA presets, plugin parameters |
| `stewart_imgui.ini` | Window positions, sizes, docking layout |

A default **SIL** (Software-in-the-Loop) entity is created automatically so you can start experimenting immediately — no hardware needed.

---

## UI Overview

The interface is built with **Dear ImGui** and supports full **docking** — every panel can be dragged, tabbed, floated, or hidden. The layout auto-saves between sessions.

```
┌──────────────────────────────────────────────────────────────────────┐
│  Menu Bar:  File │ View │ Help                                       │
├──────────────────────────────────────────────────────────────────────┤
│  Toolbar:  [Source ▾] LIVE │ START/STOP │ E-STOP │ 75% │ REC PLAY  │
├──────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌─ Entity Card ─────────┐  ┌─ Input Panel ──────────────────────┐  │
│  │  [3D Platform Viz]    │  │  Source selector (Manual/Plugin/   │  │
│  │   orbit camera        │  │  Capture) + per-axis sliders or    │  │
│  │   status overlay      │  │  plugin parameters                 │  │
│  │                        │  └────────────────────────────────────┘  │
│  │  ┌ Servo Readout ───┐ │                                          │
│  │  │ S0–S5 angles,    │ │  ┌─ Dynamics Panel ───────────────────┐  │
│  │  │ util bars        │ │  │  Device selector, profile banner   │  │
│  │  └──────────────────┘ │  │  MCA filters, axis gains, occupant │  │
│  └────────────────────────┘  └────────────────────────────────────┘  │
│                                                                      │
│  ┌─ Console ─────────────────────────────────────────────────────┐  │
│  │  Per-entity filtered log with timestamps                       │  │
│  └────────────────────────────────────────────────────────────────┘  │
│                                                                      │
│  ┌─ Data Streams ────────────────────────────────────────────────┐  │
│  │  Recording │ Frequency │ Spectrogram │ Time History            │  │
│  └────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────┘
```

### Resetting the Layout

**View → Reset Layout** restores the default panel arrangement.

---

## Toolbar

The toolbar is pinned across the top of the window. It provides quick access to the most common actions without opening any panel.

| Section | Controls | What it does |
|---------|----------|--------------|
| **Source** | Dropdown + status dot | Switch between Manual, Capture, or Plugin input. Green "LIVE" dot when active. |
| **Motion** | START / STOP, E-STOP, intensity slider | Start/stop the motion pipeline. E-STOP immediately zeros all outputs. Intensity scales overall motion 0–100%. |
| **Recording** | REC, PLAY, STOP | Quick-access recording controls (mirrors the Data Streams panel). |
| **Platform** | SIL:N HIL:N, +SIL, +HIL | Shows entity counts. Add new SIL or HIL entities. |
| **Status** | fps, MCA Hz, entity count | Read-only status bar (right-aligned). |

---

## Entities (SIL & HIL)

An **entity** is an independent Stewart platform instance with its own geometry, dynamics config, 3D visualizer, and optional ESP32 connection.

### SIL (Software-in-the-Loop)

- Runs the full pipeline locally: axis scaling → motion cueing → inverse kinematics
- No hardware required — ideal for tuning, testing, and visualization
- Click **+SIL** in the toolbar to add one

### HIL (Hardware-in-the-Loop)

- Connects to a physical ESP32-controlled Stewart platform via USB serial
- Sends motion data to the ESP32; receives telemetry back
- **When offline (ESP32 not connected):** the entity is idle — no IK computation, no animation. The 3D viewport shows the platform at its home position with an "OFFLINE" label.
- **When connected with telemetry:** the 3D viewport shows the real platform pose driven by ESP32 telemetry angles.
- Click **+HIL** in the toolbar to add one

### Entity Card

Each entity gets its own floating window containing:

1. **3D Platform Viewport** — interactive orbit camera (click-drag to rotate). Shows the Stewart platform rendered in real-time with color-coded servo arms, connecting rods, and joint dots.
2. **Status Overlay** — top-right corner shows connection state:
   - **ESP32 TELEMETRY** (green) — live data from hardware
   - **CONNECTED (awaiting telemetry)** (amber) — serial open, waiting for data flow
   - **OFFLINE** (grey) — no serial connection
3. **Servo Readout** — below the 3D viewport, shows per-servo angles, step counts, and utilization bars
4. **Context Menu** — right-click the entity card header for:
   - **Settings** — name, color, connection config
   - **Platform Setup** — geometry, motor, and axis configuration
   - **I/O Monitor** — per-entity serial/debug log
   - **Remove** — delete the entity

### Draggable Splitter

The vertical divider between the 3D viewport and the servo readout bars is draggable — pull it up or down to allocate more space to either section.

---

## Input Sources

The **Input** panel controls what data feeds the motion pipeline. All input sources — plugins and capture playback — appear in a single flat dropdown at the top.

### Plugins

Plugins are external DLLs that generate motion data. The app scans `app/build/Release/plugins/` on startup and lists all discovered plugins in the dropdown.

Built-in plugins include:

| Plugin | Description |
|--------|-------------|
| **Manual Sliders** | Direct per-axis control via six sliders (±100%) for basic testing and checkout |
| **Sine Wave Demo** | Simple per-axis sine wave for testing |
| **Test Signal** | Advanced waveform generator with per-axis frequency, amplitude, phase, DC offset, and waveform type (sine/square/triangle/sawtooth) |
| **SimTools UDP** | Receives motion data from SimTools over UDP |
| **Assetto Corsa** | Reads Assetto Corsa shared memory telemetry |

Select a plugin from the dropdown, then configure its parameters in the auto-layout panel below. Parameters are intelligently grouped:
- **Per-axis parameters** (detected by naming convention like `freq_surge`, `amp_sway`) render as compact tables
- **Global parameters** render in a responsive two-column grid

### Capture Playback

Plays back a previously recorded motion capture. Select a recording from the library in the Data Streams panel, then switch the source to Capture Playback. Controls include play, pause, stop, loop, and speed adjustment.

---

## Dynamics (Motion Cueing)

The **Dynamics** panel configures the full motion cueing pipeline for a selected entity. Use the **Device** dropdown at the top to choose which entity to edit.

### Profile System

- **Profile banner** shows the currently loaded preset name
- **Copy** — serializes all dynamics settings to clipboard as JSON
- **Paste** — loads dynamics settings from clipboard JSON
- Built-in and user presets available via the preset dropdown

### Sections

| Section | What it controls |
|---------|-----------------|
| **Intensity** | Global motion scale (0–100%) |
| **Axis Gains** | Per-axis multipliers (Surge through Yaw) with invert toggles |
| **Motion Cueing (MCA)** | Washout filter configuration — high-pass and low-pass cutoff frequencies, tilt coordination gains. Presets: Off, Gentle, Moderate, Aggressive, Race. |
| **Occupant Position** | X/Y/Z offset of the rider's head from platform center. Compensates for parasitic translations caused by off-center seating. |

### Staging Buffer

Dynamics edits use a **staging buffer** — changes are previewed in real-time but only committed to the live config when you click **Apply**. This prevents accidental parameter changes during active sessions.

---

## Platform Setup (Geometry)

Open from the entity card context menu → **Platform Setup**. Contains tabbed configuration:

### Geometry Tab

| Parameter | Description |
|-----------|-------------|
| **RD** | Base plate radius (center to servo shaft), mm |
| **PD** | Platform plate radius (center to ball joint), mm |
| **L1** | Servo arm length (shaft to rod end), mm |
| **L2** | Connecting rod length (rod end to rod end), mm |
| **Home Height** | Vertical distance base-to-platform when servos at 0° |
| **Theta R** | Base joint pair angle spread |
| **Theta P** | Platform joint pair angle spread |

> **Auto button:** The orange **Auto** button next to Home Height computes the mathematically correct value from your geometry. If it's orange, your current value is off by >0.5 mm. Always click Auto after changing any geometry parameter. See the [README geometry section](../README.md#geometry-calibration--home-height) for details.

### Motor Tab

Configure servo range, steps per degree, and bit depth.

### Axis Scales Tab

Per-axis scaling that maps raw input values to physical units (mm for translations, degrees for rotations).

---

## HIL Mode — Connecting an ESP32

### Automatic Connection

1. Click **+HIL** to create a HIL entity
2. Open **Settings** (right-click header)
3. Under **ESP32 Connection**:
   - Select the COM port from the dropdown (or leave blank for auto-detect)
   - Enable **Auto Connect**
4. The app will automatically:
   - Open the serial port
   - Perform a handshake (fingerprint → config exchange → telemetry setup)
   - Start streaming motion data and receiving telemetry

### Connection States

| State | What's happening |
|-------|-----------------|
| **Offline** | No serial connection. Entity is idle — no IK, no animation. |
| **Handshaking** | Serial open, exchanging fingerprint and configuration with ESP32. Progress shown in overlay. |
| **Connected** | Handshake complete. Motion packets streaming to ESP32. Awaiting telemetry flow. |
| **Telemetry Active** | Full bidirectional: motion TX → ESP32, telemetry RX ← ESP32. 3D viz driven by real hardware angles. |

### Auto-Reconnect

If the USB cable is unplugged or the ESP32 resets, the app detects the lost connection and automatically retries every 3 seconds (when Auto Connect is enabled). Manual disconnect via the Settings panel disables auto-reconnect.

### TX Rate & Protocol

Configurable in Settings (when disconnected):

- **TX Rate**: 10–1000 Hz (how fast motion packets are sent to ESP32)
- **Bit Depth**: 8, 10, 12, 14, or 16 bits per axis
- **Protocol**: Binary (default, efficient) or CSV (legacy, for debugging)

---

## Recording & Playback

### Recording

1. Switch to the **Data Streams** panel → **Recording** tab
2. Select a **Sample Rate** (50–1000 Hz)
3. Click **Record** — captures the current input source data
4. Click **Stop Recording** when done
5. Enter a name and click **Save to Library**

Recordings are saved as `.stwr` files in `app/recordings/` with a `manifest.json` index.

### Playback

1. Switch the input source to **Capture** (toolbar or Input panel)
2. In the Data Streams → Recording tab, select a saved capture from the library
3. Use **Play**, **Pause**, **Stop**, and **Loop** controls
4. Adjust **Speed** (0.1×–10×) for slow-motion analysis or fast-forward

### What Gets Recorded

The recording captures the **6-axis input percentages** at the configured sample rate, regardless of which input source is active. This means you can record from a plugin (e.g., Assetto Corsa), save it, and replay it later without the game running.

---

## Data Streams & Spectrogram

The **Data Streams** panel contains multiple tabs for signal analysis:

### Recording Tab

Capture library management (see [Recording & Playback](#recording--playback) above).

### Frequency Tab

Real-time FFT frequency charts for each axis. Useful for identifying resonance, vibration modes, or signal quality issues.

### Spectrogram Tab

Rolling waterfall heatmap showing frequency content over time. Supports multi-entity and multi-axis lane selection. Color intensity maps to signal power — bright spots indicate dominant frequencies.

### Time History Tab

Scrolling time-series plots of input/output signals. Compare multiple entities or axes side-by-side.

---

## Console

The **Console** panel shows timestamped log messages from all subsystems:

- **Entity filter buttons** — click to show/hide messages from specific entities
- **Pause** — freeze the log (messages still queue, displayed on resume)
- **Copy** — copy visible log to clipboard
- **Clear** — wipe the log
- **Rate limiter** — dropdown to throttle log output (e.g., 10 Hz) to prevent flooding

Log categories include: `hil`, `sil`, `serial`, `platform`, `record`, `plugin`, etc.

### Per-Entity I/O Monitor

Each entity also has its own dedicated console (right-click entity header → **I/O Monitor**). This shows only messages related to that specific entity — useful when debugging a single ESP32 connection.

---

## Plugins

Plugins are shared libraries (`.dll` on Windows) that implement the Stewart Plugin API. They run in-process and generate 6-axis motion data at the app's frame rate.

### Plugin Directory

```
app/build/Release/plugins/
├── plugin_sine_demo.dll
├── plugin_test_signal.dll
├── plugin_simtools_udp.dll
└── plugin_assetto_corsa.dll
```

The app scans this directory on startup. Plugins are listed in the Input panel when **Plugin** source is selected.

### Using a Plugin

1. Set the input source to **Plugin** (toolbar dropdown or Input panel)
2. Select a plugin from the plugin dropdown
3. Configure parameters using the auto-generated UI
4. The plugin immediately starts producing motion data

### Plugin Parameters

Parameters are auto-detected from the plugin's `stewart_plugin_info()` export. The UI engine automatically:

- Groups per-axis parameters (e.g., `freq_surge`, `freq_sway`, ..., `freq_yaw`) into compact tables
- Renders global parameters in a responsive grid
- Supports float, int, bool, and enum parameter types
- Shows live output bars when the plugin is active

### Building a Plugin

See `app/plugins/` for source examples. Each plugin is a single C file compiled to a DLL:

```bat
cd app\plugins
build_plugin.bat plugin_my_source.c
```

The plugin API requires three exports:
- `stewart_plugin_info()` — returns plugin metadata and parameter definitions
- `stewart_plugin_init()` — called when the plugin is activated
- `stewart_plugin_process()` — called every frame, produces 6-axis output
- `stewart_plugin_shutdown()` — called when the plugin is deactivated

---

## Settings & Persistence

### Auto-Save

The app automatically saves all settings to `stewart_settings.json` on exit and periodically during operation. This includes:

- All entity configurations (geometry, dynamics, axis scales, motor settings)
- Input source selection and plugin parameters
- HIL connection settings (COM port, TX rate, auto-connect)
- MCA presets (built-in + user-created)
- Recording library manifest
- Window layout (separate file: `stewart_imgui.ini`)

### Settings File Location

Settings are saved relative to the working directory where the app was launched. Typically:

```
app/stewart_settings.json
app/stewart_imgui.ini
app/recordings/manifest.json
app/recordings/000.stwr, 001.stwr, ...
```

### Resetting Settings

Delete `stewart_settings.json` to reset all entity configs and preferences to defaults. Delete `stewart_imgui.ini` to reset the window layout.

---

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| **Ctrl+Q** | Quit |

> More shortcuts to be added as the app evolves. Most interactions are mouse-driven via ImGui widgets.

---

## Tips & Troubleshooting

### General

- **Panels disappeared?** Use **View → Reset Layout** to restore the default arrangement.
- **App won't start?** Ensure your GPU supports OpenGL 3.3+. Update drivers if needed.
- **Settings corrupted?** Delete `stewart_settings.json` and restart.

### HIL / ESP32

- **Can't find COM port?** The ESP32-S3 uses native USB CDC — no FTDI driver needed. Ensure you're using the USB port (not UART) on the DevKitC. Try unplugging and re-plugging.
- **Handshake stuck?** The app sends a `FINGERPRINT?` command and waits for a response. If the ESP32 firmware is outdated or unresponsive, the handshake will time out. Reflash the firmware.
- **Telemetry not flowing?** After successful handshake, the app requests telemetry via `TELRATE:N`. If the ESP32 doesn't respond, check firmware version compatibility.
- **HIL entity shows "OFFLINE"?** This is normal when the ESP32 is not connected. The entity consumes zero CPU — it simply waits for a connection.

### Plugins

- **Plugin not appearing?** Ensure the DLL is in `app/build/Release/plugins/`. The app only scans on startup — restart after adding new plugins.
- **Plugin crashes?** Check the Console for error messages. Plugins run in-process, so a crash will take down the app. Debug with the per-entity I/O monitor.

### Performance

- **Low frame rate?** The app targets vsync (~60 fps). If it's lower, check if too many entities are active or if the spectrogram is consuming excessive resources.
- **High CPU usage?** Normal during active motion — the pipeline runs every frame. When idle (no input source active, HIL offline), CPU usage should be minimal.

---

## Glossary

| Term | Definition |
|------|-----------|
| **SIL** | Software-in-the-Loop — local simulation, no hardware |
| **HIL** | Hardware-in-the-Loop — connected to a physical ESP32 + servos |
| **IK** | Inverse Kinematics — computes servo angles from desired platform pose |
| **FK** | Forward Kinematics — computes platform pose from servo angles |
| **MCA** | Motion Cueing Algorithm — washout filters that map game telemetry to safe platform motion |
| **Telemetry** | Data streamed back from the ESP32 (servo angles, positions) |
| **Entity** | An independent platform instance with its own config and pipeline |
| **Plugin** | External DLL that generates motion data (e.g., game telemetry reader) |
| **Home Position** | Platform at rest — all servos at 0°, platform level and parallel to base |
| **Utilization** | How close a servo is to its angular limits (0–100%) |

---

*This guide is a living document. Update it as features are added or workflows change.*
