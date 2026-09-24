# Build Guide

Paths and commands in this guide start at the repository root unless noted otherwise.

After moving an existing checkout, use a fresh build directory: generated CMake
caches retain absolute source paths. Preserve old build outputs; do not reuse an
old cache without reconfiguring. Run the desktop app with `app/` as its
working directory to keep using the existing settings and recordings.

Complete build instructions for both the **ESP32 firmware** and the **desktop application**.

The current dependency-lock baseline is ESP-IDF 5.5.2. This guide describes builds;
no build result clears the [r13 firmware and commissioning gates](../hardware/pcb/6DOF2_FIRMWARE_TODO.md).
For a fresh clone, initialize the `controller/stewart-core` submodule from the repository
root with `git submodule update --init --recursive` before building. If relocating an
existing checkout, preserve local submodule edits and verify its path in `.gitmodules`
and the Git index before running submodule updates; do not reinitialize it to fix a
path mismatch. On case-insensitive filesystems, verify that committed directory names
match the lowercase paths below so the checkout also works on Linux.

---

## Table of Contents

- [Prerequisites](#prerequisites)
- [Repository Layout](#repository-layout)
- [Part 1: Desktop Application (app/)](#part-1-desktop-application)
- [Part 2: ESP32 Firmware (controller/)](#part-2-esp32-firmware)
- [Shared C Modules](#shared-c-modules)
- [Troubleshooting](#troubleshooting)

---

## Prerequisites

### Desktop Application

| Requirement | Minimum Version | Notes |
|-------------|-----------------|-------|
| **CMake** | 3.20+ | Included with Visual Studio Build Tools |
| **C++17 compiler** | MSVC 19.x / GCC 10+ / Clang 12+ | Visual Studio 2022 Build Tools recommended on Windows |
| **OpenGL** | 3.3+ | Any modern GPU driver |
| **Git** | 2.x | Required for CMake `FetchContent` dependency downloads |

**Windows (recommended):** Install [Visual Studio 2022 Build Tools](https://visualstudio.microsoft.com/downloads/#build-tools-for-visual-studio-2022) with the "Desktop development with C++" workload. This provides `cl.exe`, `cmake.exe`, and `MSBuild`.

**Linux:** `sudo apt install build-essential cmake git libgl-dev libx11-dev libxrandr-dev libxi-dev libxcursor-dev libxinerama-dev`

**macOS:** `xcode-select --install && brew install cmake`

### ESP32 Firmware

| Requirement | Version | Notes |
|-------------|---------|-------|
| **ESP-IDF** | v5.5.2 | Version recorded by the current firmware dependency locks |
| **Python** | 3.8+ | Used by ESP-IDF build system internally |
| **USB driver** | — | ESP32-S3 uses native USB CDC (no FTDI/CP210x needed) |

Install ESP-IDF v5.5.2 following the [official guide](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32s3/get-started/index.html).

**Windows typical install path:** `C:\Users\<you>\esp\v5.5.2\esp-idf`

---

## Repository Layout

```text
6DOF-Rotary-Stewart-Motion-Simulator/
├── app/                     Desktop C++/OpenGL/ImGui application
│   ├── CMakeLists.txt       FetchContent dependencies and shared-math target
│   ├── src/                Application state, UI, rendering and transports
│   ├── bridge/             Optional Linux serial/WebSocket HIL service
│   ├── recordings/         Saved motion captures
│   └── layouts/            UI layout presets
├── controller/              Full-size ESP-IDF firmware project
│   ├── CMakeLists.txt      Full-size project configuration
│   ├── main/               Application, FreeRTOS tasks and transports
│   ├── include/            I/O definitions, pulse engines and protocol headers
│   ├── mini/               Separate hobby-servo ESP32 firmware project
│   ├── test_harness/       Separate step/dir analyzer ESP32-S3 project
│   ├── stewart-core/       Shared Git submodule, used by firmware and app
│   ├── tests/              Host regression tests
│   └── tools/              Diagnostic and validation scripts
├── hardware/                PCB release packages and mechanical models
└── docs/                    Build/usage guides, developer references and screenshots
```

The desktop app's settings files remain under `app/`; use that directory as
its working directory. Build each ESP-IDF project from its own directory.

Generated build output belongs in the software project's ignored `build/` or
`build-*/` directories. Keep local session reports and working notes out of `docs/`;
that directory is for maintained project documentation. Hardware source and release
artifacts must remain eligible for tracking. Check the scoped ignore rules with:

```powershell
python -B controller/tools/check_gitignore.py
```

---

## Part 1: Desktop Application

The desktop app is a **single native C++ executable** — no Python, no server, no browser. CMake's `FetchContent` automatically downloads all dependencies (GLFW, Dear ImGui, ImPlot, cJSON) on first configure. The app compiles the same C firmware modules (IK, axis scaling, motion cueing) from the shared `controller/stewart-core/` submodule.

### Quick Build (Windows — PowerShell)

```powershell
cd app

# Configure (downloads dependencies on first run — needs internet, takes ~30s)
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build --config Release

# Run
.\build\Release\stewart-platform.exe
```

### Quick Build (Linux / macOS)

```bash
cd app

# Configure
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build --config Release -j$(nproc)

# Run
./build/Release/stewart-platform
```

### Step-by-Step Walkthrough (Windows)

#### 1. Open a Developer PowerShell

If you installed Visual Studio Build Tools, open **"Developer PowerShell for VS 2022"** from the Start Menu. This ensures `cl.exe` and `cmake` are on PATH.

Alternatively, locate CMake manually:
```powershell
# Visual Studio Build Tools CMake location:
& "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --version
```

#### 2. Configure the Build

```powershell
cd C:\Users\Chris\Documents\GitHub\6DOF-Rotary-Stewart-Motion-Simulator\app

cmake -B build -DCMAKE_BUILD_TYPE=Release
```

**What happens during configure:**
- CMake downloads GLFW 3.4, Dear ImGui v1.91.8-docking, ImPlot v0.16, and cJSON v1.7.18 via Git
- Generates Visual Studio project files in `build/`
- Builds shared math from `../controller/stewart-core/include/` and `../controller/stewart-core/src/`; uses `../controller/include/` for transport headers
- First run requires internet and takes ~30 seconds

**Expected output (last few lines):**
```
-- Configuring done (29.9s)
-- Generating done (0.2s)
-- Build files have been written to: .../app/build
```

> **Note:** The `CMAKE_BUILD_TYPE` warning on Windows is harmless — MSVC uses `--config Release` at build time instead.

#### 3. Build

```powershell
cmake --build build --config Release
```

**Expected output:**
```
...
stewart-platform.vcxproj -> ...\app\build\Release\stewart-platform.exe
```

Build takes ~20-40 seconds on first compile. Subsequent rebuilds only recompile changed files (~3-5 seconds).

#### 4. Run

```powershell
.\build\Release\stewart-platform.exe
```

The app creates `stewart_settings.json` and `stewart_imgui.ini` in the working directory for persistent settings.

### Rebuilding After Code Changes

```powershell
# Just rebuild — no need to reconfigure unless CMakeLists.txt changed
cmake --build build --config Release
```

### Fresh Build After Moving the Checkout

From `app/`, configure a new directory so earlier caches and outputs stay available:

```powershell
cmake -S . -B build-fresh -DCMAKE_BUILD_TYPE=Release
cmake --build build-fresh --config Release
.\build-fresh\Release\stewart-platform.exe
```

Keep `app/` as the launch working directory so existing settings and
recordings remain in use. For firmware, select a fresh directory with
`idf.py -B build-fresh build` from the appropriate firmware project.

### Dependencies (auto-fetched by CMake)

| Library | Version | Purpose |
|---------|---------|---------|
| [GLFW](https://www.glfw.org/) | 3.4 | Window creation, OpenGL context, input |
| [Dear ImGui](https://github.com/ocornut/imgui) | v1.91.8-docking | Immediate-mode GUI framework |
| [ImPlot](https://github.com/epezent/implot) | v0.16 | Plotting widgets for ImGui |
| [cJSON](https://github.com/DaveGamble/cJSON) | v1.7.18 | JSON settings serialization |

**Shared from firmware (compiled directly):**

| Module | Source | Purpose |
|--------|--------|---------|
| InverseKinematics | `controller/stewart-core/src/InverseKinematics.cpp` | Stewart platform IK solver |
| AxisScaling | `controller/stewart-core/src/AxisScaling.cpp` | Per-axis input scaling |
| MotionCueing | `controller/stewart-core/src/MotionCueing.cpp` | MCA washout filters |

These are compiled with `SIL_BUILD=1` and `_USE_MATH_DEFINES` to stub out ESP-IDF-specific code.

### Windows Linker Dependencies

Automatically linked on Windows:
- `ws2_32` — Winsock2 (UDP sockets for SimTools / Assetto Corsa)
- `setupapi` — Serial port enumeration
- `opengl32` — OpenGL

---

## Part 2: ESP32 Firmware

### Quick Build

```powershell
cd controller

# Option A: Use the build script (auto-finds ESP-IDF)
.\build-esp-idf.ps1

# Option B: Manual (if ESP-IDF is already on PATH)
idf.py set-target esp32s3    # first time only
idf.py build
```

### Step-by-Step Walkthrough (Windows)

#### 1. Install ESP-IDF v5.5.2

Follow the [official installer](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32s3/get-started/windows-setup.html) or use the manual method:

```powershell
mkdir C:\Users\$env:USERNAME\esp\v5.5.2
cd C:\Users\$env:USERNAME\esp\v5.5.2
git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
.\install.ps1 esp32s3
```

#### 2. Load the ESP-IDF Environment

```powershell
# Source the ESP-IDF export script (sets PATH, IDF_PATH, etc.)
. C:\Users\Chris\esp\v5.5.2\esp-idf\export.ps1
```

You should see output like:
```
Done! You can now compile ESP-IDF projects.
Go to the project directory and run:
  idf.py build
```

> **Tip:** The `build-esp-idf.ps1` script does this automatically — it searches common install paths.

#### 3. Set Target (First Time Only)

```powershell
cd controller
idf.py set-target esp32s3
```

This creates the `build/` directory and configures for ESP32-S3. Only needed once or after a `fullclean`.

#### 4. Build

```powershell
idf.py build
```

**Expected output:**
```
...
Project build complete. To flash, run this command:
  ...esptool.py -p (PORT) ... write_flash ...
or run 'idf.py -p (PORT) flash'
```

Build takes ~60-90 seconds on first compile, ~10-20 seconds on incremental rebuilds.

#### 5. Flash to ESP32

Connect the ESP32-S3 DevKitC via USB. The native USB Serial JTAG interface appears as a COM port.

```powershell
# Auto-detect port
idf.py flash

# Or specify port explicitly
idf.py -p COM3 flash

# Flash and open serial monitor
idf.py -p COM3 flash monitor
```

> **Exit the monitor** with `Ctrl+]`

#### 6. Serial Monitor (Without Flashing)

```powershell
idf.py -p COM3 monitor
```

### Build Options

#### Enable/Disable Features

Edit `controller/main/CMakeLists.txt` line 44:

```cmake
# Current defaults:
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    ENABLE_DEBUG_UART=1    # Runtime debug output (DBG:1 / DBG:0 commands)
    ENABLE_WIFI=1          # WiFi STA + UDP transport
    ENABLE_BLE=1           # BLE GATT transport
)
```

Ethernet (W5500) is always compiled but requires hardware. WiFi and BLE can be removed to save flash space if not needed.

#### Compiler Optimization

Already configured in `main/CMakeLists.txt`:
```cmake
-O2           # Optimization level 2
-ffast-math   # Fast floating point
-fno-exceptions
-fno-rtti
```

### Key sdkconfig Settings

| Setting | Value | Purpose |
|---------|-------|---------|
| `CONFIG_IDF_TARGET` | `esp32s3` | Target MCU |
| `CONFIG_FREERTOS_HZ` | `1000` | 1ms tick for motor timing |
| `CONFIG_ESP_CONSOLE_UART_DEFAULT` | `y` | UART primary console |
| `CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG` | `y` | USB CDC secondary (bidirectional) |
| `CONFIG_ESPTOOLPY_FLASHSIZE` | `8MB` | Flash size |
| `CONFIG_COMPILER_OPTIMIZATION_PERF` | `y` | Performance optimization |
| `CONFIG_RMT_ISR_IRAM_SAFE` | `y` | RMT ISR in IRAM for low latency |
| `CONFIG_BT_ENABLED` | `y` | Bluetooth (BLE) support |

### Full Clean Rebuild

```powershell
cd controller
idf.py fullclean
idf.py set-target esp32s3
idf.py build
```

---

## Shared C Modules

Three C source files are **shared between the ESP32 firmware and the desktop app** — single source of truth for all math:

| File | Description |
|------|-------------|
| `controller/stewart-core/src/InverseKinematics.cpp` | Closed-form rotary actuator IK solver (Eisele formulation) |
| `controller/stewart-core/src/AxisScaling.cpp` | Per-axis input scaling with degree-to-radian conversion |
| `controller/stewart-core/src/MotionCueing.cpp` | Biquad washout filters, tilt coordination, MCA presets |

Shared math headers are in `controller/stewart-core/include/`.

**Firmware** compiles these as an ESP-IDF component with `idf_component_register()`.

**Desktop app** compiles these into a static library (`stewart_math`) with `SIL_BUILD=1` defined to stub out ESP-IDF-specific code (e.g., `SPI3_HOST` macro guard in `helpers.h`).

If you modify any shared module, **both** the firmware and app need to be rebuilt.

---

## Troubleshooting

### Desktop App

| Problem | Solution |
|---------|----------|
| `cmake` not found | Use full path: `"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"` or open Developer PowerShell |
| FetchContent download fails | Check internet connection. Delete `build/` and retry. Ensure `git` is on PATH. |
| `Cannot remove build directory` | Kill stale processes: `Stop-Process -Name MSBuild,cmake,stewart-platform -Force -ErrorAction SilentlyContinue` then retry |
| `Cannot find source file: .../imgui.cpp` | FetchContent didn't complete. Delete `build/` and run configure again — second run succeeds after partial download. |
| Link error: `ws2_32` / `setupapi` | Windows SDK not installed. Install "Desktop development with C++" workload in VS Build Tools. |
| OpenGL errors at runtime | Update GPU drivers. Requires OpenGL 3.3+ support. |
| `SIL_BUILD` / `_USE_MATH_DEFINES` errors | These are set automatically by CMake. Build the shared `controller/stewart-core/` sources through the app's CMake target. |

### ESP32 Firmware

| Problem | Solution |
|---------|----------|
| `idf.py: command not found` | Source the environment: `. C:\Users\Chris\esp\v5.5.2\esp-idf\export.ps1` |
| Wrong IDF version | `idf.py --version` — use the dependency-lock baseline, 5.5.2. Different versions may have incompatible APIs. |
| `set-target` fails | Run `idf.py fullclean` first, then `idf.py set-target esp32s3` |
| Flash fails, port busy | Close serial monitors, other COM port tools. Try `idf.py -p COMx flash` with explicit port. |
| No serial output | Check USB cable (must be data cable, not charge-only). Try both USB ports on DevKitC (use the USB port, not UART). |
| `CONFIG_BT_ENABLED` errors | Bluetooth requires ESP32-S3. Verify target is set correctly. |
| Build takes very long | First build is slow (~90s). Incremental builds are fast. Don't `fullclean` unnecessarily. |

### Common

| Problem | Solution |
|---------|----------|
| Modified shared C module, app shows old behavior | Rebuild the desktop app — it compiles firmware sources directly |
| `sdkconfig` conflicts after git pull | `idf.py fullclean && idf.py set-target esp32s3 && idf.py build` |
| Path too long on Windows | Clone repo closer to drive root (e.g., `C:\dev\`) if nested paths cause issues |
