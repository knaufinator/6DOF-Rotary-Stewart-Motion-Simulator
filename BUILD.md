# Build Guide

Complete build instructions for both the **ESP32 firmware** and the **desktop application**.

---

## Table of Contents

- [Prerequisites](#prerequisites)
- [Repository Layout](#repository-layout)
- [Part 1: Desktop Application (app/)](#part-1-desktop-application)
- [Part 2: ESP32 Firmware (Controller/)](#part-2-esp32-firmware)
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
| **ESP-IDF** | v5.2.0 | Exact version — not v5.1, not v5.3 |
| **Python** | 3.8+ | Used by ESP-IDF build system internally |
| **USB driver** | — | ESP32-S3 uses native USB CDC (no FTDI/CP210x needed) |

Install ESP-IDF v5.2.0 following the [official guide](https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32s3/get-started/index.html).

**Windows typical install path:** `C:\Users\<you>\esp\v5.2\esp-idf`

---

## Repository Layout

```
6DOF-Rotary-Stewart-Motion-Simulator/
├── app/                          # Desktop application (C++/OpenGL/ImGui)
│   ├── CMakeLists.txt            # CMake build — fetches all dependencies automatically
│   ├── src/
│   │   ├── main.cpp              # Entry point, GLFW window, render loop
│   │   ├── app.cpp               # Application state, update loop, I/O threads
│   │   ├── app.h                 # App class, Entity, PipelineConfig structs
│   │   ├── ui_panels.cpp         # All ImGui UI panels
│   │   ├── platform_viz.cpp      # 3D Stewart platform OpenGL renderer
│   │   ├── serial_port.cpp       # Win32 serial port (COM) abstraction
│   │   └── ...
│   ├── recordings/               # Saved motion capture files (.stwr)
│   ├── layouts/                  # ImGui window layout presets
│   ├── stewart_settings.json     # Auto-saved application settings
│   └── stewart_imgui.ini         # Auto-saved ImGui window positions
│
├── Controller/                   # ESP32-S3 firmware (ESP-IDF v5.2)
│   ├── CMakeLists.txt            # ESP-IDF project root
│   ├── sdkconfig.defaults        # Default Kconfig values
│   ├── build-esp-idf.ps1         # One-click Windows build script
│   ├── main/
│   │   ├── main.cpp              # app_main, FreeRTOS tasks, command API
│   │   ├── InverseKinematics.cpp # Stewart platform IK solver (shared with app)
│   │   ├── AxisScaling.cpp       # Per-axis input scaling (shared with app)
│   │   ├── MotionCueing.cpp      # MCA washout filters (shared with app)
│   │   ├── EthernetTransport.cpp # W5500 SPI Ethernet + UDP
│   │   ├── WifiTransport.cpp     # WiFi STA + UDP
│   │   ├── BleTransport.cpp      # BLE GATT transport
│   │   ├── helpers.cpp           # Utility functions
│   │   └── CMakeLists.txt        # Component registration + compiler flags
│   └── include/                  # Shared headers
│       ├── InverseKinematics.h   # StewartConfig, PlatformDef, IK API
│       ├── AxisScaling.h         # AxisScaleConfig, mapRawToPosition()
│       ├── MotionCueing.h        # MCA filters, presets
│       ├── MCPWMMotorControl.h   # MCPWM hardware motor control
│       ├── GPTimerScheduler.h    # 100µs deterministic timer
│       ├── helpers.h             # Pin definitions, timing constants
│       ├── version.h             # Firmware version + platform ID
│       └── ...
│
├── docs/                         # Architecture docs, research, hardware plans
├── tools/                        # Utility scripts (sine_heave.py, etc.)
└── test_harness/                 # Step/dir signal analyzer firmware
```

---

## Part 1: Desktop Application

The desktop app is a **single native C++ executable** — no Python, no server, no browser. CMake's `FetchContent` automatically downloads all dependencies (GLFW, Dear ImGui, ImPlot, cJSON) on first configure. The app compiles the same C firmware modules (IK, axis scaling, motion cueing) directly from `Controller/` source files.

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
- Links shared C modules from `../Controller/include/` and `../Controller/main/`
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

### Full Clean Rebuild

```powershell
# Delete the entire build directory
Remove-Item -Recurse -Force app\build

# Reconfigure + build from scratch
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

> **Tip:** If `Remove-Item` fails with "file in use" errors, close any running instances of the app, kill stale `MSBuild.exe` or `cmake.exe` processes, then retry.

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
| InverseKinematics | `Controller/main/InverseKinematics.cpp` | Stewart platform IK solver |
| AxisScaling | `Controller/main/AxisScaling.cpp` | Per-axis input scaling |
| MotionCueing | `Controller/main/MotionCueing.cpp` | MCA washout filters |

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
cd Controller

# Option A: Use the build script (auto-finds ESP-IDF)
.\build-esp-idf.ps1

# Option B: Manual (if ESP-IDF is already on PATH)
idf.py set-target esp32s3    # first time only
idf.py build
```

### Step-by-Step Walkthrough (Windows)

#### 1. Install ESP-IDF v5.2.0

Follow the [official installer](https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32s3/get-started/windows-setup.html) or use the manual method:

```powershell
mkdir C:\Users\$env:USERNAME\esp\v5.2
cd C:\Users\$env:USERNAME\esp\v5.2
git clone -b v5.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
.\install.ps1 esp32s3
```

#### 2. Load the ESP-IDF Environment

```powershell
# Source the ESP-IDF export script (sets PATH, IDF_PATH, etc.)
. C:\Users\Chris\esp\v5.2\esp-idf\export.ps1
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
cd Controller
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

Edit `Controller/main/CMakeLists.txt` line 44:

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
cd Controller
idf.py fullclean
idf.py set-target esp32s3
idf.py build
```

---

## Shared C Modules

Three C source files are **shared between the ESP32 firmware and the desktop app** — single source of truth for all math:

| File | Description |
|------|-------------|
| `Controller/main/InverseKinematics.cpp` | Closed-form rotary actuator IK solver (Eisele formulation) |
| `Controller/main/AxisScaling.cpp` | Per-axis input scaling with degree-to-radian conversion |
| `Controller/main/MotionCueing.cpp` | Biquad washout filters, tilt coordination, MCA presets |

Headers are in `Controller/include/`.

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
| `SIL_BUILD` / `_USE_MATH_DEFINES` errors | These are set automatically by CMake. Don't compile `Controller/` sources manually. |

### ESP32 Firmware

| Problem | Solution |
|---------|----------|
| `idf.py: command not found` | Source the environment: `. C:\Users\Chris\esp\v5.2\esp-idf\export.ps1` |
| Wrong IDF version | `idf.py --version` — must be 5.2.x. Different versions may have incompatible APIs. |
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
