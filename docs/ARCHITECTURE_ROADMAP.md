# Stewart Platform Controller — Architecture Roadmap

> **Decision: Single native C++ executable. No server. No browser. No IPC.**

---

## 1. Background

The project previously used a Python FastAPI backend + browser-based HTML/JS dashboard (`sil_test/`). That architecture was replaced by a single native C++ executable because a server/client model is fundamentally wrong for a real-time control application.

### What exists today

| Directory | Role |
|-----------|------|
| `app/src/` | Native C++/ImGui desktop app: SIL, HIL, test signals, capture, spectrogram |
| `Controller/main/` | ESP32 firmware + shared C modules (IK, AxisScaling, MotionCueing) |
| `test_harness/` | Step/dir signal analyzer firmware for ESP-to-ESP validation |

---

## 2. Target Architecture: Single Native C++ Application

```
┌─────────────────────────────────────────────────────────────────┐
│                   stewart-platform.exe                            │
│                   Single process, single render loop              │
│                                                                   │
│  ┌──────────────────────── Main Loop (vsync, ~60fps) ──────────┐ │
│  │                                                               │ │
│  │  1. INPUT PHASE                                               │ │
│  │     ├── Poll SimTools UDP socket (non-blocking)               │ │
│  │     ├── Read slider values (ImGui state, zero-copy)           │ │
│  │     └── Check serial RX buffer (lock-free ring buffer)        │ │
│  │                                                               │ │
│  │  2. PIPELINE PHASE (deterministic, single-threaded)           │ │
│  │     ├── Axis Scaling     (existing C: AxisScaling.cpp)        │ │
│  │     ├── Soft Limits      (S-curve clamping)                   │ │
│  │     ├── Occupant Comp    (parasitic translation)              │ │
│  │     ├── Motion Cueing    (existing C: MotionCueing.cpp)       │ │
│  │     ├── Inverse Kinematics (existing C: InverseKinematics.cpp)│ │
│  │     └── Rate Limiter     (output smoothing)                   │ │
│  │                                                               │ │
│  │  3. OUTPUT PHASE                                              │ │
│  │     ├── Send binary packet → ESP32 serial                    │ │
│  │     ├── Send binary packet → ESP32 UDP (if WiFi active)      │ │
│  │     └── Update telemetry state from ESP32 serial RX           │ │
│  │                                                               │ │
│  │  4. RENDER PHASE (ImGui + OpenGL)                             │ │
│  │     ├── 3D Stewart platform visualization (direct OpenGL)     │ │
│  │     ├── Control panels (ImGui immediate mode)                 │ │
│  │     ├── Console log viewer                                    │ │
│  │     ├── Frequency charts                                      │ │
│  │     └── Config editors                                        │ │
│  │                                                               │ │
│  └───────────────────────────────────────────────────────────────┘ │
│                                                                   │
│  Background threads (only 2):                                     │
│  ├── Serial RX reader → lock-free ring buffer                    │
│  └── SimTools UDP recv → lock-free ring buffer                   │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

### Why this is the best low-level approach

| Property | Native C++ | Python server + browser |
|----------|-----------|------------------------|
| **Threads** | 3 total (main + serial RX + UDP RX) | 8 threads + async event loop |
| **Locks** | 0 on hot path (lock-free ring buffers) | 18 `threading.Lock()` |
| **Timing precision** | `QueryPerformanceCounter`: <1µs | `time.sleep()`: 1–15ms jitter |
| **IK call overhead** | Direct function call: ~2µs | CFFI bridge + Python dict: ~100µs |
| **Viz update** | Direct OpenGL in same frame: 0ms | HTTP poll → JSON → parse → Three.js: 5–50ms |
| **Memory** | Shared struct, zero-copy | JSON serialize → HTTP → deserialize per frame |
| **Failure mode** | One process, if it crashes it crashes clean | Thread A deadlocks, thread B spins, UI shows stale data |
| **Deployment** | Single `.exe` | Python install + pip + uvicorn + browser |
| **Code reuse with ESP32** | Same `.cpp` files, `#include` directly | CFFI bridge, separate compilation, struct mirroring |

---

## 3. Technology Stack

### Core (all C/C++, all in one executable)

| Component | Library | Why |
|-----------|---------|-----|
| **Window + OpenGL context** | [GLFW 3](https://www.glfw.org/) | Tiny, cross-platform, battle-tested. Gives you a window, OpenGL context, and input events. |
| **UI widgets** | [Dear ImGui](https://github.com/ocornut/imgui) | Immediate-mode GUI. Renders into your OpenGL frame. Sliders, tables, plots, text, docking — all built-in. Zero overhead. Used by every game engine and industrial tool. |
| **3D rendering** | Raw OpenGL 3.3+ | Stewart platform geometry is simple: 6 cylinders (arms), 6 cylinders (rods), 2 plates, 12 spheres (joints). ~200 lines of OpenGL. No engine needed. |
| **Plotting** | [ImPlot](https://github.com/epezent/implot) | ImGui-native plotting. Frequency charts, rate graphs, telemetry — drop-in. |
| **Serial I/O** | OS native (`CreateFile`/`ReadFile` on Windows, `termios` on Linux) | Direct, no wrapper overhead. Or use [libserialport](https://sigrok.org/wiki/Libserialport) for cross-platform. |
| **UDP sockets** | OS native (`winsock2` / POSIX) | Already doing this in Python — trivial in C. |
| **JSON config** | [cJSON](https://github.com/DaveGamble/cJSON) | Single `.c` file. Parse/write platform config, MCA presets, axis scales. |
| **BLE** (optional) | [SimpleBLE](https://github.com/OpenBluetoothToolbox/SimpleBLE) | C/C++ BLE library. Or defer — USB serial + WiFi UDP cover all cases. |
| **Math** | None needed | `<math.h>` is sufficient. IK/MCA already use only `sin`, `cos`, `sqrt`, `atan2`. |

### What you DON'T need

| Thing | Why not |
|-------|---------|
| Web server | No client/server split |
| Browser | ImGui renders natively at 60fps+ |
| JSON serialization for real-time data | Shared structs in same process |
| HTTP/WebSocket | No IPC needed |
| Node.js / npm / Vite | No web frontend |
| Python / pip / CFFI | C compiled directly |
| Threading locks on hot path | Lock-free ring buffers for serial/UDP |
| `asyncio` | Synchronous main loop |

---

## 4. Shared Code With ESP32 Firmware

This is the key advantage. The same `.cpp/.h` files compile for both targets:

```
Controller/
├── include/
│   ├── InverseKinematics.h    ← #include'd by both firmware AND desktop app
│   ├── AxisScaling.h          ←
│   ├── MotionCueing.h         ←
│   └── helpers.h              ← (already has #ifdef guards for desktop vs ESP32)
├── main/
│   ├── InverseKinematics.cpp  ← compiled by both
│   ├── AxisScaling.cpp        ←
│   └── MotionCueing.cpp       ←
```

The desktop app `#include`s these directly. No CFFI bridge, no duplicate code, no struct mirroring. When you fix a bug in IK, both ESP32 firmware and desktop app get the fix.

---

## 5. Application Layout (ImGui Panels)

```
┌──────────────────────────────────────────────────────────────────────┐
│  Stewart Platform Controller                              [_][□][X] │
├──────────────────────────────────────────────────────────────────────┤
│ ┌─ SIL ──────────────────────┐ ┌─ HIL (ESP32) ─────────────────┐   │
│ │                            │ │                                │   │
│ │   [3D Stewart Platform]    │ │   [3D Stewart Platform]        │   │
│ │    OpenGL viewport         │ │    OpenGL viewport             │   │
│ │    orbit/zoom/pan          │ │    from ESP32 telemetry        │   │
│ │                            │ │                                │   │
│ └────────────────────────────┘ └────────────────────────────────┘   │
│                                                                      │
│ ┌─ Input ───────────┐ ┌─ Servo Readout ──┐ ┌─ Transport ─────────┐ │
│ │ Surge [====|====] │ │ S0: -17.4° ✓     │ │ USB: COM3 Connected │ │
│ │ Sway  [====|====] │ │ S1: -55.7° ✓     │ │ UDP: 192.168.1.50 ✓│ │
│ │ Heave [====|====] │ │ S2:  51.8° ✓     │ │ BLE: Disconnected   │ │
│ │ Roll  [====|====] │ │ S3: -40.7° ✓     │ │ SimTools: 60 Hz     │ │
│ │ Pitch [====|====] │ │ S4: -60.0° ⚠     │ │ Fwd: USB ● On      │ │
│ │ Yaw   [====|====] │ │ S5: -56.1° ✓     │ │                     │ │
│ │ [Home All]        │ │                   │ │ Bits: [12▾]         │ │
│ └───────────────────┘ └───────────────────┘ └─────────────────────┘ │
│                                                                      │
│ ┌─ Config ──────────────────────────────────────────────────────┐   │
│ │ Geometry │ Motor │ Axis Scales │ MCA │ Soft Limits │ Occupant │   │
│ │ RD: 400.05  PD: 406.40  L1: 184.15  L2: 723.90  H: 648.13  │   │
│ └───────────────────────────────────────────────────────────────┘   │
│                                                                      │
│ ┌─ Console ─────────────────────────────────────────────────────┐   │
│ │ 20:15:32 [sil] IK #4770: pos=[152.0 0.0 14.6 ...] → [...]   │   │
│ │ 20:15:32 [esp32] IK #129: pos=[-154.9 ...] → [...]           │   │
│ │ 20:15:33 [hil] TX #17317: raw=[29818 32767 47840 ...]        │   │
│ │ [All] [SIL] [ESP32] [HIL] [SimTools]          [Clear] [▾10Hz]│   │
│ └───────────────────────────────────────────────────────────────┘   │
│                                                                      │
│ ┌─ Rates ──────────────────┐ ┌─ ESP32 ────────────────────────┐   │
│ │ SimTools In:  60.0 Hz    │ │ Port: [COM3 ▾] [Connect]       │   │
│ │ SIL IK:      100.0 Hz   │ │ FW: v1.2.3  [Build+Flash]      │   │
│ │ HIL TX:      60.0 Hz    │ │ Cmd: [____________] [Send]      │   │
│ │ ESP32 TEL:   10.0 Hz    │ │ Quick: [HOME] [PARK] [DBG]      │   │
│ └──────────────────────────┘ └─────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────────┘
```

ImGui supports **docking** — all these panels are drag-rearrangeable, can be floated, tabbed, or hidden. The layout is saved/restored automatically.

---

## 6. Threading Model (3 threads, 0 locks on hot path)

```
MAIN THREAD (render loop, 60+ fps)
│
├── Poll UDP ring buffer (non-blocking, lock-free SPSC)
├── Poll serial RX ring buffer (non-blocking, lock-free SPSC)
├── Run pipeline: AxisScale → SoftLim → Occupant → MCA → IK
├── Write to serial TX (direct, or via small queue)
├── Write to UDP TX (direct)
├── Render OpenGL 3D + ImGui
│
SERIAL RX THREAD
│  Blocking read from COM port
│  Parse TEL packets and text lines
│  Push to lock-free SPSC ring buffer → main thread
│
UDP RX THREAD (SimTools listener)
│  Blocking recvfrom() on port 4123
│  Push raw packet to lock-free SPSC ring buffer → main thread
```

**Lock-free SPSC (Single-Producer Single-Consumer) ring buffer:**
- ~30 lines of C
- One thread writes, one thread reads — no mutex, no contention
- Cache-line aligned, memory-ordered with atomics
- Standard pattern in audio engines, game engines, and real-time systems

The entire pipeline runs in the main thread. No locks. No contention. No GIL. Deterministic execution every frame.

---

## 7. Comparison: All Options Considered

| | Native C++ (ImGui+GL) | C Core + Web UI | Python + Browser (current) |
|---|---|---|---|
| **Processes** | 1 | 2 | 1 (but 8 threads + async) |
| **Hot-path threads** | 1 | 1 | 4+ |
| **Locks on hot path** | 0 | 0 | 18 |
| **Viz latency** | 0 ms (same frame) | 1-5ms (WebSocket) | 5-50ms (HTTP poll) |
| **IK call** | Direct: ~2µs | Direct: ~2µs | CFFI+Python: ~100µs |
| **Deployment** | Single `.exe` | 2 processes | Python + pip + browser |
| **UI flexibility** | ImGui (functional, not pretty) | React (rich, modern CSS) | Raw HTML (ugly but works) |
| **3D rendering** | OpenGL (native, fast) | Three.js (browser, good) | Three.js (browser, good) |
| **Serial I/O** | OS native, reliable | OS native, reliable | pyserial (wrapper, decent) |
| **Code reuse w/ ESP32** | Direct `#include` | Direct compile | CFFI bridge |
| **Iteration speed** | Recompile (~2s) | Hot reload (instant UI) | Refresh browser (instant) |
| **Learning curve** | OpenGL + ImGui | React + WebSocket | Already known |
| **Production quality** | ★★★★★ | ★★★★ | ★★ |
| **Long-term maintenance** | ★★★★★ | ★★★ | ★ |

**The native C++ option wins on every technical metric.** The only trade-off is UI aesthetics — ImGui is functional/industrial, not consumer-pretty. But for a motion simulator controller used by engineers, that's a feature, not a bug. vi-grade, dSPACE, NI — all use native UI.

---

## 8. Project Structure

```
6DOF-Rotary-Stewart-Motion-Simulator/
├── Controller/                          # ESP32 firmware (unchanged)
│   ├── include/                         # Shared headers
│   │   ├── InverseKinematics.h          ← shared
│   │   ├── AxisScaling.h               ← shared
│   │   ├── MotionCueing.h              ← shared
│   │   └── helpers.h                   ← shared (has #ifdef guards)
│   └── main/                           # Shared implementations
│       ├── InverseKinematics.cpp        ← shared
│       ├── AxisScaling.cpp             ← shared
│       └── MotionCueing.cpp            ← shared
│
├── app/                                 # Native desktop application
│   ├── CMakeLists.txt                   # Build system
│   ├── src/
│   │   ├── main.cpp                     # Entry point, GLFW window, main loop
│   │   ├── app.h / app.cpp             # Application state, pipeline, settings persistence
│   │   ├── platform_viz.h / .cpp       # OpenGL 3D Stewart platform renderer
│   │   ├── ui_panels.h / .cpp          # ImGui panel definitions (all UI)
│   │   └── serial_port.h / .cpp        # Cross-platform serial I/O + binary protocol
│
└── docs/
    └── ARCHITECTURE_ROADMAP.md
```

---

## 9. Migration Status

### Phase 1: Skeleton app with 3D viz ✔️
- CMake project, GLFW + OpenGL + ImGui + ImPlot
- Shared C modules compiled directly (IK, AxisScaling, MotionCueing)
- 3D Stewart platform renderer with orbit camera
- 6-DOF sliders → direct IK → 3D update in same frame

### Phase 2: Serial bridge + ESP32 ✔️
- Serial RX/TX with binary protocol (0xAA/0x55 framing)
- TEL packet parsing, telemetry display, real-time rate monitoring
- Connect/disconnect UI, auto-detect COM port, auto-reconnect
- TX rate configurable up to 1000 Hz

### Phase 3: SimTools + full pipeline ✔️
- UDP listener thread for SimTools input
- Full pipeline: axis scaling → IK
- SimTools start/stop, bit depth config
- JSON settings persistence with auto-save

### Phase 4: Beyond parity ✔️
- Console log panel with per-entity filtering and configurable rate
- Multi-entity test bench (SIL + HIL side-by-side)
- Test signal generator with per-axis waveforms, smooth interpolation, presets
- Motion capture recording and playback with speed control and looping
- Rolling spectrogram heatmap with multi-entity/axis overlay
- Input source persistence (restores last-used source on startup)
- Capture library management with delete support

### Phase 5: Future work
- **Direct game telemetry plugins**: iRacing, Assetto Corsa shared memory readers
- **Hardware E-stop chain**: Monitor GPIO state, auto-park on fault
- **WiFi/UDP transport management**: ESP32 WiFi credential config from desktop app
- **BLE transport** (deferred — serial + WiFi UDP cover all current use cases)
- **Multi-viewport**: Undock panels to second monitor
- **Profile system**: Save/load complete configurations (geometry + MCA + scales + limits)

---

## 10. Build System

```cmake
cmake_minimum_required(VERSION 3.16)
project(stewart-platform LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

# GLFW
add_subdirectory(vendor/glfw)

# Shared C modules from ESP32 firmware
set(SHARED_SOURCES
    ${CMAKE_SOURCE_DIR}/../Controller/main/InverseKinematics.cpp
    ${CMAKE_SOURCE_DIR}/../Controller/main/AxisScaling.cpp
    ${CMAKE_SOURCE_DIR}/../Controller/main/MotionCueing.cpp
)

# Application
add_executable(stewart-platform
    src/main.cpp
    src/app.cpp
    src/platform_viz.cpp
    src/ui_panels.cpp
    src/serial_bridge.cpp
    src/udp_transport.cpp
    src/config.cpp
    src/console.cpp
    ${SHARED_SOURCES}
    vendor/imgui/imgui.cpp
    vendor/imgui/imgui_draw.cpp
    vendor/imgui/imgui_tables.cpp
    vendor/imgui/imgui_widgets.cpp
    vendor/imgui/backends/imgui_impl_glfw.cpp
    vendor/imgui/backends/imgui_impl_opengl3.cpp
    vendor/implot/implot.cpp
    vendor/implot/implot_items.cpp
    vendor/glad/glad.c
    vendor/cJSON/cJSON.c
)

target_include_directories(stewart-platform PRIVATE
    src
    ${CMAKE_SOURCE_DIR}/../Controller/include
    vendor/imgui
    vendor/imgui/backends
    vendor/implot
    vendor/glad
    vendor/cJSON
)

target_link_libraries(stewart-platform PRIVATE glfw)

# Windows: link winsock2 for UDP, setupapi for serial
if(WIN32)
    target_link_libraries(stewart-platform PRIVATE ws2_32 setupapi)
endif()
```

Build: `cmake -B build && cmake --build build`
Run: `build/stewart-platform.exe`

---

## 11. Key Implementation Notes

### Lock-free SPSC ring buffer (~30 lines)
```cpp
template<typename T, size_t N>
struct RingBuffer {
    std::array<T, N> buf;
    std::atomic<size_t> head{0};  // writer increments
    std::atomic<size_t> tail{0};  // reader increments

    bool push(const T& item) {
        size_t h = head.load(std::memory_order_relaxed);
        size_t next = (h + 1) % N;
        if (next == tail.load(std::memory_order_acquire)) return false; // full
        buf[h] = item;
        head.store(next, std::memory_order_release);
        return true;
    }

    bool pop(T& item) {
        size_t t = tail.load(std::memory_order_relaxed);
        if (t == head.load(std::memory_order_acquire)) return false; // empty
        item = buf[t];
        tail.store((t + 1) % N, std::memory_order_release);
        return true;
    }
};
```

### ImGui is immediate-mode (no state management)
```cpp
// This is the entire slider panel. Every frame. No callbacks, no state sync.
void DrawSliderPanel(float position[6]) {
    ImGui::Begin("Input");
    bool changed = false;
    changed |= ImGui::SliderFloat("Surge", &position[0], -200, 200);
    changed |= ImGui::SliderFloat("Sway",  &position[1], -200, 200);
    changed |= ImGui::SliderFloat("Heave", &position[2], -200, 200);
    changed |= ImGui::SliderFloat("Roll",  &position[3], -30, 30);
    changed |= ImGui::SliderFloat("Pitch", &position[4], -30, 30);
    changed |= ImGui::SliderFloat("Yaw",   &position[5], -30, 30);
    if (ImGui::Button("Home All")) { memset(position, 0, 6*sizeof(float)); }
    ImGui::End();
    // 'changed' is true if user moved a slider this frame — pipeline runs automatically
}
```

### 3D viz is just basic OpenGL
The Stewart platform geometry is trivial:
- 6 cylinders for servo arms (oriented by IK angle)
- 6 cylinders for connecting rods (oriented between arm tip and platform joint)
- 2 extruded polygons for base and platform plates
- 12 spheres for joints
- Orbit camera (ImGui handles mouse input)

Total: ~200 lines of OpenGL draw code. No engine, no scene graph, no asset loading.

---

## 12. Multi-Instance Test Bench Architecture

The native C++ architecture enables something the Python server never could: **dynamically spawning independent SIL and HIL entities**, each with their own pipeline, settings, 3D visualizer, and optional ESP32 connection — all running in the same process, same frame, fed from the same SimTools input.

### Concept

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Stewart Platform Controller                                  [+ Add] │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  SimTools Input (shared)  ──→  broadcast to all entities               │
│  ┌──────────────────────┐                                               │
│  │ UDP :4123  60 Hz     │                                               │
│  │ Raw: 128,128,128,... │                                               │
│  └──────────┬───────────┘                                               │
│             │                                                           │
│    ┌────────┼────────────────────┬──────────────────────┐              │
│    ▼        ▼                    ▼                      ▼              │
│                                                                         │
│  ┌─ Entity 0 (SIL) ─────┐  ┌─ Entity 1 (HIL) ─────┐  ┌─ Entity 2 ─┐ │
│  │ Name: "Gentle MCA"    │  │ Name: "ESP32 #1"      │  │ "Race Pro" │ │
│  │ Type: SIL (local IK)  │  │ Type: HIL (COM3)      │  │ SIL        │ │
│  │                        │  │                        │  │            │ │
│  │ ┌──────────────────┐  │  │ ┌──────────────────┐  │  │ ┌────────┐ │ │
│  │ │ [3D Platform]    │  │  │ │ [3D Platform]    │  │  │ │ [3D]   │ │ │
│  │ │  from local IK   │  │  │ │  from ESP32 TEL  │  │  │ │        │ │ │
│  │ └──────────────────┘  │  │ └──────────────────┘  │  │ └────────┘ │ │
│  │                        │  │                        │  │            │ │
│  │ Pipeline:              │  │ Pipeline:              │  │ Pipeline:  │ │
│  │  MCA: Gentle           │  │  MCA: Moderate         │  │  MCA: Pro  │ │
│  │  Bits: 12              │  │  Bits: 16              │  │  Bits: 12  │ │
│  │  Scales: 8,8,7,30,30  │  │  Scales: 10,10,8,35   │  │  Custom    │ │
│  │  Occupant: 0,0,800    │  │  Occupant: 50,0,750   │  │  0,0,800   │ │
│  │  Soft lim: ±150mm     │  │  Soft lim: ±120mm     │  │  ±180mm    │ │
│  │                        │  │                        │  │            │ │
│  │ Output:                │  │ Output:                │  │ Output:    │ │
│  │  Local IK only         │  │  → COM3 (USB serial)  │  │  Local IK  │ │
│  │  (no hardware)         │  │  TX: 60 Hz, RX: 10 Hz │  │            │ │
│  │                        │  │  Telemetry: ✓          │  │            │ │
│  │ Angles:                │  │ Angles (ESP32):        │  │ Angles:    │ │
│  │  -17.4° -55.7° 51.8°  │  │  -17.3° -55.5° 51.9°  │  │  -22.1° …  │ │
│  │  -40.7° -60.0° -56.1° │  │  -40.6° -59.8° -56.0° │  │            │ │
│  │                        │  │                        │  │            │ │
│  │ [Settings] [Remove]    │  │ [Settings] [Remove]    │  │ [⚙] [✕]  │ │
│  └────────────────────────┘  └────────────────────────┘  └────────────┘ │
│                                                                         │
│  ┌─ Comparison Overlay ────────────────────────────────────────────┐   │
│  │  Servo 0: Gentle=-17.4°  ESP32=-17.3°  RacePro=-22.1°  Δ=4.7° │   │
│  │  Servo 1: Gentle=-55.7°  ESP32=-55.5°  RacePro=-48.2°  Δ=7.5° │   │
│  │  ...                                                             │   │
│  │  [Overlay all on single 3D viz]  [Export comparison CSV]         │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  ┌─ Console (all entities, color-coded) ───────────────────────────┐   │
│  │ 20:15:32 [E0/sil] IK #4770: angles=[-17.4° -55.7° ...]        │   │
│  │ 20:15:32 [E1/esp32] IK #129: angles=[-17.3° -55.5° ...]       │   │
│  │ 20:15:32 [E2/sil] IK #4770: angles=[-22.1° -48.2° ...]        │   │
│  │ [All] [E0] [E1] [E2]                              [Clear]      │   │
│  └─────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
```

### Data Model

```cpp
enum class EntityType { SIL, HIL };

struct PipelineConfig {
    StewartConfig    geometry;        // RD, PD, L1, L2, H, theta_r, theta_p
    AxisScaleConfig  axis_scales;     // per-axis scale + is_angle
    MotionCueingConfig mca;           // preset + per-axis gains
    float            soft_limits[6];  // per-axis S-curve limits
    float            occupant[3];     // x, y, z offset
    int              bit_depth;       // 8, 10, 12, 14, 16
};

struct Entity {
    int              id;
    char             name[64];
    EntityType       type;              // SIL or HIL
    PipelineConfig   config;            // fully independent settings

    // Pipeline state (updated every frame)
    float            input_raw[6];      // from SimTools (shared) or manual sliders
    float            input_scaled[6];   // after axis scaling
    float            input_filtered[6]; // after MCA
    float            output_angles[6];  // IK result (radians)
    int              valid_mask;
    int              ik_seq;

    // HIL-specific (only if type == HIL)
    SerialBridge*    serial;            // ESP32 connection (nullptr for SIL)
    UdpTransport*    udp;              // WiFi transport (optional)
    float            esp32_angles[6];   // telemetry from hardware
    int              esp32_tel_seq;

    // Viz state
    PlatformViz      viz;               // independent OpenGL 3D renderer
    bool             show_settings;     // settings panel open
    ImVec4           color;             // entity color for comparison overlay
};

struct App {
    std::vector<Entity> entities;       // dynamically add/remove
    float               shared_input[6]; // from SimTools, broadcast to all
    bool                simtools_active;
    // ...
};
```

### Main Loop With Multiple Entities

```cpp
void App::Update() {
    // 1. Read shared SimTools input (if active)
    SimToolsPacket pkt;
    while (simtools_rx.pop(pkt)) {
        memcpy(shared_input, pkt.raw, sizeof(shared_input));
    }

    // 2. Process each entity independently
    for (auto& e : entities) {
        // Input: shared SimTools or entity's own manual sliders
        float input[6];
        if (simtools_active) {
            memcpy(input, shared_input, sizeof(input));
        } else {
            memcpy(input, e.input_raw, sizeof(input));
        }

        // Pipeline (each entity has its own config)
        mapRawToPosition(input, &e.config.axis_scales,
                         (1 << e.config.bit_depth) - 1, e.input_scaled);
        applySoftLimits(e.input_scaled, e.config.soft_limits, e.input_filtered);
        applyOccupantComp(e.input_filtered, e.config.occupant);
        processMotionCueing(&e.config.mca, e.input_filtered, e.input_filtered);
        calculateAllServoAngles(e.input_filtered, &e.config.geometry, e.output_angles);
        e.valid_mask = validatePosition(e.input_filtered, &e.config.geometry);
        e.ik_seq++;

        // Output to hardware (HIL only)
        if (e.type == EntityType::HIL && e.serial) {
            e.serial->sendBinaryPacket(input);  // raw values to ESP32
        }

        // Read ESP32 telemetry (HIL only)
        if (e.serial) {
            TelPacket tel;
            while (e.serial->rx_buffer.pop(tel)) {
                memcpy(e.esp32_angles, tel.angles, sizeof(e.esp32_angles));
                e.esp32_tel_seq++;
            }
        }
    }

    // 3. Render all entities
    for (auto& e : entities) {
        float* angles = (e.type == EntityType::HIL && e.esp32_tel_seq > 0)
                        ? e.esp32_angles : e.output_angles;
        e.viz.Draw(e.input_filtered, angles, &e.config.geometry);
    }

    // 4. Draw ImGui panels for each entity + comparison overlay
    DrawEntityPanels();
    DrawComparisonOverlay();
    DrawConsole();
}
```

### Use Cases This Enables

| Scenario | How |
|----------|-----|
| **Compare MCA presets** | 3 SIL entities: Gentle, Moderate, Race Pro. Same SimTools input. Watch how each filters the motion differently in real-time. |
| **Validate ESP32 IK match** | 1 SIL + 1 HIL (same config). SIL computes locally, HIL gets ESP32 telemetry. Compare angles — should be identical. |
| **Bit depth comparison** | 3 SIL entities at 8-bit, 12-bit, 16-bit. See quantization effects on motion. |
| **Occupant offset tuning** | 2 entities with different occupant positions. See parasitic motion difference. |
| **Multi-hardware test bench** | 3 ESP32s on COM3, COM4, COM5. All receive same SimTools data. Compare firmware versions or config differences. |
| **Regression testing** | Record a SimTools session. Replay it through multiple configs. Export CSV for offline analysis. |
| **A/B tuning** | While driving, have 2 entities running. Switch which one drives the physical platform. Instant A/B comparison. |

### Threading Impact

Adding entities does NOT add threads. The pipeline is single-threaded (main loop). Each HIL entity adds one serial RX thread (dedicated, blocking read), but these are trivial — just pushing to lock-free ring buffers.

| Entities | Threads |
|----------|---------|
| 1 SIL | 2 (main + UDP RX) |
| 1 SIL + 1 HIL | 3 (main + UDP RX + 1 serial RX) |
| 3 SIL + 2 HIL | 4 (main + UDP RX + 2 serial RX) |
| 5 SIL + 3 HIL | 5 (main + UDP RX + 3 serial RX) |

Compare to current architecture: 8 threads + 18 locks for a SINGLE entity.

---

## 13. IK Generalization & Workspace Abstraction (Phases A–D)

See `docs/IK_RESEARCH.md` for full analysis. The closed-form rotary IK (asin/atan2) is correct — these phases improve **parameterization** and **user-facing presentation**.

### Phase A: Generalize IK to Per-Actuator Definition

Replace the symmetric 3-pair topology (DxMultiplier, AngleMultiplier, OffsetAngle arrays) with per-actuator parameterization:

```cpp
struct ActuatorDef {
    float base_pos[3];        // B_k: servo shaft position on base [x,y,z]
    float platform_pos[3];    // P_k: ball joint on platform (platform frame) [x,y,z]
    float beta;               // β_k: servo axis orientation angle (radians)
    float L1, L2;             // arm + rod lengths (can differ per motor)
};
```

- Same e/f/g/asin math, same speed, same accuracy
- Supports arbitrary motor placement, non-planar bases, mixed arm lengths
- Helper function generates ActuatorDef[6] from compact form (RD, PD, θr, θp, L1, L2) for backward compat
- Shared between desktop app and ESP32 firmware

### Phase B: Normalized Percentage Input Layer

Replace raw mm/degree slider limits with normalized -100% to +100%:

- **100% = max safe displacement** (computed from geometry via workspace binary search)
- User sees: `Surge: 75%` → internally `75% × 177mm = 133mm`
- SimTools raw integers map to % naturally: `raw / max_raw × 100%`
- Physical units shown as small annotation: `(±177mm)` tooltip, not primary control

### Phase C: Real-Time Workspace Feedback

Replace boolean `valid_mask` with gradient visual feedback:

- **Servo headroom bars**: per-servo 0-100% of ±60° limit (green/yellow/red)
- **Workspace utilization gauge**: max servo utilization across all 6
- **Proximity warnings**: entity card border color shifts green→yellow→red
- **Manipulability indicator** (advanced): Jacobian condition number → "Good/Marginal/Near Singularity"

### Phase D: Motion Intensity Abstraction

- **Global Motion Intensity** knob (0-100%): uniformly scales all axes
- **Per-axis gain trim**: e.g., "Heave: 120%" allows intentional extra range
- Matches vi-grade DiM250, Motion4Sim, FlyPT Mover industry pattern
- User never needs to understand raw workspace dimensions

---

## Summary

**One executable. One render loop. Zero servers. Zero browsers. Zero JSON serialization on the hot path. Direct C function calls. Shared source files with ESP32 firmware. Single-binary deployment. Dynamic multi-instance entities for real-time comparison testing. Per-actuator generalized IK with normalized percentage input.**

This is how vi-grade, dSPACE, and NI build their tools. It's the correct architecture for a real-time motion simulator controller.
