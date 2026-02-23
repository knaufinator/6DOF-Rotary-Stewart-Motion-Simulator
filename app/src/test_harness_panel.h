#pragma once
/*
 * Test Harness Panel — Step/Dir Signal Analyzer integration
 *
 * Connects to the second ESP32-S3 (orange LED) running the step_dir_analyzer
 * firmware. Displays per-motor metrics, supports running RATETEST on the
 * controller and capturing external verification data, and generates reports.
 *
 * USB Serial# 5A84016326 = test harness (auto-detected).
 */

#include <string>
#include <vector>
#include <array>
#include <memory>
#include <mutex>
#include <atomic>
#include <chrono>

class SerialPort;   // forward from serial_port.h
struct Entity;      // forward from app.h

// ── Per-motor data parsed from test harness JSON ────────────────────

struct AnalyzerMotor {
    int      id           = -1;
    int32_t  position     = 0;
    uint32_t total_steps  = 0;
    float    rate_hz      = 0.0f;
    int      dir          = 0;
    uint32_t min_us       = 0;
    uint32_t max_us       = 0;
    uint32_t avg_us       = 0;
    uint32_t dir_changes  = 0;
    uint32_t idle_ms      = 0;
};

// ── Test result snapshot (for report generation) ────────────────────

struct TestSnapshot {
    std::string                     timestamp;      // ISO-like string
    std::string                     test_name;      // e.g. "RATETEST:50000:6"
    int                             steps_requested = 0;
    int                             num_motors      = 0;

    // Controller-side results — pipeline (one-shot burst)
    float                           pipe_rate_hz    = 0.0f;
    float                           pipe_pct_max    = 0.0f;
    int64_t                         pipe_time_us    = 0;
    std::array<int, 6>              pipe_errors     = {};

    // Controller-side results — continuous (hardware-counted)
    float                           ctrl_rate_hz    = 0.0f;
    float                           ctrl_pct_max    = 0.0f;
    int64_t                         ctrl_time_us    = 0;
    std::array<int, 6>              ctrl_errors     = {};
    std::array<std::string, 6>      ctrl_mode;      // "PCNT" / "RMT" / "ISR"

    // Test harness external measurement
    std::array<AnalyzerMotor, 6>    analyzer;
    bool                            analyzer_valid  = false;

    // Pass/fail
    bool                            passed          = false;
    std::string                     verdict;
};

// ── Pin check result ────────────────────────────────────────────────

struct PinCheck {
    std::array<int, 6>  step_levels = {};   // GPIO levels for STEP pins
    std::array<int, 6>  dir_levels  = {};   // GPIO levels for DIR pins
    bool                valid       = false;
    double              timestamp   = 0.0;
};

// ── Test Harness State ──────────────────────────────────────────────

enum class TestPhase {
    Idle,
    WaitRatetest,   // waiting for RATETEST:DONE from controller
    WaitAnalyzer,   // settling — collecting analyzer data after RATETEST
    Done            // results ready
};

struct TestHarnessState {
    // Connection
    std::shared_ptr<SerialPort>     serial;
    char                            port[32]        = {};
    bool                            auto_detected   = false;

    // Live motor data (updated from polling)
    std::array<AnalyzerMotor, 6>    motors;
    bool                            motors_valid    = false;
    double                          last_poll_time  = 0.0;

    // Rolling window for max_us (recent peak, not all-time)
    static constexpr int MAX_US_WINDOW = 20;  // ~3 seconds at 0.15s poll
    uint32_t max_us_ring[6][MAX_US_WINDOW] = {};
    int      max_us_ring_head = 0;
    int      max_us_ring_count = 0;
    uint32_t max_us_prev[6] = {};              // previous poll's raw max_us (for delta)

    // Pin check
    PinCheck                        pin_check;

    // Test execution
    TestPhase                       phase           = TestPhase::Idle;
    int                             test_steps      = 50000;
    int                             test_motor_count = 6;
    std::vector<std::string>        ctrl_lines;     // raw controller output
    double                          test_start_time = 0.0;

    // Snapshots / report history
    std::vector<TestSnapshot>       snapshots;

    // UI
    bool                            show_panel      = false;
    bool                            streaming       = false;
};

// ── Public API ──────────────────────────────────────────────────────

// Call once at startup
void TestHarnessInit();

// Called every frame from DrawUI (standalone window — legacy)
void DrawTestHarnessPanel();

// Draw test harness as a tab inside the HIL entity card.
// Uses entity's serial for controller commands; analyzer connection is internal.
void DrawTestHarnessTabContent(Entity& e);

// Access global state
TestHarnessState& GetTestHarnessState();
