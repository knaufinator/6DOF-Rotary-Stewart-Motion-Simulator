#pragma once

#include <vector>
#include <string>
#include <array>
#include <cstring>
#include <cmath>
#include <mutex>
#include <atomic>
#include <thread>
#include <memory>

// History ring buffer size (seconds × expected Hz)
static const int HISTORY_LEN = 600;   // ~10s at 60fps
static const int SPECTRUM_BINS = 128; // FFT/DFT bins

// Global input history (source-agnostic, feeds time series + spectrogram)
static const int INPUT_HISTORY_LEN = 2048;  // ~10s at 200Hz

// Shared C headers from ESP32 firmware
#include "InverseKinematics.h"
#include "AxisScaling.h"
#include "MotionCueing.h"

// Forward declaration
class SerialPort;

// ── Entity Types ────────────────────────────────────────────────────

enum class EntityType { SIL, HIL };

struct PipelineConfig {
    StewartConfig       geometry;       // compact form (backward compat)
    PlatformDef         platform;       // generalized per-actuator form
    AxisScaleConfig     axis_scales;    // workspace limits (internal, hidden from user)
    MotionCueingConfig  mca;
    float               occupant[3];
    int                 bit_depth;

    // Phase D: intensity & gain
    float               intensity;      // global motion intensity 0-100%
    float               axis_gain[6];   // per-axis gain multiplier (default 100%)

    void initDefaults();
    void rebuildPlatform();             // rebuild PlatformDef from geometry
};

struct PipelineState {
    float input_pct[6];         // user-facing: -100% to +100%
    float input_physical[6];    // internal: mm / radians (converted from pct)
    float output_angles[6];     // servo angles (radians)
    float output_angles_deg[6]; // servo angles (degrees)
    float output_steps[6];      // motor steps
    float servo_util[6];        // Phase C: per-servo utilization 0-100%
    float max_util;             // Phase C: max utilization across all servos
    int   valid_mask;
    int   ik_seq;
};

struct TransportState {
    bool  usb_connected;
    char  usb_port[32];
    int   usb_tx_count;
    int   usb_rx_count;
    bool  udp_connected;
    char  udp_ip[32];
    bool  ble_connected;
};

// ── Console Log ─────────────────────────────────────────────────────

struct LogEntry {
    double      timestamp;
    int         entity_id;
    char        source[16];   // "sil", "esp32", "hil", "system"
    char        message[256];
};

// ── 3D Visualization Camera ──────────────────────────────────────────

struct VizCamera {
    float azimuth;      // horizontal orbit angle (radians)
    float elevation;    // vertical orbit angle (radians)
    float distance;     // distance from target (mm), 0 = auto
};

// ── HIL Protocol ────────────────────────────────────────────────────

enum class HilProtocol { Binary, CSV };

// ── Entity ──────────────────────────────────────────────────────────

struct Entity {
    int             id;
    char            name[64];
    EntityType      type;
    bool            enabled;
    PipelineConfig  config;
    PipelineState   state;
    TransportState  transport;
    float           color[4];       // RGBA for UI identification
    bool            show_settings;
    bool            show_platform;  // Platform Setup window
    bool            show_dynamics;  // Dynamics window
    VizCamera       viz_cam;

    // Rates
    float           rate_ik_hz;
    float           rate_tx_hz;
    float           rate_tel_hz;

    // HIL: serial connection to ESP32
    std::shared_ptr<SerialPort> serial;  // nullptr for SIL entities
    int             hil_tx_hz;           // target motion packet send rate
    double          hil_last_tx_time;    // last binary packet send time
    int             hil_tel_seq;         // last processed telemetry seq
    bool            hil_tel_active;      // true when ESP32 telemetry is overriding local IK
    char            hil_port[32];        // selected COM port name
    bool            hil_auto_connect;    // try to reconnect if disconnected
    double          hil_last_reconnect;  // last reconnect attempt time
    HilProtocol     hil_protocol;        // Binary (big platform) or CSV (Mini-6DOF)
    uint16_t        hil_tx_raw[6];       // latest raw packet for background TX thread

    // Time-series history ring buffers (for comparison charts)
    float           history_angles[6][HISTORY_LEN];   // servo angles over time (deg)
    float           history_input[6][HISTORY_LEN];     // scaled input over time
    float           history_time[HISTORY_LEN];         // timestamps (seconds)
    int             history_head;                       // write index
    int             history_count;                      // how many valid samples

    // Placeholder: frequency spectrum per axis (not wired yet)
    float           spectrum[6][SPECTRUM_BINS];         // magnitude bins
    float           spectrum_freq_max;                  // max frequency (Hz)

    void            pushHistory(double t);              // record current state
};

// ── Recording / Playback ───────────────────────────────────────────

enum class RecordMode { Idle, Recording };

struct RecordSample {
    double time;        // seconds since record start
    float  input[6];    // 6-DOF input values at this moment
};

struct RecordingState {
    RecordMode                  mode;
    std::vector<RecordSample>   samples;
    double                      start_time;     // wall time when record started
    char                        name[64];       // recording label

    RecordingState() : mode(RecordMode::Idle), start_time(0) {
        snprintf(name, sizeof(name), "Recording");
    }

    double duration() const {
        if (samples.empty()) return 0.0;
        return samples.back().time;
    }
};

// ── Versioned Recording File Format ────────────────────────────────

#define RECORDING_MAGIC   0x53545752   // "STWR"
#define RECORDING_VERSION 1

struct RecordingFileHeader {
    uint32_t magic;              // RECORDING_MAGIC
    uint32_t version;            // format version
    double   sample_rate_hz;     // recording sample rate
    double   duration_sec;       // total duration
    int32_t  sample_count;       // number of RecordSample entries
    int32_t  channels;           // number of channels (6)
    double   created_time;       // unix timestamp when recorded
    char     name[64];           // recording name
    char     source[32];         // input source ("manual", "udp", "capture")
    uint8_t  reserved[128];      // future use — zero-filled
};

// ── Saved Recording Library ────────────────────────────────────────

struct SavedRecording {
    char                        name[64];
    double                      sample_rate_hz;
    double                      created_time;   // unix timestamp
    char                        source[32];
    std::vector<RecordSample>   samples;
    double duration() const {
        if (samples.empty()) return 0.0;
        return samples.back().time;
    }
};

// ── Input Source ────────────────────────────────────────────────────

enum class InputSource { Manual, SimToolsUDP, CapturePlayback, TestSignal };

// ── Test Signal Generator ──────────────────────────────────────────

enum class WaveformType { Sine, Square, Triangle, Sawtooth };

struct TestSignalConfig {
    bool          enabled;           // generator active
    WaveformType  waveform;          // waveform shape
    float         frequency[6];      // Hz per axis (target)
    float         amplitude[6];      // % per axis (target, 0-100)
    float         phase_offset[6];   // degrees per axis (target)
    bool          axis_enabled[6];   // per-axis on/off
    bool          ramp_up;           // S-curve ramp envelope on start
    float         ramp_duration;     // ramp-up time in seconds
    bool          smooth_changes;    // smoothly interpolate parameter changes
    float         smooth_rate;       // interpolation speed (units/sec, higher=faster)

    // Active (smoothed) values used by the generator
    float         active_freq[6];
    float         active_amp[6];
    float         active_phase[6];

    TestSignalConfig() : enabled(false), waveform(WaveformType::Sine),
                         ramp_up(true), ramp_duration(2.0f),
                         smooth_changes(true), smooth_rate(2.0f) {
        for (int i = 0; i < 6; i++) {
            frequency[i] = 1.0f;
            amplitude[i] = 50.0f;
            phase_offset[i] = 0.0f;
            axis_enabled[i] = true;
            active_freq[i] = 1.0f;
            active_amp[i] = 50.0f;
            active_phase[i] = 0.0f;
        }
    }
};

// ── UDP Listener State ──────────────────────────────────────────────

struct UdpListenerState {
    std::atomic<bool>   running{false};
    std::thread         thread;
    int                 sock = -1;          // SOCKET handle (INVALID_SOCKET on Windows)

    // Stats (written by listener thread, read by main thread)
    std::atomic<int>    packets_received{0};
    std::atomic<int>    packets_bad{0};
    std::atomic<double> last_packet_time{0.0};
    std::atomic<float>  rate_hz{0.0f};

    // Rate tracking internals (listener thread only)
    double              rate_window_start = 0.0;
    int                 rate_window_count = 0;
};

// ── Application State ───────────────────────────────────────────────

struct App {
    std::vector<Entity>   entities;
    int                   next_entity_id;

    // Shared input (from SimTools or manual override)
    float                 shared_input[6];
    std::mutex            input_mutex;       // protects shared_input when UDP writes
    InputSource           input_source;

    // SimTools UDP config
    bool                  simtools_active;   // is UDP listener running?
    int                   simtools_port;
    int                   simtools_bit_depth;
    float                 simtools_rate;
    UdpListenerState      udp;

    // Console
    std::vector<LogEntry> console_log;
    int                   console_max;
    bool                  console_auto_scroll;
    int                   console_filter;       // -1 = all, else entity_id

    // Global
    bool                  running;
    double                frame_time;
    double                fps;
    int                   frame_count;

    // Recording / Playback
    RecordingState        recording;

    // Saved recordings library
    std::vector<SavedRecording> saved_recordings;
    int                   capture_playback_idx;   // index into saved_recordings (-1 = none)
    bool                  capture_playing;         // is capture playback active?
    int                   capture_play_cursor;
    double                capture_start_time;
    bool                  capture_loop;
    float                 capture_speed;           // playback speed multiplier (0.1 = 10%, 2.0 = 200%)

    // Configurable record/playback rate
    int                   record_rate_hz;          // default 200

    // Test signal generator
    TestSignalConfig      test_signal;
    double                test_signal_start_time;  // when generator was started

    // Test signal presets
    struct TestSignalPreset {
        char              name[64];
        TestSignalConfig  config;
    };
    std::vector<TestSignalPreset> test_signal_presets;
    void    saveTestSignalPreset(const char* name);
    void    deleteTestSignalPreset(int idx);
    void    saveTestSignalPresetsToDisk();
    void    loadTestSignalPresetsFromDisk();

    // Global input history (source-agnostic ring buffer for time series & spectrogram)
    float                 input_history[6][INPUT_HISTORY_LEN];
    float                 input_history_time[INPUT_HISTORY_LEN];
    int                   input_history_head;
    int                   input_history_count;
    double                input_history_last_push;  // timestamp of last push

    // Input spectrum (DFT of input_history per axis)
    float                 input_spectrum[6][SPECTRUM_BINS];
    float                 input_spectrum_freq_max;

    // Console input logging rate (user-controlled)
    double                last_input_log_time;
    int                   console_log_rate;    // 0=off, 1=1Hz, 2=10Hz, 3=30Hz, 4=60Hz, 5=every frame

    App();
    ~App();

    Entity& addEntity(const char* name, EntityType type);
    void    removeEntity(int id);
    Entity* findEntity(int id);
    void    log(int entity_id, const char* source, const char* fmt, ...);
    void    update();   // called every frame

    // UDP listener
    bool    startUdpListener();
    void    stopUdpListener();

    // Recording
    void    startRecording();
    void    stopRecording();
    // Capture library
    void    saveRecordingToLibrary(const char* name);
    void    deleteRecordingFromLibrary(int idx);
    void    startCapturePlayback(int idx);
    void    stopCapturePlayback();
    void    updateCapturePlayback();
    void    saveRecordingsToDisk();
    void    loadRecordingsFromDisk();

    // Settings persistence (auto-save on change)
    void    saveSettings();
    void    loadSettings();
    bool    settings_dirty;    // set true when any setting changes, checked each frame

    // Background HIL TX thread (runs independently of UI frame rate)
    std::thread       hil_tx_thread;
    std::atomic<bool> hil_tx_stop{false};
    void              hilTxLoop();
};

// Global app instance
extern App g_app;
