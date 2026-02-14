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
#include "plugin_manager.h"

// Forward declaration
class SerialPort;

// ── Platform Types ──────────────────────────────────────────────────

enum class PlatformType {
    Stepper,    // AASD-15A / closed-loop stepper with gearbox (main platform)
    PWMServo    // Hobby PWM servos (Mini-6DOF)
};

struct ServoConfig {
    int   center_us[6];     // per-servo center pulse width (µs)
    float pulse_per_rad;    // µs per radian of servo rotation
    int   min_pulse_us;     // minimum pulse width (µs)
    int   max_pulse_us;     // maximum pulse width (µs)
    int   pwm_freq_hz;      // PWM frequency (Hz)
    bool  inverted[6];      // per-servo inversion flag

    void initDefaults() {
        for (int i = 0; i < 6; i++) center_us[i] = 1500;
        pulse_per_rad = 450.0f;
        min_pulse_us = 800;
        max_pulse_us = 2200;
        pwm_freq_hz = 50;
        inverted[0] = true;  inverted[1] = false;
        inverted[2] = true;  inverted[3] = false;
        inverted[4] = true;  inverted[5] = false;
    }
};

// ── Entity Types ────────────────────────────────────────────────────

enum class EntityType { SIL, HIL };

struct PipelineConfig {
    StewartConfig       geometry;       // compact form (backward compat)
    PlatformDef         platform;       // generalized per-actuator form
    AxisScaleConfig     axis_scales;    // workspace limits (internal, hidden from user)
    MotionCueingConfig  mca;
    InputFilterConfig   input_filter;   // pre-MCA signal conditioning
    float               occupant[3];
    int                 bit_depth;

    // Platform type — determines which drive train controls to show
    PlatformType        platform_type;
    ServoConfig         servo;          // PWM servo params (only used when platform_type == PWMServo)

    // Phase D: intensity & gain
    float               intensity;      // global motion intensity 0-100%
    float               axis_gain[6];   // per-axis gain multiplier (default 100%)

    void initDefaults();
    void rebuildPlatform();             // rebuild PlatformDef from geometry
    bool isStepper() const { return platform_type == PlatformType::Stepper; }
    bool isServo()   const { return platform_type == PlatformType::PWMServo; }
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

// ── Handshake State Machine ─────────────────────────────────────────

enum class HandshakePhase {
    Idle,               // not connected
    WaitFingerprint,    // sent FINGERPRINT?, awaiting response
    WaitConfig,         // sent CONFIG?, awaiting response
    WaitBits,           // sent BITS?, awaiting response
    Validating,         // all responses in — comparing params
    Ready,              // handshake passed — motion enabled
    Failed              // mismatch or timeout — motion blocked
};

// Parameters reported by the device during handshake
struct DeviceParams {
    // Identity (from FINGERPRINT response)
    char fingerprint[16];
    char fw_version[16];
    char platform_id[16];
    int  proto_ver;

    // Geometry (from CONFIG response)
    float RD, PD, L1, L2, height, theta_r, theta_p;
    int   servo_center[6];
    float pulse_per_rad;
    bool  config_received;

    // Input (from BITS response)
    int   bit_depth;
    float max_raw;
    bool  bits_received;

    void clear() {
        memset(this, 0, sizeof(*this));
    }
};

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
    bool            show_console;   // Per-entity I/O monitor window
    VizCamera       viz_cam;
    float           viz_split_ratio;    // 3D viewport height fraction (0.2–0.9, default 0.55)

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
    float           hil_fk_pose[6];      // FK-solved platform pose [tx,ty,tz,roll,pitch,yaw] in IK frame
    // Telemetry interpolation for smooth viz
    float           hil_tel_prev[6];     // previous telemetry angles
    float           hil_tel_curr[6];     // current telemetry angles
    double          hil_tel_prev_time;   // timestamp of previous telemetry
    double          hil_tel_curr_time;   // timestamp of current telemetry
    int             hil_tel_target_hz;   // requested telemetry rate from ESP32
    char            hil_port[32];        // selected COM port name
    bool            hil_auto_connect;    // try to reconnect if disconnected
    double          hil_last_reconnect;  // last reconnect attempt time
    HilProtocol     hil_protocol;        // Binary (big platform) or CSV (Mini-6DOF)
    uint16_t        hil_tx_raw[6];       // latest raw packet for background TX thread

    // Device fingerprint / handshake
    char            hil_fingerprint[16]; // stored MAC fingerprint (12 hex chars + NUL)
    char            hil_fw_version[16];  // firmware version string from device
    int             hil_proto_ver;       // protocol version from device
    bool            hil_handshake_ok;    // true after full handshake passed — motion enabled
    bool            hil_handshake_pending; // waiting for any handshake response

    // Multi-step handshake state
    HandshakePhase  hil_handshake_phase; // current phase of handshake sequence
    DeviceParams    hil_device_params;   // parameters reported by device during handshake
    double          hil_handshake_start; // frame_time when handshake began (for timeout)
    char            hil_handshake_msg[128]; // status message for UI display
    bool            hil_geo_synced;         // true if app geometry matches device after handshake
    std::vector<std::string> hil_cmd_queue; // queued serial commands (sent one per frame)

    // Time-series history ring buffers (for comparison charts)
    float           history_angles[6][HISTORY_LEN];   // servo angles over time (deg)
    float           history_input[6][HISTORY_LEN];     // scaled input over time
    float           history_time[HISTORY_LEN];         // timestamps (seconds)
    int             history_head;                       // write index
    int             history_count;                      // how many valid samples

    // Placeholder: frequency spectrum per axis (not wired yet)
    float           spectrum[6][SPECTRUM_BINS];         // magnitude bins
    float           spectrum_freq_max;                  // max frequency (Hz)

    // Dynamics apply S-curve transition (prevents jarring jumps when settings change mid-motion)
    float           last_scaled_pct[6];                 // last pipeline scaled_pct (updated each frame)
    bool            dyn_transition_active;              // true = crossfade in progress
    double          dyn_transition_start;               // frame_time when transition began
    float           dyn_transition_duration;            // seconds (default 1.5)
    float           dyn_transition_from[6];             // snapshot of scaled_pct before apply

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

enum class InputSource { Manual, SimToolsUDP, CapturePlayback, TestSignal, AssettoCorsa, Plugin };

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

// ── Assetto Corsa Shared Memory ─────────────────────────────────────
// Struct from decompiled SimTools AC plugin (SPageFilePhysics)
// AC exposes telemetry via memory-mapped files, not UDP.

#pragma pack(push, 4)
struct ACVec3 { float x, y, z; };

struct ACPhysics {
    int32_t   packetId;
    float     gas;
    float     brake;
    float     fuel;
    int32_t   gear;            // 0=R, 1=N, 2=1st ...
    int32_t   rpms;
    float     steerAngle;
    float     speedKmh;
    ACVec3    velocity;        // world velocity
    ACVec3    accG;            // G-forces: x=lateral, y=vertical, z=frontal
    float     wheelSlip[4];
    float     wheelLoad[4];
    float     wheelsPressure[4];
    float     wheelAngularSpeed[4];
    float     tyreWear[4];
    float     tyreDirtyLevel[4];
    float     tyreCoreTemperature[4];
    float     camberRAD[4];
    float     suspensionTravel[4];
    float     drs;
    float     tc;
    float     heading;         // world heading (rad)
    float     pitch;           // world pitch (rad)
    float     roll;            // world roll (rad)
    float     cgHeight;
    float     carDamage[5];
    int32_t   numberOfTyresOut;
    int32_t   pitLimiterOn;
    float     abs;
    float     kersCharge;
    float     kersInput;
    int32_t   autoShifterOn;
    float     rideHeight[2];
    float     turboBoost;
    float     ballast;
    float     airDensity;
    float     airTemp;
    float     roadTemp;
    ACVec3    localAngularVel;
    float     finalFF;
    float     performanceMeter;
    int32_t   engineBrake;
    int32_t   ersRecoveryLevel;
    int32_t   ersPowerLevel;
    int32_t   ersHeatCharging;
    int32_t   ersIsCharging;
    float     kersCurrentKJ;
    int32_t   drsAvailable;
    int32_t   drsEnabled;
    float     brakeTemp[4];
    float     clutch;
    float     tyreTempI[4];
    float     tyreTempM[4];
    float     tyreTempO[4];
    int32_t   isAIControlled;
    float     tyreContactPoint[12];   // 4 wheels × 3 (XYZ)
    float     tyreContactNormal[12];
    float     tyreContactHeading[12];
    float     brakeBias;
    ACVec3    localVelocity;
};
#pragma pack(pop)

// Available telemetry channels from AC shared memory
enum ACChannel {
    AC_CH_NONE = 0,          // disabled (output = 0)
    AC_CH_SURGE_G,           // accG.z (frontal G)
    AC_CH_SWAY_G,            // accG.x (lateral G)
    AC_CH_HEAVE_G,           // accG.y (vertical G, already gravity-compensated)
    AC_CH_ROLL,              // body roll (rad)
    AC_CH_PITCH,             // body pitch (rad)
    AC_CH_YAW_RATE,          // d(heading)/dt (rad/s) — computed, can be noisy
    AC_CH_LOCAL_VEL_X,       // localVelocity.x (lateral m/s)
    AC_CH_LOCAL_VEL_Z,       // localVelocity.z (longitudinal m/s)
    AC_CH_ANG_VEL_X,         // localAngularVel.x (roll rate rad/s)
    AC_CH_ANG_VEL_Y,         // localAngularVel.y (yaw rate rad/s, direct from physics)
    AC_CH_ANG_VEL_Z,         // localAngularVel.z (pitch rate rad/s)
    AC_CH_TRACTION_LOSS,     // max(wheelSlip[0..3])
    AC_CH_TRACTION_LOSS_AVG, // avg(wheelSlip[0..3])
    AC_CH_SUSP_TRAVEL_FL,    // suspensionTravel[0]
    AC_CH_SUSP_TRAVEL_FR,    // suspensionTravel[1]
    AC_CH_SUSP_TRAVEL_RL,    // suspensionTravel[2]
    AC_CH_SUSP_TRAVEL_RR,    // suspensionTravel[3]
    AC_CH_G_FORCE_LAT,       // same as SWAY_G but unsigned (abs)
    AC_CH_G_FORCE_LON,       // same as SURGE_G but unsigned (abs)
    AC_CH_COUNT
};

static const char* const AC_CHANNEL_NAMES[] = {
    "None (disabled)",
    "Surge G (accG.z)",
    "Sway G (accG.x)",
    "Heave G (accG.y)",
    "Roll (rad)",
    "Pitch (rad)",
    "Yaw Rate (heading delta)",
    "Local Vel X (lateral)",
    "Local Vel Z (longitudinal)",
    "Angular Vel X (roll rate)",
    "Angular Vel Y (yaw rate)",
    "Angular Vel Z (pitch rate)",
    "Traction Loss (max slip)",
    "Traction Loss (avg slip)",
    "Susp Travel FL",
    "Susp Travel FR",
    "Susp Travel RL",
    "Susp Travel RR",
    "G-Force Lateral (abs)",
    "G-Force Longitudinal (abs)",
};

// Per-axis channel mapping: which AC channel feeds each platform output axis
struct ACAxisMapping {
    int   channel;    // ACChannel enum value
    float min_val;    // raw value that maps to -100% (typically negative, e.g. -3.0)
    float max_val;    // raw value that maps to +100% (typically positive, e.g. 1.0)
    bool  invert;     // flip sign
};

struct ACState {
    std::atomic<bool>   running{false};
    std::thread         thread;
    void*               hMapFile = nullptr;   // HANDLE to memory-mapped file
    const ACPhysics*    mapped = nullptr;     // pointer to mapped view

    // Connection state
    std::atomic<bool>   connected{false};

    // Stats
    std::atomic<int>    packets_received{0};
    std::atomic<float>  rate_hz{0.0f};
    double              rate_window_start = 0.0;
    int                 rate_window_count = 0;
    int32_t             last_packet_id = -1;

    // Latest telemetry snapshot (for UI display, written by listener thread)
    std::atomic<float>  speed_kmh{0.0f};
    std::atomic<int>    rpm{0};
    std::atomic<int>    gear{0};

    // Raw channel values (written by listener, read by UI — use mutex)
    float               raw_channels[AC_CH_COUNT];
    double              last_packet_time;   // actual timestamp of last packet (for dt)
    float               yaw_rate_filtered;  // LP-filtered yaw rate
};

// ── Application State ───────────────────────────────────────────────

struct App {
    std::vector<Entity>   entities;
    int                   next_entity_id;

    // Shared input (from SimTools or manual override)
    float                 shared_input[6];
    std::mutex            input_mutex;       // protects shared_input when UDP writes
    InputSource           input_source;

    // Source-switch ramp-to-home state machine
    bool                  source_switch_active;     // true = ramp-out in progress
    InputSource           source_switch_target;     // source to switch to after ramp completes
    double                source_switch_start;      // frame_time when ramp started
    float                 source_switch_from[6];    // snapshot of input at ramp start
    static constexpr float SOURCE_RAMP_OUT_S = 1.0f;  // seconds to ramp to home
    void                  requestSourceSwitch(InputSource target);  // initiate ramp-to-home then switch

    // SimTools UDP config
    bool                  simtools_active;   // is UDP listener running?
    int                   simtools_port;
    int                   simtools_bit_depth;
    float                 simtools_rate;
    UdpListenerState      udp;

    // Assetto Corsa shared memory config
    bool                  ac_active;
    int                   ac_port;           // legacy, kept for settings compat
    ACAxisMapping         ac_axis_map[6];    // per-axis channel mapping
    ACState               ac;

    // Plugin system
    PluginManager         plugin_mgr;
    int                   active_plugin_idx;  // index into plugin_mgr.plugins(), -1 = none

    // Console
    std::vector<LogEntry> console_log;
    std::vector<LogEntry> console_log_frozen;   // snapshot when paused
    int                   console_max;
    bool                  console_auto_scroll;
    bool                  console_paused;        // freeze log display
    int                   console_filter;       // -1 = all, else entity_id

    // Global
    bool                  running;
    bool                  motion_started;       // true = motion output active, false = platform at home
    bool                  start_ramp_active;    // S-curve ramp from home to current input on START
    double                start_ramp_begin;     // frame_time when ramp started
    float                 start_ramp_target[6]; // target input_pct at ramp start
    static constexpr float START_RAMP_S = 0.6f; // ramp duration in seconds
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

    // Capture ramp state machine (prevents jarring starts/stops/loops)
    enum class CaptureRampPhase { RampIn, Playing, RampOut, HomeHold };
    CaptureRampPhase      capture_ramp_phase;
    double                capture_ramp_start;      // frame_time when current ramp phase started
    bool                  capture_stop_requested;  // true = ramp out then stop (don't loop)
    float                 capture_last_vals[6];    // last data values before ramp-out (for smooth blend)
    static constexpr float CAPTURE_RAMP_IN_S  = 2.0f;
    static constexpr float CAPTURE_RAMP_OUT_S = 2.0f;
    static constexpr float CAPTURE_HOME_HOLD_S = 1.0f;

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

    // MCA dynamics presets (user-saved, persisted to disk)
    struct McaDynamicsPreset {
        char                name[64];
        bool                is_builtin;       // true = from PRESET_TABLE, false = user-created
        MotionCueingConfig  mca;              // full MCA config snapshot
        float               intensity;        // global intensity
        float               axis_gain[6];     // per-axis gain trim
    };
    std::vector<McaDynamicsPreset> mca_presets;
    void    saveMcaPreset(const char* name, const MotionCueingConfig& mca, float intensity, const float axis_gain[6]);
    void    deleteMcaPreset(int idx);
    void    loadMcaPreset(int idx, MotionCueingConfig& mca, float& intensity, float axis_gain[6]);
    void    saveMcaPresetsToDisk();
    void    loadMcaPresetsFromDisk();

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
    void    handleHilLine(int entity_id, const char* line);  // parse ESP32 serial responses
    void    update();   // called every frame

    // UDP listener
    bool    startUdpListener();
    void    stopUdpListener();

    // Assetto Corsa UDP
    bool    startAssettoCorsaListener();
    void    stopAssettoCorsaListener();

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
