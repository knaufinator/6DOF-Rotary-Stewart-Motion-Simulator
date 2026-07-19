/*
 * Mini-6DOF Stewart Platform Controller - ESP-IDF Native Implementation
 *
 * PWM hobby-servo variant of the Stewart Platform controller.
 * Shares the same serial command API and protocol as the main controller
 * for full compatibility with the desktop HIL entity system.
 *
 * Key differences from the main (stepper) controller:
 *   - LEDC PWM output for hobby servos (50 Hz, 800-2200 µs)
 *   - No MCPWM, no step/dir GPIO, no planetary gearbox
 *   - Smaller platform geometry defaults
 *   - Servo angle → pulse width mapping instead of step counting
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// ESP-IDF headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include <fcntl.h>
#include <unistd.h>
#include "nvs_flash.h"
#include "esp_task_wdt.h"

// Project headers
#include "helpers.h"
#include "debug_uart.h"
#include "InverseKinematics.h"
#include "AxisScaling.h"
#include "MotionCueing.h"
#include "version.h"
#include "BleTransport.h"
#include "CobsTransport.h"

static const char* TAG __attribute__((unused)) = "mini6dof";

// ── Debug Output ─────────────────────────────────────────────────────
#ifdef ENABLE_DEBUG_UART
bool debugEnabled = false;
#endif

// ── Thread-safe serial printf (via COBS RESP channel) ───────────────
#define serial_printf(fmt, ...) cobs_send_fmt(COBS_CH_RESP, fmt, ##__VA_ARGS__)

// ── Platform Configuration ───────────────────────────────────────────
static StewartConfig stewartConfig;
static AxisScaleConfig axisScales;

// ── Motion Cueing (shared on-device cue engine) ──────────────────────
static MotionCueingConfig mcaConfig;
static InputFilterConfig  inputFilter;   // pre-MCA signal conditioning
#define MCA_SAMPLE_RATE 50.0f  // seed; retuned to servoRateHz once configured

// ── Servo-rate profile (runtime, NVS-persisted) ──────────────────────
// The mini ships with ANALOG servos (SG90/MG996R class) that physically
// latch once per PWM frame; driving them at 250 Hz would damage them, so the
// default carrier is 50 Hz. A digital-servo upgrade (DS3218) accepts 250 Hz,
// unlocking the high-fidelity path. ONE knob sets BOTH the LEDC carrier AND
// the CueTask loop rate (they must match); switch with SERVO:RATE / SERVO:MODE
// at runtime — no reflash. cueLoopHz == servoRateHz.
#define SERVO_RATE_ANALOG_HZ   50
#define SERVO_RATE_DIGITAL_HZ  250
#define SERVO_RATE_MIN_HZ      20
#define SERVO_RATE_MAX_HZ      333
static volatile uint16_t servoRateHz    = SERVO_RATE_ANALOG_HZ;   // default = analog/safe
static volatile float    servoPeriodUs  = 1000000.0f / SERVO_RATE_ANALOG_HZ;

// ── On-device cue engine: shared latest-sample target (hold-only) ─────
// Ingest paths (CH_DATA, CH_DATA_RAW, PlaybackTask, BLE) STOP driving IK/servo
// and just write the freshest sample here + an esp_timer receive-stamp. The
// fixed-rate CueTask is the SOLE servo writer: it reads this, runs the cue
// chain (for RAW), maps to position, per-time slews, IK, and writes servos.
// (MCU_HIFI_CUEING.md "DECISIONS APPLIED" hold-only variant.)
typedef enum {
    TGT_BAKED = 0,   // raw uint16 counts, cue already baked -> mapRawToPosition only
    TGT_RAW   = 1,   // pre-cue telemetry -> inputFilter -> MCA -> outputStage -> mapRawToPosition
    TGT_PHYS  = 2,   // already physical mm/rad (BLE accel) -> straight to slew/IK
} TargetFmt;
static portMUX_TYPE g_targetMux = portMUX_INITIALIZER_UNLOCKED;
static float   g_targetCh[6]  = {0, 0, 0, 0, 0, 0};
static int64_t g_targetTsUs   = 0;
static int     g_targetFmt    = TGT_BAKED;

// Stale/lost ladder (hold-only): beyond LOST, CueTask decays to home so the
// platform never parks at a stale tilt. HOLD (< LOST) keeps feeding the last
// sample; the washout HP naturally returns RAW motion toward center.
#define CUE_LOST_DECAY_MS  300

// Mini-6DOF specific defaults (geometry in mm, converted from inches)
static void initMiniDefaults(StewartConfig* cfg) {
    cfg->theta_r = 10.0f;
    cfg->theta_s[0] = 150.0f; cfg->theta_s[1] = -90.0f; cfg->theta_s[2] = 30.0f;
    cfg->theta_s[3] = 150.0f; cfg->theta_s[4] = -90.0f; cfg->theta_s[5] = 30.0f;
    cfg->theta_p = 30.0f;
    cfg->RD = 15.75f;               // base radius (mm — original Mini uses mm directly)
    cfg->PD = 16.0f;                // platform radius (mm)
    cfg->ServoArmLengthL1 = 7.25f;  // servo horn length (mm)
    cfg->ConnectingArmLengthL2 = 28.5f; // connecting rod length (mm)
    cfg->platformHeight = 25.517f;   // neutral height (mm)

    // Drive train not used for PWM servos — kept for API compat
    cfg->virtual_gear = 1.0f;
    cfg->planetary_ratio = 1.0f;
    cfg->encoder_ppr = 1;
    cfg->steps_per_degree = 1.0f;
}

// ── Input Scaling ────────────────────────────────────────────────────
static uint8_t inputBitRange = 12;
static float maxRawInput = 4095.0f;

// ── Motion State ─────────────────────────────────────────────────────
static volatile float arr[6] = {0, 0, 0, 0, 0, 0};  // current IK position (physical units)
static volatile float lastServoAngles[6] = {0};         // latest IK output (radians) for telemetry
static SemaphoreHandle_t xMutex = NULL;

// ── Telemetry Rate ──────────────────────────────────────────────────
static volatile int telemetryDelayMs = 20;  // default 50Hz (was 100ms = 10Hz)
static volatile bool telemetryEnabled = false;  // silent until app sends TELRATE:N after handshake

// ── Activity Watchdog ────────────────────────────────────────────────
static volatile int64_t lastPacketTimeUs = 0;            // esp_timer_get_time() of last motion packet
#define WATCHDOG_TIMEOUT_US  500000                      // 500ms — home if no input
static volatile bool watchdogTripped = false;

// ── Input Mode ──────────────────────────────────────────────────────
typedef enum {
    INPUT_SERIAL = 0,       // serial binary/CSV motion commands
    INPUT_BLE_MOTION = 1,   // BLE binary motion commands (Android app)
    INPUT_BLE_ACCEL = 2,    // BLE raw accel/gyro from phone sensor
    INPUT_PLAYBACK = 3,     // embedded motion-cued sequence playback (no host)
} InputMode;
static volatile InputMode inputMode = INPUT_SERIAL;

// ── Motion Source (OFF / DEMO / LIVE) ────────────────────────────────
// The "easy button" selector (HIL_BRIDGE.md). OFF homes + gates all motion
// (one-tap kill); DEMO plays the embedded sequence; LIVE accepts streamed
// motion (CH_DATA baked or CH_DATA_RAW pre-cue). Persists a boot default in
// NVS (generalizes the old play_on_boot). Default boot source = DEMO.
typedef enum {
    SRC_OFF  = 0,   // motion off: home + ignore incoming motion (safe idle)
    SRC_DEMO = 1,   // on-device embedded playback (PlaybackTask)
    SRC_LIVE = 2,   // accept streamed motion (CH_DATA / CH_DATA_RAW)
} Source;
static volatile Source g_source = SRC_DEMO;

// ── BLE Accel Input ──────────────────────────────────────────────────
// Raw sensor data from phone: [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
// Accel in m/s² (Android TYPE_ACCELEROMETER, includes gravity)
// Gyro in rad/s  (Android TYPE_GYROSCOPE)
static volatile float accelRaw[6] = {0};
static volatile bool accelFresh = false;
static volatile uint32_t accelPacketCount = 0;

// Per-axis gain: maps sensor units to platform physical units
// Translation axes: mm per G   (e.g. 5.0 = 1G → 5mm displacement)
// Rotation axes:    rad per rad/s (e.g. 0.15 = 1rad/s → 0.15rad)
static float accelGain[6] = {
    1.0f,   // surge (mm per m/s^2 from phone)
    1.0f,   // sway  (mm per m/s^2 from phone)
    1.0f,   // heave (mm per m/s^2 from phone)
    1.0f,   // roll  (1.0 = 1:1 phone-to-platform degrees)
    1.0f,   // pitch (1.0 = 1:1 phone-to-platform degrees)
    1.0f,   // yaw   (1.0 = 1:1 phone-to-platform degrees)
};

// Axis mapping: which phone sensor index maps to each platform axis
// Phone flat, screen up, top edge forward:
//   phone X = right, Y = forward, Z = up (matches TYPE_ACCELEROMETER)
// Positive value = same polarity, negative = inverted
// abs(value)-1 = source index (1-based to allow sign encoding, 0 = disabled)
static int8_t accelAxisMap[6] = {
     2,  // surge  ← phone Y (forward)
     1,  // sway   ← phone X (right)
     3,  // heave  ← phone Z (up)
     4,  // roll   ← gyro X
     5,  // pitch  ← gyro Y
     6,  // yaw    ← gyro Z
};

#define GRAVITY_MS2 9.80665f

// ── Slew-Rate Limiter (per-TIME, rate-independent) ───────────────────
// FIX TRAP B (MCU_HIFI_CUEING.md §2): the old limit was per-CALL (5 units/
// cycle) which silently assumed 50 Hz. At 250 Hz that would be ~5x harsher.
// Express it as units/SECOND and multiply by dt so it's identical in feel at
// any cueLoopHz. 250 units/s == the old 5 units × 50 Hz.
#define SLEW_RATE_MAX_PER_S   250.0f                     // mm (or rad) per SECOND
static float smoothedPosition[6] = {0};                  // current smoothed output
static bool smoothingInitialized = false;

// ── IK Angle Limits ──────────────────────────────────────────────────
// Max servo arm deflection in radians (±45° is typical hobby servo range)
#define SERVO_MAX_ANGLE_RAD  (IK_PI / 4.0f)

// ── Servo PWM ────────────────────────────────────────────────────────
static const int servoPins[6] = {
    SERVO_PIN_0, SERVO_PIN_1, SERVO_PIN_2,
    SERVO_PIN_3, SERVO_PIN_4, SERVO_PIN_5
};

// Servo center calibration (µs) — can be adjusted per-servo
static int servoCenter[6] = {1500, 1500, 1500, 1500, 1500, 1500};

// Pulse width multiplier: µs per radian of servo arm rotation
static float servoPulsePerRad = 800.0f / (float)(IK_PI / 4.0);

// Inverted servos (mounted mirrored)
static const bool servoInverted[6] = {true, false, true, false, true, false};

// ── NVS Persistence ─────────────────────────────────────────────────
static const char* NVS_NAMESPACE = "mini6dof";

static void saveConfigToNVS() {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, "servo_center", servoCenter, sizeof(servoCenter));
        nvs_set_blob(h, "pulse_per_rad", &servoPulsePerRad, sizeof(servoPulsePerRad));
        nvs_set_blob(h, "geometry", &stewartConfig, sizeof(stewartConfig));
        nvs_set_u8(h, "bit_depth", inputBitRange);
        uint16_t sr = servoRateHz;
        nvs_set_blob(h, "servo_rate", &sr, sizeof(sr));
        nvs_commit(h);
        nvs_close(h);
    }
}

static void loadConfigFromNVS() {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        size_t sz;
        sz = sizeof(servoCenter);
        nvs_get_blob(h, "servo_center", servoCenter, &sz);
        sz = sizeof(servoPulsePerRad);
        nvs_get_blob(h, "pulse_per_rad", &servoPulsePerRad, &sz);
        sz = sizeof(stewartConfig);
        if (nvs_get_blob(h, "geometry", &stewartConfig, &sz) == ESP_OK) {
            // Geometry loaded from NVS — recompute scales after load
        }
        uint8_t bits = 0;
        if (nvs_get_u8(h, "bit_depth", &bits) == ESP_OK && bits >= 8 && bits <= 16) {
            inputBitRange = bits;
            maxRawInput = (float)((1 << bits) - 1);
        }
        uint16_t sr = 0; sz = sizeof(sr);
        if (nvs_get_blob(h, "servo_rate", &sr, &sz) == ESP_OK &&
            sr >= SERVO_RATE_MIN_HZ && sr <= SERVO_RATE_MAX_HZ) {
            servoRateHz   = sr;
            servoPeriodUs = 1000000.0f / (float)sr;
        }
        nvs_close(h);
    }
}

// ── Boot source (NVS) ────────────────────────────────────────────────
// Generalizes the old play_on_boot flag into the OFF/DEMO/LIVE source model.
// Default = DEMO: the rig is a headless demo that "powers on and autonomously
// plays the lap" (DECISIONS r3). SOURCE:BOOT=OFF|DEMO|LIVE sets it; PLAY:BOOT=0
// (OFF) / PLAY:BOOT=1 (DEMO) stay as aliases. Reads the legacy play_on_boot key
// as a fallback so an already-flashed device keeps its setting.
static Source loadBootSource() {
    nvs_handle_t h;
    Source src = SRC_DEMO;   // default
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        uint8_t v;
        if (nvs_get_u8(h, "boot_source", &v) == ESP_OK) {
            src = (v <= SRC_LIVE) ? (Source)v : SRC_DEMO;
        } else if (nvs_get_u8(h, "play_on_boot", &v) == ESP_OK) {
            src = v ? SRC_DEMO : SRC_OFF;    // legacy fallback
        }
        nvs_close(h);
    }
    return src;
}

static void saveBootSource(Source src) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "boot_source", (uint8_t)src);
        nvs_commit(h);
        nvs_close(h);
    }
}

static const char* sourceName(Source s) {
    switch (s) { case SRC_OFF: return "OFF"; case SRC_DEMO: return "DEMO";
                 case SRC_LIVE: return "LIVE"; default: return "?"; }
}

// ── LEDC PWM Setup ──────────────────────────────────────────────────

// LEDC timer resolution: 16-bit gives good precision at 50Hz
// 50Hz = 20ms period. At 16-bit (65536 ticks): 1 tick ≈ 0.305 µs
#define LEDC_TIMER_BITS   LEDC_TIMER_16_BIT
#define LEDC_TIMER_MAX    65535

static void setupServoPWM() {
    // Configure LEDC timer at the current servo-rate profile (default 50 Hz).
    // 16-bit resolution is valid for both 50 Hz and 250 Hz (80MHz/2^16 ≈ 1.2kHz max).
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.timer_num = LEDC_TIMER_0;
    timer_conf.duty_resolution = LEDC_TIMER_BITS;
    timer_conf.freq_hz = servoRateHz;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_conf);

    // Configure each servo channel
    for (int i = 0; i < 6; i++) {
        ledc_channel_config_t ch_conf = {};
        ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
        ch_conf.channel = (ledc_channel_t)i;
        ch_conf.timer_sel = LEDC_TIMER_0;
        ch_conf.intr_type = LEDC_INTR_DISABLE;
        ch_conf.gpio_num = servoPins[i];
        ch_conf.duty = 0;
        ch_conf.hpoint = 0;
        ledc_channel_config(&ch_conf);
    }
}

// Convert microseconds to LEDC duty value.
// duty = (us / period_us) * max_duty, where period_us tracks the servo rate
// (20000us @ 50Hz, 4000us @ 250Hz) so pulse widths stay absolute microseconds.
static uint32_t usToDuty(int us) {
    return (uint32_t)((float)us / servoPeriodUs * (float)LEDC_TIMER_MAX);
}

// Set servo pulse width in microseconds
static void setServoPulse(int channel, int us) {
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, usToDuty(us));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
}

// ── Slew-Rate Limited Position Update ────────────────────────────────
// Applies per-axis slew-rate limiting to prevent servo jerk from large steps.
// Returns the smoothed position in `out[6]`.

static void slewRateLimit(const float target[6], float out[6], float dt) {
    if (!smoothingInitialized) {
        memcpy(smoothedPosition, target, sizeof(smoothedPosition));
        smoothingInitialized = true;
    }
    const float maxStep = SLEW_RATE_MAX_PER_S * dt;   // per-time -> per-call
    for (int i = 0; i < 6; i++) {
        float delta = target[i] - smoothedPosition[i];
        if (delta > maxStep) delta = maxStep;
        else if (delta < -maxStep) delta = -maxStep;
        smoothedPosition[i] += delta;
        out[i] = smoothedPosition[i];
    }
}

// ── Drive Servos (slew + IK + LEDC write) ────────────────────────────
// The low-level actuator stage. Called ONLY from CueTask (the sole servo
// writer) once tasks are running, plus directly at boot before CueTask starts.
//   1. Per-time slew-rate limiting prevents servo jerk from large steps
//   2. IK output validated (NaN / out-of-range clamped)
//   3. Atomic servo update: all 6 duties set first, then all 6 updated
// `dt` is the loop period in seconds (1/cueLoopHz) for the per-time slew.

static void driveServos(float position[6], float dt) {
    // Slew-rate limit the input position (rate-independent)
    float limited[6];
    slewRateLimit(position, limited, dt);

    // Run inverse kinematics
    float angles[6];
    calculateAllServoAngles(limited, &stewartConfig, angles);

    // Validate IK output — clamp NaN and out-of-range angles
    for (int i = 0; i < 6; i++) {
        if (isnan(angles[i]) || isinf(angles[i])) {
            angles[i] = 0.0f;  // safe fallback
        } else if (angles[i] > SERVO_MAX_ANGLE_RAD) {
            angles[i] = SERVO_MAX_ANGLE_RAD;
        } else if (angles[i] < -SERVO_MAX_ANGLE_RAD) {
            angles[i] = -SERVO_MAX_ANGLE_RAD;
        }
    }

    memcpy((void*)lastServoAngles, angles, sizeof(lastServoAngles));

    // Pre-compute all 6 pulse widths
    int pulse[6];
    for (int i = 0; i < 6; i++) {
        if (servoInverted[i]) {
            pulse[i] = servoCenter[i] + (int)(angles[i] * servoPulsePerRad);
        } else {
            pulse[i] = servoCenter[i] - (int)(angles[i] * servoPulsePerRad);
        }
        if (pulse[i] < SERVO_MIN_US) pulse[i] = SERVO_MIN_US;
        if (pulse[i] > SERVO_MAX_US) pulse[i] = SERVO_MAX_US;
    }

    // Atomic batch update: set all duties first, then trigger all updates
    for (int i = 0; i < 6; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, usToDuty(pulse[i]));
    }
    for (int i = 0; i < 6; i++) {
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
    }
}

// ── Shared latest-sample handoff (producers -> CueTask) ──────────────
// Ingest paths call these instead of driving servos. Zero-order hold: the
// CueTask always reads the freshest sample under a short critical section.
static void writeTarget(const float ch[6], int fmt) {
    int64_t t = esp_timer_get_time();
    taskENTER_CRITICAL(&g_targetMux);
    for (int i = 0; i < 6; i++) g_targetCh[i] = ch[i];
    g_targetTsUs = t;
    g_targetFmt  = fmt;
    taskEXIT_CRITICAL(&g_targetMux);
    lastPacketTimeUs = t;
    watchdogTripped  = false;
}
static void readTarget(float ch[6], int64_t* ts, int* fmt) {
    taskENTER_CRITICAL(&g_targetMux);
    for (int i = 0; i < 6; i++) ch[i] = g_targetCh[i];
    *ts  = g_targetTsUs;
    *fmt = g_targetFmt;
    taskEXIT_CRITICAL(&g_targetMux);
}

// ── Servo-rate profile applier ───────────────────────────────────────
// Sets BOTH the LEDC carrier and the CueTask loop rate (cueLoopHz) together,
// and retunes the biquads (FIX TRAP A) so filters match the new loop rate.
// Safe to call at runtime; CueTask picks up the new period on its next tick.
static void applyServoRate(uint16_t hz) {
    if (hz < SERVO_RATE_MIN_HZ) hz = SERVO_RATE_MIN_HZ;
    if (hz > SERVO_RATE_MAX_HZ) hz = SERVO_RATE_MAX_HZ;
    servoRateHz   = hz;
    servoPeriodUs = 1000000.0f / (float)hz;

    // Re-init the LEDC timer to the new carrier frequency.
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode     = LEDC_LOW_SPEED_MODE;
    timer_conf.timer_num      = LEDC_TIMER_0;
    timer_conf.duty_resolution= LEDC_TIMER_BITS;
    timer_conf.freq_hz        = hz;
    timer_conf.clk_cfg        = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_conf);

    // FIX TRAP A: biquads are rate-dependent (ω = 2π·fc/fs) — retune to the
    // loop rate (== servoRateHz), NOT the telemetry/seq rate.
    mcaUpdateSampleRate(&mcaConfig, (float)hz);
    inputFilterUpdateSampleRate(&inputFilter, (float)hz);
}

// ── BLE Accel Callback ───────────────────────────────────────────────
// Called from BLE transport when 24-byte accel packet arrives on 0xFF03.
// Stores raw data and sets fresh flag for main loop processing.

void process_accel_packet(const float* data) {
    for (int i = 0; i < 6; i++) accelRaw[i] = data[i];
    accelFresh = true;
    accelPacketCount++;
    lastPacketTimeUs = esp_timer_get_time();
    watchdogTripped = false;
    inputMode = INPUT_BLE_ACCEL;
}

// ── Forward Declarations ─────────────────────────────────────────────
void process_data(char* data);
void process_binary_packet(const uint8_t* payload);
void process_raw_packet(const uint8_t* payload);
static void setSource(Source s);

// ── Binary Packet Protocol ───────────────────────────────────────────
// Same 15-byte framed packet as main controller:
// [0xAA][0x55][ch0_lo][ch0_hi][ch1_lo]...[ch5_hi][xor_checksum]  (little-endian)

// Baked 6×uint16 frame (CH_DATA / M6P1 playback). Producer only: decode and
// hand the latest sample to CueTask as TGT_BAKED (cue already applied on the
// desktop, so CueTask runs mapRawToPosition only — no double-cueing).
void process_binary_packet(const uint8_t* payload) {
    float raw[6];
    for (int i = 0; i < 6; i++) {
        uint16_t val = (uint16_t)payload[i * 2] | ((uint16_t)payload[i * 2 + 1] << 8);
        raw[i] = (float)val;
    }
    writeTarget(raw, TGT_BAKED);
}

// RAW pre-cue frame: 6×float32 LE = 24 bytes (CH_DATA_RAW / M6P2 playback).
// Producer only: CueTask runs the full cue chain (inputFilter -> MCA ->
// outputStage -> mapRawToPosition) at cueLoopHz.
void process_raw_packet(const uint8_t* payload) {
    float raw[6];
    memcpy(raw, payload, 6 * sizeof(float));   // LE float32, wire order
    writeTarget(raw, TGT_RAW);
}

// ── Embedded Sequence Playback ───────────────────────────────────────
// A motion-cued lap baked offline by the desktop app's export_sequence (see
// cobra-6dof-mount). File .m6p: 64-byte header {"M6P1", u16 ver, u16 rate,
// u32 count, u32 loop_point, char name[32], u16 bits @48, u32 crc32 @52} then
// count × 6×uint16 LE — the exact 12-byte payload process_binary_packet()
// consumes (wire order, surge/sway already swapped). Playing == streaming that
// payload at `rate`, so it reuses the whole decode→IK→servo path and, because
// process_binary_packet() feeds the activity watchdog, playback keeps the rig
// live with no extra plumbing.
extern const uint8_t _seq_start[] asm("_binary_laps123_moderate_m6p_start");
extern const uint8_t _seq_end[]   asm("_binary_laps123_moderate_m6p_end");

static const uint8_t* seqSamples   = nullptr;  // -> first sample
static uint32_t        seqCount     = 0;
static uint32_t        seqLoopPoint = 0;
static uint16_t        seqRateHz    = 50;
static uint16_t        seqBits      = 12;
static uint16_t        seqStride    = 12;      // bytes/frame: 12 (M6P1) or 24 (M6P2)
static bool            g_framesRaw  = false;   // true = M6P2 float32 pre-cue frames
static volatile bool     playbackActive = false;
static volatile bool     playbackLoop   = true;
static volatile uint32_t playbackIdx    = 0;

// .m6p header (64 bytes), branch on magic (app Phase-2 contract, PR #29):
//   common: magic[4]@0, u16 ver@4, u16 rate@6, u32 count@8, u32 loop@12,
//           name[32]@16..47, u32 crc32@52.
//   M6P1 (baked): u16 bits@48; frames = 6×uint16 LE (12B, post-cue, wire order
//                 = device order, surge/sway already swapped). g_framesRaw=false.
//   M6P2 (raw):   u8 format@48 (1=float32), u8 channels@49 (=6), [50..51] rsvd,
//                 [56..63] rsvd; frames = 6×float32 LE (24B), PRE-cueing, app
//                 axis order (surge=0, sway=1), NO surge/sway swap on the wire
//                 (CueTask swaps after cueing). g_framesRaw=true.
static bool parseSequence() {
    const uint8_t* h = _seq_start;
    size_t total = (size_t)(_seq_end - _seq_start);
    if (total < 64) return false;

    bool isM6P1 = (memcmp(h, "M6P1", 4) == 0);
    bool isM6P2 = (memcmp(h, "M6P2", 4) == 0);
    if (!isM6P1 && !isM6P2) return false;

    memcpy(&seqRateHz,    h + 6,  2);
    memcpy(&seqCount,     h + 8,  4);
    memcpy(&seqLoopPoint, h + 12, 4);

    if (isM6P2) {
        uint8_t format   = h[48];         // 1 = float32
        uint8_t channels = h[49];         // must be 6
        if (format != 1 || channels != 6) return false;
        g_framesRaw = true;
        seqStride   = 24;                 // 6 × float32
        seqBits     = 16;                 // raw float32 has no bit depth (placeholder)
    } else {
        memcpy(&seqBits, h + 48, 2);      // M6P1: u16 bit depth @48
        g_framesRaw = false;
        seqStride   = 12;                 // 6 × uint16 (baked)
    }

    if (seqRateHz == 0 || seqCount == 0) return false;
    if ((size_t)64 + (size_t)seqCount * seqStride > total) return false;   // truncated
    if (seqLoopPoint >= seqCount) seqLoopPoint = 0;
    seqSamples = h + 64;
    return true;
}

// Decode one embedded frame (M6P1 uint16 or M6P2 float32) into raw[6].
static inline void decodeSeqFrame(const uint8_t* p, float raw[6]) {
    if (g_framesRaw) {
        memcpy(raw, p, 6 * sizeof(float));
    } else {
        for (int i = 0; i < 6; i++) {
            uint16_t v = (uint16_t)p[i * 2] | ((uint16_t)p[i * 2 + 1] << 8);
            raw[i] = (float)v;
        }
    }
}

// PlaybackTask is now a PRODUCER: it paces to the file rate and pushes each
// sample into the shared target (raw M6P2 -> TGT_RAW so CueTask cues it;
// baked M6P1 -> TGT_BAKED). CueTask does the IK/servo at cueLoopHz.
static void PlaybackTask(void* pv) {
    (void)pv;
    uint16_t rate = (seqRateHz ? seqRateHz : 50);
    TickType_t period = pdMS_TO_TICKS(1000 / rate);
    if (period < 1) period = 1;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        if (playbackActive && seqSamples && seqCount) {
            float raw[6];
            decodeSeqFrame(seqSamples + (size_t)playbackIdx * seqStride, raw);
            writeTarget(raw, g_framesRaw ? TGT_RAW : TGT_BAKED);
            uint32_t nxt = playbackIdx + 1;
            if (nxt >= seqCount) {
                if (playbackLoop) {
                    nxt = seqLoopPoint;
                } else {
                    playbackActive = false;
                    nxt = 0;
                    serial_printf("PLAY:DONE\r\n");
                }
            }
            playbackIdx = nxt;
        }
        vTaskDelayUntil(&last, period);
    }
}

// ── CueTask — fixed-rate consumer (SOLE servo writer) ────────────────
// MCU_HIFI_CUEING.md "DECISIONS APPLIED" hold-only variant: reads the freshest
// sample (zero-order hold), runs the cue chain for RAW frames, maps to
// position, per-time slews, IK, writes servos — all at cueLoopHz (== servoRateHz).
static void CueTask(void* pv) {
    (void)pv;
    // Retune biquads to the loop rate up front (FIX TRAP A).
    applyServoRate(servoRateHz);
    uint16_t curRate = servoRateHz;
    TickType_t period = pdMS_TO_TICKS(1000 / curRate);
    if (period < 1) period = 1;
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        // Pick up a runtime servo-rate change (SERVO:RATE / SERVO:MODE).
        if (servoRateHz != curRate) {
            curRate = servoRateHz;
            period  = pdMS_TO_TICKS(1000 / curRate);
            if (period < 1) period = 1;
        }
        const float dt = 1.0f / (float)curRate;

        float ch[6]; int64_t ts; int fmt;
        readTarget(ch, &ts, &fmt);
        int64_t age = esp_timer_get_time() - ts;

        float pos[6];
        if (g_source == SRC_OFF) {
            // Gated: home and ignore incoming motion (one-tap kill).
            for (int i = 0; i < 6; i++) pos[i] = 0.0f;
        } else if (age > (int64_t)CUE_LOST_DECAY_MS * 1000) {
            // Lost: decay toward home so we never park at a stale tilt.
            for (int i = 0; i < 6; i++) pos[i] = 0.0f;
        } else if (fmt == TGT_RAW) {
            // RAW = pre-cue telemetry in APP axis order (surge=0, sway=1), no
            // wire swap. Run the cue chain in app order (the washout/tilt engine
            // is defined in app order), THEN swap surge<->sway into device order
            // before scaling/IK — the swap the baked wire already carried.
            float f[6], m[6], o[6];
            processInputFilter(&inputFilter, ch, f);
            processMotionCueing(&mcaConfig, f, m);
            mcaApplyOutputStage(&mcaConfig, m, o);
            float tmp = o[0]; o[0] = o[1]; o[1] = tmp;   // surge<->sway (app->device)
            // RAW = signed PERCENT (pre-cue telemetry). mapRawToPosition expects the
            // COUNT domain [0..max_raw] centered at home, so convert first per the
            // pinned contract: counts = home*(1 + pct/100)  =>  ±100% spans 0..max_raw
            // and the mapping yields position = scale*(pct/100). Feeding raw percent
            // straight in (the previous code) sits ~0 counts << home and rails the
            // output — decoupled from input. Bug found on the first live stream.
            float home = (float)((int)maxRawInput / 2);
            float counts[6];
            for (int i = 0; i < 6; i++) counts[i] = home * (1.0f + o[i] * 0.01f);
            mapRawToPosition(counts, &axisScales, maxRawInput, pos);
        } else if (fmt == TGT_PHYS) {
            for (int i = 0; i < 6; i++) pos[i] = ch[i];
        } else { // TGT_BAKED
            mapRawToPosition(ch, &axisScales, maxRawInput, pos);
        }

        for (int i = 0; i < 6; i++) arr[i] = pos[i];   // telemetry snapshot
        driveServos(pos, dt);
        vTaskDelayUntil(&last, period);
    }
}

// ── Source transitions ───────────────────────────────────────────────
// Reset filter state on EVERY transition (DECISIONS r4 / task requirement) so
// stale washout doesn't bleed across a mode change.
static void setSource(Source s) {
    resetMotionCueing(&mcaConfig);
    resetInputFilter(&inputFilter);

    switch (s) {
        case SRC_OFF: {
            playbackActive = false;
            inputMode = INPUT_SERIAL;
            float home[6] = {0, 0, 0, 0, 0, 0};
            writeTarget(home, TGT_PHYS);      // CueTask homes + gates
            break;
        }
        case SRC_DEMO: {
            if (seqSamples && seqCount) {
                if (!g_framesRaw) {                 // baked M6P1 uses its bit depth
                    inputBitRange = (uint8_t)seqBits;
                    maxRawInput   = (float)((1 << seqBits) - 1);
                }
                playbackIdx    = 0;
                playbackActive = true;
                inputMode      = INPUT_PLAYBACK;
            }
            break;
        }
        case SRC_LIVE: {
            playbackActive = false;
            inputMode = INPUT_SERIAL;         // await CH_DATA / CH_DATA_RAW
            break;
        }
    }
    g_source = s;
}

// Gate for incoming live motion (CH_DATA / CH_DATA_RAW / CSV). Returns false
// when SOURCE:OFF (motion killed -> ignore). Otherwise auto-switches DEMO->LIVE
// on first live packet (preserves the old "live input takes over playback"
// behavior; resets filters exactly once on the transition).
static inline bool liveMotionGate() {
    if (g_source == SRC_OFF) return false;
    if (g_source != SRC_LIVE) setSource(SRC_LIVE);
    return true;
}

// ── Legacy CSV Parser + Command Handler ──────────────────────────────

void process_data(char* data) {
    // ── DBG:1 / DBG:0 — Toggle verbose debug output ─────────────────
    if (strcmp(data, DEBUG_ENABLE_CMD) == 0) {
        debugEnabled = true;
        serial_printf("Debug output enabled\r\n");
        return;
    } else if (strcmp(data, DEBUG_DISABLE_CMD) == 0) {
        debugEnabled = false;
        serial_printf("Debug output disabled\r\n");
        return;
    }

    // ── HIGH-PRIORITY: Handshake commands ──────────────────────────────
    // These must be at the top so the app's handshake completes instantly.
    // Any delay here blocks motion — the app won't send packets until
    // the handshake reaches Ready.

    if (strcmp(data, "FINGERPRINT?") == 0) {
        uint8_t mac[6];
        esp_efuse_mac_get_default(mac);
        // caps=raw advertises on-device RAW-HIL cueing (CH_DATA_RAW + M6P2) so
        // the app enables raw mode (it gates on caps=...raw... in FINGERPRINT).
        serial_printf("FINGERPRINT:%02X%02X%02X%02X%02X%02X,fw=%s,proto=%d,platform=%s,caps=raw\r\n",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
            FW_VERSION_STRING, FW_PROTOCOL_VERSION, FW_PLATFORM_ID);
        return;
    }

    if (strcmp(data, "CONFIG?") == 0) {
        serial_printf("CONFIG:RD=%.2f,PD=%.2f,L1=%.2f,L2=%.2f,height=%.2f,theta_r=%.2f,theta_p=%.2f\r\n",
            stewartConfig.RD, stewartConfig.PD,
            stewartConfig.ServoArmLengthL1, stewartConfig.ConnectingArmLengthL2,
            stewartConfig.platformHeight, stewartConfig.theta_r, stewartConfig.theta_p);
        serial_printf("SERVO:center=%d,%d,%d,%d,%d,%d,pulse_per_rad=%.1f\r\n",
            servoCenter[0], servoCenter[1], servoCenter[2],
            servoCenter[3], servoCenter[4], servoCenter[5], servoPulsePerRad);
        return;
    }

    if (strcmp(data, "BITS?") == 0) {
        serial_printf("BITS:%d,max_raw=%.0f\r\n", inputBitRange, maxRawInput);
        return;
    }

    if (strcmp(data, "VERSION?") == 0) {
        serial_printf("VERSION:%s,proto=%d,platform=%s,date=%s,time=%s\r\n",
            FW_VERSION_STRING, FW_PROTOCOL_VERSION, FW_PLATFORM_ID,
            FW_BUILD_DATE, FW_BUILD_TIME);
        return;
    }

    // ── SCALE? — Query current axis scaling factors ──────────────────
    if (strcmp(data, "SCALE?") == 0) {
        serial_printf("SCALE:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
            axisScales.scale[0], axisScales.scale[1], axisScales.scale[2],
            axisScales.scale[3], axisScales.scale[4], axisScales.scale[5]);
        return;
    }

    // ── BITS:N — Set input bit depth ─────────────────────────────────
    if (strncmp(data, "BITS:", 5) == 0) {
        int bits = atoi(data + 5);
        if (bits >= 8 && bits <= 16) {
            inputBitRange = (uint8_t)bits;
            maxRawInput = (float)((1 << bits) - 1);
            serial_printf("BITS:%d,max_raw=%.0f\r\n", inputBitRange, maxRawInput);
            saveConfigToNVS();
        } else {
            serial_printf("ERR:BITS range 8-16\r\n");
        }
        return;
    }

    // ── CONFIG:key=value — Set platform geometry parameter ───────────
    if (strncmp(data, "CONFIG:", 7) == 0) {
        char* param = data + 7;
        char* eq = strchr(param, '=');
        if (eq) {
            *eq = '\0';
            float val = atof(eq + 1);
            bool changed = true;
            if (strcmp(param, "RD") == 0) stewartConfig.RD = val;
            else if (strcmp(param, "PD") == 0) stewartConfig.PD = val;
            else if (strcmp(param, "L1") == 0) stewartConfig.ServoArmLengthL1 = val;
            else if (strcmp(param, "L2") == 0) stewartConfig.ConnectingArmLengthL2 = val;
            else if (strcmp(param, "height") == 0) stewartConfig.platformHeight = val;
            else if (strcmp(param, "theta_r") == 0) stewartConfig.theta_r = val;
            else if (strcmp(param, "theta_p") == 0) stewartConfig.theta_p = val;
            else { changed = false; serial_printf("CONFIG:ERR unknown key '%s'\r\n", param); }
            if (changed) {
                computeAxisScalesFromGeometry(&axisScales, &stewartConfig, 0.90f);
                serial_printf("CONFIG:OK %s=%.4f (scales recomputed)\r\n", param, val);
                saveConfigToNVS();
            }
        }
        return;
    }

    // ── SERVO:CENTER=c0,c1,c2,c3,c4,c5 — Set servo center calibration ─
    if (strncmp(data, "SERVO:CENTER=", 13) == 0) {
        int vals[6];
        int parsed = sscanf(data + 13, "%d,%d,%d,%d,%d,%d",
            &vals[0], &vals[1], &vals[2], &vals[3], &vals[4], &vals[5]);
        if (parsed == 6) {
            for (int i = 0; i < 6; i++) {
                if (vals[i] >= SERVO_MIN_US && vals[i] <= SERVO_MAX_US) {
                    servoCenter[i] = vals[i];
                }
            }
            serial_printf("SERVO:CENTER=%d,%d,%d,%d,%d,%d\r\n",
                servoCenter[0], servoCenter[1], servoCenter[2],
                servoCenter[3], servoCenter[4], servoCenter[5]);
            saveConfigToNVS();
        } else {
            serial_printf("ERR:SERVO:CENTER needs 6 comma-separated values\r\n");
        }
        return;
    }

    // ── SERVO:PULSE=value — Set pulse-per-radian multiplier ──────────
    if (strncmp(data, "SERVO:PULSE=", 12) == 0) {
        float val = atof(data + 12);
        if (val > 0.0f && val < 10000.0f) {
            servoPulsePerRad = val;
            serial_printf("SERVO:PULSE=%.1f\r\n", servoPulsePerRad);
            saveConfigToNVS();
        } else {
            serial_printf("ERR:SERVO:PULSE out of range\r\n");
        }
        return;
    }

    // ── TELRATE? — Query telemetry rate ────────────────────────────────
    if (strcmp(data, "TELRATE?") == 0) {
        serial_printf("TELRATE:%d\r\n", (int)(1000 / telemetryDelayMs));
        return;
    }

    // ── TELRATE:N — Set telemetry rate in Hz (1-100) ────────────────
    if (strncmp(data, "TELRATE:", 8) == 0) {
        int hz = atoi(data + 8);
        if (hz >= 1 && hz <= 100) {
            telemetryDelayMs = 1000 / hz;
            if (telemetryDelayMs < 10) telemetryDelayMs = 10;  // cap at 100Hz
            telemetryEnabled = true;  // start sending telemetry
            serial_printf("TELRATE:%d (delay=%dms)\r\n", hz, telemetryDelayMs);
        } else {
            serial_printf("ERR:TELRATE range 1-100\r\n");
        }
        return;
    }

    // ── MCA? — Query motion cueing config ──────────────────────────────
    if (strcmp(data, "MCA?") == 0) {
        serial_printf("MCA:preset=%s,enabled=%d,sr=%.0f\r\n",
            mcaPresetName(mcaConfig.preset), mcaConfig.enabled, mcaConfig.sample_rate);
        serial_printf("MCA:intensity=%.3f,gain=%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
            mcaGetIntensity(&mcaConfig),
            mcaConfig.axis_gain[0], mcaConfig.axis_gain[1], mcaConfig.axis_gain[2],
            mcaConfig.axis_gain[3], mcaConfig.axis_gain[4], mcaConfig.axis_gain[5]);
        serial_printf("MCA:invert=%d,%d,%d,%d,%d,%d\r\n",
            mcaConfig.axis_invert[0], mcaConfig.axis_invert[1], mcaConfig.axis_invert[2],
            mcaConfig.axis_invert[3], mcaConfig.axis_invert[4], mcaConfig.axis_invert[5]);
        return;
    }

    // ── MCA:preset_name / granular cue-param setters ────────────────
    // Granular setters map to the shared stewart-core API + the new output
    // stage (intensity / per-axis gain / invert). Persist with MCA:SAVE
    // (mcaSaveToNVS writes the whole shared struct blob).
    if (strncmp(data, "MCA:", 4) == 0) {
        const char* name = data + 4;
        if (strcmp(name, "SAVE") == 0) {
            mcaSaveToNVS(&mcaConfig);
            serial_printf("MCA:SAVED\r\n");
            return;
        }
        if (strcmp(name, "RESET") == 0) {
            resetMotionCueing(&mcaConfig);
            resetInputFilter(&inputFilter);
            serial_printf("MCA:RESET\r\n");
            return;
        }
        // ── Output stage (v6) ──
        if (strncmp(name, "INTENSITY=", 10) == 0) {
            float v = atof(name + 10);
            mcaSetIntensity(&mcaConfig, v);
            serial_printf("MCA:INTENSITY=%.3f\r\n", mcaGetIntensity(&mcaConfig));
            return;
        }
        if (strncmp(name, "GAIN=", 5) == 0) {          // output-stage per-axis gain
            int ax; float v;
            if (sscanf(name + 5, "%d,%f", &ax, &v) == 2) {
                mcaSetAxisGain(&mcaConfig, ax, v);
                serial_printf("MCA:GAIN[%d]=%.3f\r\n", ax, mcaGetAxisGain(&mcaConfig, ax));
            } else serial_printf("MCA:ERR GAIN=<axis>,<val>\r\n");
            return;
        }
        if (strncmp(name, "INVERT=", 7) == 0) {
            int ax, v;
            if (sscanf(name + 7, "%d,%d", &ax, &v) == 2) {
                mcaSetAxisInvert(&mcaConfig, ax, v);
                serial_printf("MCA:INVERT[%d]=%d\r\n", ax, mcaGetAxisInvert(&mcaConfig, ax));
            } else serial_printf("MCA:ERR INVERT=<axis>,<0|1>\r\n");
            return;
        }
        // ── Washout channel setters ──
        if (strncmp(name, "CHGAIN=", 7) == 0) {
            int ax; float v;
            if (sscanf(name + 7, "%d,%f", &ax, &v) == 2) {
                mcaSetChannelGain(&mcaConfig, ax, v);
                serial_printf("MCA:CHGAIN[%d]=%.3f\r\n", ax, v);
            } else serial_printf("MCA:ERR CHGAIN=<axis>,<val>\r\n");
            return;
        }
        if (strncmp(name, "HPFC=", 5) == 0) {
            int ax; float v;
            if (sscanf(name + 5, "%d,%f", &ax, &v) == 2) {
                mcaSetChannelHpFc(&mcaConfig, ax, v);
                serial_printf("MCA:HPFC[%d]=%.3f\r\n", ax, v);
            } else serial_printf("MCA:ERR HPFC=<axis>,<fc>\r\n");
            return;
        }
        if (strncmp(name, "LPFC=", 5) == 0) {
            int ax; float v;
            if (sscanf(name + 5, "%d,%f", &ax, &v) == 2) {
                mcaSetChannelLpFc(&mcaConfig, ax, v);
                serial_printf("MCA:LPFC[%d]=%.3f\r\n", ax, v);
            } else serial_printf("MCA:ERR LPFC=<axis>,<fc>\r\n");
            return;
        }
        // ── Tilt coordination gains (surge->pitch, sway->roll) ──
        if (strncmp(name, "TILT=", 5) == 0) {
            float sg, wg;
            if (sscanf(name + 5, "%f,%f", &sg, &wg) == 2) {
                mcaSetTiltSurgeGain(&mcaConfig, sg);
                mcaSetTiltSwayGain(&mcaConfig, wg);
                serial_printf("MCA:TILT surge=%.3f sway=%.3f\r\n", sg, wg);
            } else serial_printf("MCA:ERR TILT=<surge>,<sway>\r\n");
            return;
        }
        int found = -1;
        for (int i = 0; i < MCA_PRESET_COUNT; i++) {
            if (strcmp(name, mcaPresetName(i)) == 0) { found = i; break; }
        }
        if (found >= 0) {
            setMotionCueingPreset(&mcaConfig, found);
            serial_printf("MCA:OK preset=%s enabled=%d\r\n", mcaPresetName(found), mcaConfig.enabled);
        } else {
            serial_printf("MCA:ERR unknown preset '%s' (off/gentle/moderate/aggressive/race_pro)\r\n", name);
        }
        return;
    }

    // ── ACCEL? — Query accel input config ────────────────────────────
    if (strcmp(data, "ACCEL?") == 0) {
        serial_printf("ACCEL:gain=%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
            accelGain[0], accelGain[1], accelGain[2],
            accelGain[3], accelGain[4], accelGain[5]);
        serial_printf("ACCEL:map=%d,%d,%d,%d,%d,%d\r\n",
            accelAxisMap[0], accelAxisMap[1], accelAxisMap[2],
            accelAxisMap[3], accelAxisMap[4], accelAxisMap[5]);
        serial_printf("ACCEL:mode=%d,packets=%lu,ble=%s\r\n",
            (int)inputMode, accelPacketCount, ble_transport_state_str());
        return;
    }

    // ── ACCEL:GAIN=s,sw,h,r,p,y — Set per-axis accel gains ─────────
    if (strncmp(data, "ACCEL:GAIN=", 11) == 0) {
        float g[6];
        int parsed = sscanf(data + 11, "%f,%f,%f,%f,%f,%f",
            &g[0], &g[1], &g[2], &g[3], &g[4], &g[5]);
        if (parsed == 6) {
            for (int i = 0; i < 6; i++) accelGain[i] = g[i];
            serial_printf("ACCEL:GAIN=%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                accelGain[0], accelGain[1], accelGain[2],
                accelGain[3], accelGain[4], accelGain[5]);
        } else {
            serial_printf("ERR:ACCEL:GAIN needs 6 comma-separated values\r\n");
        }
        return;
    }

    // ── ACCEL:MAP=s,sw,h,r,p,y — Set axis mapping (1-based, neg=invert) ─
    if (strncmp(data, "ACCEL:MAP=", 10) == 0) {
        int m[6];
        int parsed = sscanf(data + 10, "%d,%d,%d,%d,%d,%d",
            &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]);
        if (parsed == 6) {
            for (int i = 0; i < 6; i++) accelAxisMap[i] = (int8_t)m[i];
            serial_printf("ACCEL:MAP=%d,%d,%d,%d,%d,%d\r\n",
                accelAxisMap[0], accelAxisMap[1], accelAxisMap[2],
                accelAxisMap[3], accelAxisMap[4], accelAxisMap[5]);
        } else {
            serial_printf("ERR:ACCEL:MAP needs 6 comma-separated values\r\n");
        }
        return;
    }

    // ── ESTOP:SOFT — Return all servos to center ─────────────────────
    if (strcmp(data, "ESTOP:SOFT") == 0) {
        playbackActive = false;   // E-stop overrides playback
        float home[6] = {0, 0, 0, 0, 0, 0};
        writeTarget(home, TGT_PHYS);   // CueTask homes on next tick
        serial_printf("ESTOP:SOFT -- Servos homing to center\r\n");
        return;
    }

    // ── ZERO — Reset to home position ────────────────────────────────
    if (strcmp(data, "ZERO") == 0) {
        float home[6] = {0, 0, 0, 0, 0, 0};
        writeTarget(home, TGT_PHYS);
        serial_printf("ZERO:OK -- All servos homing to center\r\n");
        return;
    }

    // ── SOURCE:* — Motion source selector (OFF / DEMO / LIVE) ─────────
    // SOURCE:OFF|DEMO|LIVE  |  SOURCE:BOOT=OFF|DEMO|LIVE  |  SOURCE?
    if (strcmp(data, "SOURCE?") == 0) {
        serial_printf("SOURCE:%s boot=%s\r\n", sourceName(g_source), sourceName(loadBootSource()));
        return;
    }
    if (strncmp(data, "SOURCE:", 7) == 0) {
        const char* arg = data + 7;
        Source parsed; bool ok = true;
        if      (strcmp(arg, "OFF")  == 0) parsed = SRC_OFF;
        else if (strcmp(arg, "DEMO") == 0) parsed = SRC_DEMO;
        else if (strcmp(arg, "LIVE") == 0) parsed = SRC_LIVE;
        else ok = false;
        if (ok) {
            setSource(parsed);
            serial_printf("SOURCE:%s\r\n", sourceName(g_source));
            return;
        }
        if (strncmp(arg, "BOOT=", 5) == 0) {
            const char* b = arg + 5;
            Source bs; bool bok = true;
            if      (strcmp(b, "OFF")  == 0) bs = SRC_OFF;
            else if (strcmp(b, "DEMO") == 0) bs = SRC_DEMO;
            else if (strcmp(b, "LIVE") == 0) bs = SRC_LIVE;
            else bok = false;
            if (bok) { saveBootSource(bs); serial_printf("SOURCE:BOOT=%s\r\n", sourceName(bs)); }
            else serial_printf("SOURCE:ERR boot expects OFF|DEMO|LIVE\r\n");
            return;
        }
        serial_printf("SOURCE:ERR unknown '%s' (OFF|DEMO|LIVE|BOOT=...)\r\n", arg);
        return;
    }

    // ── SERVO:RATE / SERVO:MODE — servo-rate profile (analog/digital) ─
    // SERVO:RATE=50|250 (Hz)  |  SERVO:MODE=ANALOG|DIGITAL  |  SERVO:RATE?
    // Sets BOTH the LEDC carrier and the CueTask loop rate; persists in NVS.
    if (strcmp(data, "SERVO:RATE?") == 0) {
        serial_printf("SERVO:RATE=%u (%s)\r\n", (unsigned)servoRateHz,
                      servoRateHz >= 200 ? "digital" : "analog");
        return;
    }
    if (strncmp(data, "SERVO:RATE=", 11) == 0) {
        int hz = atoi(data + 11);
        if (hz >= SERVO_RATE_MIN_HZ && hz <= SERVO_RATE_MAX_HZ) {
            applyServoRate((uint16_t)hz);
            saveConfigToNVS();
            serial_printf("SERVO:RATE=%u (cueLoopHz + carrier)\r\n", (unsigned)servoRateHz);
        } else {
            serial_printf("ERR:SERVO:RATE range %d-%d\r\n", SERVO_RATE_MIN_HZ, SERVO_RATE_MAX_HZ);
        }
        return;
    }
    if (strncmp(data, "SERVO:MODE=", 11) == 0) {
        const char* m = data + 11;
        if (strcmp(m, "ANALOG") == 0) {
            applyServoRate(SERVO_RATE_ANALOG_HZ); saveConfigToNVS();
            serial_printf("SERVO:MODE=ANALOG (%dHz)\r\n", SERVO_RATE_ANALOG_HZ);
        } else if (strcmp(m, "DIGITAL") == 0) {
            applyServoRate(SERVO_RATE_DIGITAL_HZ); saveConfigToNVS();
            serial_printf("SERVO:MODE=DIGITAL (%dHz)\r\n", SERVO_RATE_DIGITAL_HZ);
        } else {
            serial_printf("ERR:SERVO:MODE expects ANALOG|DIGITAL\r\n");
        }
        return;
    }

    // ── PLAY:* — Embedded motion-cued sequence playback ──────────────
    // PLAY:START | PLAY:STOP | PLAY:LOOP=0/1 | PLAY:STATUS | PLAY:BOOT=0/1
    if (strncmp(data, "PLAY:", 5) == 0) {
        const char* arg = data + 5;
        if (strcmp(arg, "START") == 0) {          // alias for SOURCE:DEMO
            if (!seqSamples) { serial_printf("PLAY:ERR no sequence\r\n"); return; }
            setSource(SRC_DEMO);
            serial_printf("PLAY:START %u samples @ %uHz (%d-bit, %s)\r\n",
                          (unsigned)seqCount, (unsigned)seqRateHz, seqBits,
                          g_framesRaw ? "raw" : "baked");
        } else if (strcmp(arg, "STOP") == 0) {    // alias for SOURCE:OFF
            setSource(SRC_OFF);
            serial_printf("PLAY:STOP\r\n");
        } else if (strncmp(arg, "LOOP=", 5) == 0) {
            playbackLoop = atoi(arg + 5) != 0;
            serial_printf("PLAY:LOOP=%d\r\n", playbackLoop ? 1 : 0);
        } else if (strcmp(arg, "STATUS") == 0) {
            serial_printf("PLAY:STATUS active=%d idx=%u/%u rate=%u loop=%d src=%s boot=%s\r\n",
                playbackActive ? 1 : 0, (unsigned)playbackIdx, (unsigned)seqCount,
                (unsigned)seqRateHz, playbackLoop ? 1 : 0,
                sourceName(g_source), sourceName(loadBootSource()));
        } else if (strncmp(arg, "BOOT=", 5) == 0) {   // alias: 0=OFF, 1=DEMO
            bool on = atoi(arg + 5) != 0;
            saveBootSource(on ? SRC_DEMO : SRC_OFF);
            serial_printf("PLAY:BOOT=%d (source=%s)\r\n", on ? 1 : 0, on ? "DEMO" : "OFF");
        } else {
            serial_printf("PLAY:ERR unknown '%s'\r\n", arg);
        }
        return;
    }

    // ── CSV motion data: val0,val1,val2,val3,val4,val5 ───────────────
    // Parse comma-separated raw values (legacy protocol)
    float raw[6] = {0};
    int count = 0;
    char* tok = strtok(data, ",");
    while (tok != NULL && count < 6) {
        raw[count++] = (float)atof(tok);
        tok = strtok(NULL, ",");
    }

    if (count == 6) {
        if (liveMotionGate())
            writeTarget(raw, TGT_BAKED);   // legacy CSV = raw counts, baked path
    }
}

// ── Interface Monitor Task (Core 0: serial I/O via COBS) ─────────────

void InterfaceMonitorTask(void* pvParameters) {
    for (;;) {
        // COBS transport: read bytes, decode frames, dispatch to registered handlers
        // (data_handler → process_binary_packet, cmd_handler → process_data)
        int got = cobs_read_process(5);

        // Yield when no data — prevents tight busy-loop from starving core 0
        if (got == 0)
            vTaskDelay(1);
    }
}

// ── App Main ─────────────────────────────────────────────────────────

extern "C" void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Create mutexes
    xMutex = xSemaphoreCreateMutex();

    // Initialize COBS transport on UART0 (must be before any serial_printf)
    cobs_transport_init(921600);
    cobs_set_data_handler([](const uint8_t *payload, int len) {
        (void)len;
        if (liveMotionGate())          // SOURCE:OFF gates; DEMO auto-switches to LIVE
            process_binary_packet(payload);   // baked -> shared target
    });
    cobs_set_data_raw_handler([](const uint8_t *payload, int len) {
        (void)len;
        if (liveMotionGate())
            process_raw_packet(payload);       // RAW pre-cue -> shared target (cued by CueTask)
    });
    cobs_set_cmd_handler([](const char *cmd) {
        char buf[256];
        int n = snprintf(buf, sizeof(buf), "%s", cmd);
        if (n > 0) process_data(buf);
    });

    // Initialize platform config with Mini-6DOF defaults, then overlay NVS
    initMiniDefaults(&stewartConfig);
    loadConfigFromNVS();

    // Compute axis scales from geometry (may have been loaded from NVS)
    computeAxisScalesFromGeometry(&axisScales, &stewartConfig, 0.90f);

    serial_printf("\r\n");
    serial_printf("+==========================================+\r\n");
    serial_printf("|     Mini-6DOF Controller v%s          |\r\n", FW_VERSION_STRING);
    serial_printf("|     Protocol: %d  Platform: %s   |\r\n", FW_PROTOCOL_VERSION, FW_PLATFORM_ID);
    serial_printf("+==========================================+\r\n");
    serial_printf("Geometry: RD=%.2f PD=%.2f L1=%.2f L2=%.2f H=%.2f\r\n",
        stewartConfig.RD, stewartConfig.PD,
        stewartConfig.ServoArmLengthL1, stewartConfig.ConnectingArmLengthL2,
        stewartConfig.platformHeight);
    serial_printf("Scales: %.1f,%.1f,%.1f,%.1f,%.1f,%.1f\r\n",
        axisScales.scale[0], axisScales.scale[1], axisScales.scale[2],
        axisScales.scale[3], axisScales.scale[4], axisScales.scale[5]);
    serial_printf("Bit depth: %d (max_raw=%.0f)\r\n", inputBitRange, maxRawInput);

    // Print fingerprint
    {
        uint8_t mac[6];
        esp_efuse_mac_get_default(mac);
        serial_printf("Fingerprint: %02X%02X%02X%02X%02X%02X\r\n",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    // Setup servo enable pin
    gpio_set_direction((gpio_num_t)SERVO_ENABLE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)SERVO_ENABLE_PIN, 0); // disabled initially

    // Setup LEDC PWM for servos
    setupServoPWM();

    // Home all servos to center (direct — CueTask not started yet)
    {
        float home[6] = {0, 0, 0, 0, 0, 0};
        driveServos(home, 1.0f / (float)servoRateHz);
    }

    serial_printf("Servos initialized at center. Enabling power...\r\n");

    // Brief delay then enable servo power
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level((gpio_num_t)SERVO_ENABLE_PIN, 1);

    serial_printf("Servo power ON. Ready for motion data.\r\n");

    // Start serial monitor task on Core 0
    xTaskCreatePinnedToCore(
        InterfaceMonitorTask,
        "SerialMonitor",
        8192,
        NULL,
        5,
        NULL,
        0
    );

    serial_printf("Serial monitor started. Accepting commands.\r\n");
    serial_printf("Commands: VERSION? FINGERPRINT? CONFIG? SCALE? BITS:N ZERO ESTOP:SOFT\r\n");
    serial_printf("         MCA? MCA:preset ACCEL? ACCEL:GAIN= ACCEL:MAP=\r\n");

    // Initialize the shared on-device cue engine (MCA + input filter).
    initMotionCueing(&mcaConfig, MCA_SAMPLE_RATE);
    initInputFilter(&inputFilter, MCA_SAMPLE_RATE);
    if (mcaLoadFromNVS(&mcaConfig) == 0) {
        serial_printf("MCA: Loaded from NVS (preset=%s, intensity=%.2f)\r\n",
                      mcaPresetName(mcaConfig.preset), mcaGetIntensity(&mcaConfig));
    } else {
        setMotionCueingPreset(&mcaConfig, MCA_MODERATE);
        serial_printf("MCA: Default preset=%s\r\n", mcaPresetName(mcaConfig.preset));
    }
    // Sync biquads + LEDC carrier to the servo-rate profile (FIX TRAP A).
    applyServoRate(servoRateHz);
    serial_printf("SERVO:RATE=%u Hz (cueLoopHz; %s)\r\n", (unsigned)servoRateHz,
                  servoRateHz >= 200 ? "digital" : "analog");

    // Initialize BLE transport
#ifdef ENABLE_BLE
    if (ble_transport_init(process_binary_packet)) {
        ble_transport_set_accel_callback(process_accel_packet);
        serial_printf("BLE initialized -- advertising as 'Mini6DOF'\r\n");
        serial_printf("BLE accel char 0xFF03: 24-byte [ax,ay,az,gx,gy,gz] float32 LE\r\n");
    } else {
        serial_printf("BLE init FAILED\r\n");
    }
#endif

    // ── CueTask: the fixed-rate consumer + SOLE servo writer ─────────
    // High prio, core 1 (APP_CPU), clear of the serial monitor on core 0.
    xTaskCreatePinnedToCore(CueTask, "Cue", 4096, NULL, 7, NULL, 1);

    // ── Embedded motion-cued sequence playback (producer) ────────────
    bool haveSeq = parseSequence();
    if (haveSeq) {
        serial_printf("PLAY: sequence ready -- %u samples @ %uHz, %d-bit, %s, loop@%u (~%.1fs)\r\n",
            (unsigned)seqCount, (unsigned)seqRateHz, seqBits, g_framesRaw ? "raw" : "baked",
            (unsigned)seqLoopPoint, (double)seqCount / (seqRateHz ? seqRateHz : 50));
        xTaskCreatePinnedToCore(PlaybackTask, "Playback", 4096, NULL, 6, NULL, 1);
    } else {
        serial_printf("PLAY: no valid embedded sequence\r\n");
    }

    // ── Apply the boot source (OFF / DEMO / LIVE), default = DEMO ─────
    Source bootSrc = loadBootSource();
    if (bootSrc == SRC_DEMO && !haveSeq) {
        serial_printf("SOURCE: boot=DEMO but no sequence -> OFF\r\n");
        setSource(SRC_OFF);
    } else {
        setSource(bootSrc);
        serial_printf("SOURCE: boot=%s applied\r\n", sourceName(g_source));
    }

    // Seed watchdog timer so it doesn't trip immediately on boot
    lastPacketTimeUs = esp_timer_get_time();

    // Main loop: 6-axis processing + watchdog + telemetry
    for (;;) {
        // ── BLE 6-Axis Pipeline ──────────────────────────────────────
        // Phone sends [roll_deg, pitch_deg, yaw_deg, surge_ms2, sway_ms2, heave_ms2]
        // Rotation: degrees → radians, apply gain
        // Translation: m/s² → mm, apply gain (gain acts as mm-per-m/s²)
        if (accelFresh && inputMode == INPUT_BLE_ACCEL) {
            accelFresh = false;

            // Rotation axes: degrees → radians
            float roll_rad  = accelRaw[0] * DEG_TO_RAD * accelGain[3];
            float pitch_rad = accelRaw[1] * DEG_TO_RAD * accelGain[4];
            float yaw_rad   = accelRaw[2] * DEG_TO_RAD * accelGain[5];

            // Translation axes: m/s² → mm (gain = mm per m/s²)
            float surge_mm = accelRaw[3] * accelGain[0];
            float sway_mm  = accelRaw[4] * accelGain[1];
            float heave_mm = accelRaw[5] * accelGain[2];

            // Position: [surge, sway, heave, roll, pitch, yaw] — already physical.
            // Producer: hand to CueTask as TGT_PHYS (it does slew + IK + servo).
            float position[6] = {surge_mm, sway_mm, heave_mm, roll_rad, pitch_rad, yaw_rad};
            if (g_source != SRC_OFF)
                writeTarget(position, TGT_PHYS);
        }

        // ── Activity watchdog (advisory) ──────────────────────────────
        // CueTask already decays to home when the target goes stale
        // (CUE_LOST_DECAY_MS), so the watchdog no longer drives servos — it
        // just logs once and drops BLE-accel mode back to serial.
        int64_t now = esp_timer_get_time();
        if (lastPacketTimeUs > 0 && (now - lastPacketTimeUs) > WATCHDOG_TIMEOUT_US) {
            if (!watchdogTripped) {
                watchdogTripped = true;
                if (inputMode == INPUT_BLE_ACCEL) {
                    inputMode = INPUT_SERIAL;  // fall back to serial when accel stops
                }
                serial_printf("WDT:HOME -- No input for 500ms (CueTask decaying to home)\r\n");
            }
        }

        // ── Binary COBS telemetry ──
        // Telemetry stays silent until app sends TELRATE:N after handshake.
        if (telemetryEnabled) {
            float positions[6];
            for (int i = 0; i < 6; i++) positions[i] = arr[i];
            cobs_send_telemetry((const float*)lastServoAngles, positions);
        }

        vTaskDelay(pdMS_TO_TICKS(telemetryDelayMs));
    }
}
