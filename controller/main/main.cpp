/*
 * Stewart Platform Motion Controller - ESP-IDF Native Implementation
 * 
 * Pure ESP-IDF implementation (no Arduino framework dependency)
 * Optimized for ESP32-S3 with deterministic real-time control
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <atomic>

// ESP-IDF headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include <fcntl.h>
#include <unistd.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_task_wdt.h"

// Project headers
#include "helpers.h"
#include "EStopSafety.h"
#include "DriveScaleValidation.h"
#if defined(R14_FAIL_CLOSED) && PCB_VERSION == 2
#include "R14DriveAlarmLatch.h"
#include "R14MotionAdmission.h"
#include "R14RuntimeSafety.h"
#endif
#include "ControllerMotionPayload.h"
#if defined(R14_BOARD_IO)
#include "R14Hardware.h"
#endif
#if defined(R13_BENCH_ONE_AXIS) && defined(R14_FAIL_CLOSED) && PCB_VERSION == 2
#include "R13BenchAxis.h"
#define R13_BENCH_ACTIVE 1
#endif
#include "debug_uart.h"
#include "InverseKinematics.h"
#include "AxisScaling.h"
#include "MotionCueing.h"
#include "version.h"

// ── Step driver backend (compile-time selectable) ───────────────────
// Set backend in main/CMakeLists.txt (STEP_DRIVER_BACKEND variable).
// Defaults to STEP_DRIVER_SSE (SimpleStepEngine) if none defined.
// PCBv1 (ESP32 + MCP23S17) keeps its own path below; StepDriver is PCBv2-only.
//
// STEP_DRIVER_ISR_BACKEND = true when using any single-timer ISR backend
// (STEP_DRIVER_MCPWM_ISR or STEP_DRIVER_SHARED_MCPWM). These backends use
// StepDriver_getTotalSteps/resetStats/isInitialized via the engine singleton.
// STEP_DRIVER_MCPWM (MCPWMMotorControl) uses the per-motor _mcpwm_motors[] array.
#if defined(STEP_DRIVER_MCPWM_ISR) || defined(STEP_DRIVER_SHARED_MCPWM)
#  define STEP_DRIVER_ISR_BACKEND 1
#endif
#if PCB_VERSION == 1
#include "MCP23S17.h"
#include "GPTimerScheduler.h"
#else
#include "StepDriver.h"  // selects SSE or MCPWM backend via define
#if defined(R13_BENCH_ONE_AXIS) && !defined(STEP_DRIVER_SHARED_MCPWM)
#error "R13_BENCH_ONE_AXIS requires STEP_DRIVER_SHARED_MCPWM (the bench hooks read that engine)"
#endif
#if defined(R13_BENCH_ONE_AXIS) && defined(STEP_DRIVER_SHARED_MCPWM)
// Bench: step-engine ISR cost (see R13BenchAxis.cpp BENCH:STATUS isr=...).
void r13_engine_isr_stats(uint32_t *count, uint32_t *cycles, uint32_t *maxc, bool reset) {
    SharedMcpwmStepEngine &e = SharedMcpwmStepEngine::instance();
    *count = e.isrCount(); *cycles = e.isrCycles(); *maxc = e.isrMaxCycles();
    if (reset) e.resetIsrStats();
}
void r13_engine_isr_gap(uint32_t *gap_min, uint32_t *gap_sum) {
    SharedMcpwmStepEngine &e = SharedMcpwmStepEngine::instance();
    *gap_min = e.isrGapMin(); *gap_sum = e.isrGapSum();
}
uint32_t r13_engine_isr_missed(void) { return SharedMcpwmStepEngine::instance().isrMissed(); }
void r13_engine_latch_stats(uint32_t *dneg, uint32_t *dpos, uint32_t *dbig, uint32_t *mpos, uint32_t *smax) {
    SharedMcpwmStepEngine::instance().latchStats(dneg, dpos, dbig, mpos, smax);
}
// Latch-event ring (bench diagnostics): returns the event count; reset=true clears the ring.
uint32_t r13_engine_latch_events(bool reset) {
    SharedMcpwmStepEngine &e = SharedMcpwmStepEngine::instance();
    uint32_t n = e._ringN; if (reset) e._ringN = 0; return n;
}
// k-th of the last LATCH_RING events (tick, since, m, skip, hist0; d separately); false past the end.
bool r13_engine_latch_event(uint32_t k, uint32_t n, uint32_t out[5], int32_t *d) {
    SharedMcpwmStepEngine &e = SharedMcpwmStepEngine::instance();
    if (k >= n) return false;
    uint32_t first = n > (uint32_t)SharedMcpwmStepEngine::LATCH_RING ? n - SharedMcpwmStepEngine::LATCH_RING : 0;
    if (k < first) return false;
    volatile SharedMcpwmStepEngine::LatchEvent &ev = e._ring[k % SharedMcpwmStepEngine::LATCH_RING];
    out[0] = ev.tick; out[1] = ev.since; out[2] = ev.m; out[3] = ev.skip; out[4] = ev.hist0; *d = ev.d;
    return true;
}
uint32_t r13_engine_period_cycles(void) { return SharedMcpwmStepEngine::instance().periodCycles(); }
void r13_engine_resync_stats(uint32_t *resyncs, uint32_t *skipmax) {
    SharedMcpwmStepEngine &e = SharedMcpwmStepEngine::instance(); *resyncs = e.resyncs(); *skipmax = e.skipMax();
}
void r13_engine_isr_delay(uint32_t cyc) { SharedMcpwmStepEngine::instance().setIsrDelay(cyc); }
bool r13_engine_latch_comp(int set) {
    if (set >= 0) SharedMcpwmStepEngine::instance().setLatchComp(set != 0);
    return SharedMcpwmStepEngine::instance().latchComp();
}
#endif
#endif

#ifdef ENABLE_ETHERNET
#include "EthernetTransport.h"
#endif

#include "LedStatus.h"
#define STATUS_LED_GPIO 48
#ifdef ENABLE_WIFI
#include "WifiTransport.h"
#endif
#ifdef ENABLE_BLE
#include "BleTransport.h"
#endif
#include "CobsTransport.h"
#include "ControllerHandshake.h"

static const char *TAG = "stewart_main";

// Platform configuration (single source of truth for geometry)
static StewartConfig stewartConfig;

#if defined(R13_BENCH_ONE_AXIS)
// Bench: run the production IK on a pose and report the six arm angles (deg) and axis steps
// relative to the home pose, exactly as setPos() would compute them. pos = x y z (mm), roll
// pitch yaw (rad). Returns validatePosition()'s verdict (1 = reachable).
int r13_bench_ik(const float pos[6], float angles_deg[6], long steps[6]) {
    static const float home[6] = {0, 0, 0, 0, 0, 0};
    float a[6], h[6];
    calculateAllServoAngles(home, &stewartConfig, h);
    calculateAllServoAngles(pos, &stewartConfig, a);
    for (int i = 0; i < 6; i++) {
        angles_deg[i] = a[i] * IK_RAD_TO_DEG;
        steps[i] = (long)((a[i] - h[i]) * IK_RAD_TO_DEG * stewartConfig.steps_per_degree);
    }
    return validatePosition(pos, &stewartConfig);
}
float r13_bench_steps_per_degree() { return stewartConfig.steps_per_degree; }
#endif

// Per-axis scaling (derived from platform geometry at boot, read-only via SCALE?)
static AxisScaleConfig axisScales;

// Input bit depth — determines max_raw for mapRawToPosition
// Configurable at runtime via BITS:N serial command, default 12-bit
static uint8_t  inputBitRange = 12;
static float    maxRawInput   = 4094.0f;  // (1 << 12) - 2 = 4094

// Motion cueing (classical washout + tilt coordination, NVS-persistent)
static MotionCueingConfig mcaConfig;

// Motor control variables
volatile float arr[6] = {0, 0, 0, 0, 0, 0};

#if PCB_VERSION == 1
// ── PCBv1: MCP23S17 SPI expander motor control ─────────────────────
static MCP23S17* outputBank = nullptr;
static volatile int32_t motorCurrentPos[6] = {0};
static volatile int32_t motorTargetPos[6] = {0};
static bool motorInitialized = false;
static uint16_t motorOutputReg = 0;
static bool stepPinState = false;
static const int mcpStepPins[6] = {
    MCP_STEP_PIN_0, MCP_STEP_PIN_1, MCP_STEP_PIN_2,
    MCP_STEP_PIN_3, MCP_STEP_PIN_4, MCP_STEP_PIN_5
};
static const int mcpDirPins[6] = {
    MCP_DIR_PIN_0, MCP_DIR_PIN_1, MCP_DIR_PIN_2,
    MCP_DIR_PIN_3, MCP_DIR_PIN_4, MCP_DIR_PIN_5
};
static const bool motorInverted[6] = { true, false, true, false, true, false };
#else
// ── PCBv2: step/dir pins (shared by all StepDriver backends) ────────
static const gpio_num_t stepPins[6] = {
    (gpio_num_t)STEP_PIN_1, (gpio_num_t)STEP_PIN_2, (gpio_num_t)STEP_PIN_3,
    (gpio_num_t)STEP_PIN_4, (gpio_num_t)STEP_PIN_5, (gpio_num_t)STEP_PIN_6
};
static const gpio_num_t dirPins[6] = {
    (gpio_num_t)DIR_PIN_1, (gpio_num_t)DIR_PIN_2, (gpio_num_t)DIR_PIN_3,
    (gpio_num_t)DIR_PIN_4, (gpio_num_t)DIR_PIN_5, (gpio_num_t)DIR_PIN_6
};
#endif

static bool estop_motion_is_blocked();

// ── Motor abstraction layer ─────────────────────────────────────────
// PCBv1: direct MCP23S17 arrays.
// PCBv2: delegates to whichever StepDriver backend is selected.
#if PCB_VERSION == 1
static inline int32_t motor_getPos(int i)    { return motorCurrentPos[i]; }
static inline int32_t motor_getTarget(int i) { return motorTargetPos[i]; }
static inline bool    motor_setTarget(int i, int32_t pos) { return controller::guarded_motor_target(estop_motion_is_blocked(), [=]() { motorTargetPos[i] = pos; return true; }); }
static inline void    motor_resetPosition(int i) { motorCurrentPos[i] = 0; motorTargetPos[i] = 0; }
static inline void    motor_emergencyStop(int i) { motorTargetPos[i] = motorCurrentPos[i]; }
static inline bool    motor_isInit(int)      { return motorInitialized; }
#else
static inline int32_t motor_getPos(int i)    { return StepDriver_getPosition(i); }
static inline int32_t motor_getTarget(int i) { return StepDriver_getTarget(i); }
static inline bool    motor_setTarget(int i, int32_t pos) { return controller::guarded_motor_target(estop_motion_is_blocked(), [=]() { return StepDriver_setTarget(i, pos); }); }
static inline void    motor_resetPosition(int i) { StepDriver_resetPosition(i); }
static inline void    motor_emergencyStop(int i) { StepDriver_setTarget(i, StepDriver_getPosition(i)); }
static inline bool    motor_isInit(int)      { return StepDriver_isInitialized(); }
#endif

// Loop timing instrumentation (updated every GPTimer tick)
static volatile uint32_t loopCount = 0;
static volatile uint32_t loopMin_us = UINT32_MAX;
static volatile uint32_t loopMax_us = 0;
static volatile uint64_t loopSum_us = 0;
static volatile bool     motorTestRunning = false;

// Timing variables
int64_t currentMicros = 0;
int64_t previousMicros = 0;
int64_t lastDebugOutput = 0;
int microInterval = MICRO_INTERVAL_FAST;
const int64_t DEBUG_OUTPUT_INTERVAL = 100000; // 100ms = 10Hz debug output

// E-stop debouncing
typedef struct {
    gpio_num_t pin;
    bool current_state;
    std::atomic<bool> debounced_state;
    int64_t last_change_time;
    int debounce_time_us;
} debounce_button_t;

static debounce_button_t estop_button;

// State variables
static std::atomic<bool> isPausedEStop{true};  // Boot disarmed; explicit reset is required.
#if defined(R14_FAIL_CLOSED) && PCB_VERSION == 2
static r14::DriveAlarmLatch r14DriveAlarms;
// Compile-time false: there is no qualified local-enable, Hall, drive-ready or
// gravity-bottom adapter. The separate operator arm and home requests in this
// admission core therefore cannot open the live motion gate.
static r14::MotionAdmission<false> r14MotionAdmission(1, 1, 1);
#endif
bool isRateLimiting = false;
bool debugEnabled = false;  // Debug output state, default disabled for safety
SemaphoreHandle_t xMutex = NULL;
TaskHandle_t xGPIOLoopHandle = NULL;  // Task handle for GPTimer notification
volatile float lastServoAngles[6] = {0};  // Latest IK output (radians) for telemetry

// ── Input source selection ───────────────────────────────────────────
// Only the active input source is allowed to feed process_binary_packet.
// Prevents stray packets from other transports corrupting the IK output.
typedef enum {
    INPUT_SOURCE_SERIAL = 0,
    INPUT_SOURCE_ETHERNET = 1,
    INPUT_SOURCE_WIFI = 2,
    INPUT_SOURCE_BLE = 3
} InputSource;
static volatile InputSource activeInputSource = INPUT_SOURCE_SERIAL;

static const char* inputSourceName(InputSource src) {
    switch (src) {
        case INPUT_SOURCE_SERIAL:   return "SERIAL";
        case INPUT_SOURCE_ETHERNET: return "ETHERNET";
        case INPUT_SOURCE_WIFI:     return "WIFI";
        case INPUT_SOURCE_BLE:      return "BLE";
        default:                    return "UNKNOWN";
    }
}

// Telemetry response: [0xBB] [0xCC] [float32 × 6 little-endian] [XOR checksum] = 27 bytes
// Sends computed servo angles (radians) back to dashboard after each motion packet
#define TEL_SYNC_0       0xBB
#define TEL_SYNC_1       0xCC
#define TEL_PAYLOAD_SIZE 24   // 6 × float32
#define TEL_PACKET_SIZE  27   // 2 sync + 24 payload + 1 checksum

// Function declarations
void setupMotorPins();
void handleStepDirection();
void applyMotionValues(float values[6]);
void process_data(char * data);
void process_binary_packet(const uint8_t *payload, int len);
void InterfaceMonitorTask(void * pvParameters);
void GPIOLoopTask(void * pvParameters);
void EStopMonitorTask(void * pvParameters);
void outputDebugData();
void initDebounceButton(debounce_button_t *btn, gpio_num_t pin, int debounce_ms);
bool updateDebounceButton(debounce_button_t *btn);

// Per-transport packet counters (for diagnostics via INPUT_STAT command)
static volatile uint32_t pktCount_serial = 0;
static volatile uint32_t pktCount_eth = 0;
static volatile uint32_t pktCount_wifi = 0;
static volatile uint32_t pktCount_ble = 0;
static volatile uint32_t pktDrop_serial = 0;
static volatile uint32_t pktDrop_eth = 0;
static volatile uint32_t pktDrop_wifi = 0;
static volatile uint32_t pktDrop_ble = 0;

// Per-transport wrappers that gate on activeInputSource
static void serial_packet_handler(const uint8_t *payload, int len) {
    pktCount_serial++;
    if (activeInputSource == INPUT_SOURCE_SERIAL)
        process_binary_packet(payload, len);
    else
        pktDrop_serial++;
}
#ifdef ENABLE_ETHERNET
static void ethernet_packet_handler(const uint8_t *payload, int len) {
    pktCount_eth++;
    if (activeInputSource == INPUT_SOURCE_ETHERNET)
        process_binary_packet(payload, len);
    else
        pktDrop_eth++;
}
#endif
#ifdef ENABLE_WIFI
static void wifi_packet_handler(const uint8_t *payload) {
    pktCount_wifi++;
    if (activeInputSource == INPUT_SOURCE_WIFI)
        process_binary_packet(payload, 12);
    else
        pktDrop_wifi++;
}
#endif
#ifdef ENABLE_BLE
static void ble_packet_handler(const uint8_t *payload) {
    pktCount_ble++;
    if (activeInputSource == INPUT_SOURCE_BLE)
        process_binary_packet(payload, 12);
    else
        pktDrop_ble++;
}
#endif

/* Serial output routed through COBS RESP channel */
void serial_printf(const char *format, ...) {
    char buf[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    if (len > 0) {
        if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
        // Strip trailing \r\n — COBS frames don't need line terminators
        while (len > 0 && (buf[len-1] == '\n' || buf[len-1] == '\r')) len--;
        if (len > 0)
            cobs_send(COBS_CH_RESP, (const uint8_t *)buf, len);
    }
}

void serial_println(const char *str) {
    cobs_send_str(COBS_CH_RESP, str);
}

// ── NVS persistence for Stewart platform geometry ───────────────────
#define CONFIG_NVS_NAMESPACE "stewart_cfg"
#define CONFIG_NVS_KEY       "geo"
#define CONFIG_NVS_VERSION   1

struct ConfigNvsBlob {
    uint8_t version;
    float RD, PD, L1, L2, height, theta_r, theta_p;
};

static int configSaveToNVS(const StewartConfig* cfg) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(CONFIG_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return -1;
    ConfigNvsBlob blob = {};
    blob.version = CONFIG_NVS_VERSION;
    blob.RD = cfg->RD; blob.PD = cfg->PD;
    blob.L1 = cfg->ServoArmLengthL1; blob.L2 = cfg->ConnectingArmLengthL2;
    blob.height = cfg->platformHeight;
    blob.theta_r = cfg->theta_r; blob.theta_p = cfg->theta_p;
    err = nvs_set_blob(handle, CONFIG_NVS_KEY, &blob, sizeof(blob));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return (err == ESP_OK) ? 0 : -1;
}

// ── NVS persistence for the drive train (DRIVE:gear/planetary/encoder) ───
// Stored as separate keys so the geometry blob version stays untouched.
static int driveSaveToNVS(const StewartConfig* cfg) {
    nvs_handle_t handle;
    if (nvs_open(CONFIG_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) return -1;
    uint32_t g, p; memcpy(&g, &cfg->virtual_gear, 4); memcpy(&p, &cfg->planetary_ratio, 4);
    esp_err_t err = nvs_set_u32(handle, "drv_gear", g);
    if (err == ESP_OK) err = nvs_set_u32(handle, "drv_planet", p);
    if (err == ESP_OK) err = nvs_set_i32(handle, "drv_enc", cfg->encoder_ppr);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return (err == ESP_OK) ? 0 : -1;
}

static int driveLoadFromNVS(StewartConfig* cfg) {
    nvs_handle_t handle;
    if (nvs_open(CONFIG_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return -1;
    uint32_t g = 0, p = 0; int32_t e = 0;
    bool ok = nvs_get_u32(handle, "drv_gear", &g) == ESP_OK &&
              nvs_get_u32(handle, "drv_planet", &p) == ESP_OK &&
              nvs_get_i32(handle, "drv_enc", &e) == ESP_OK;
    nvs_close(handle);
    if (!ok) return -1;
    float gf, pf; memcpy(&gf, &g, 4); memcpy(&pf, &p, 4);
    if (!valid_drive_scale(e, gf, pf)) return -1;
    cfg->virtual_gear = gf; cfg->planetary_ratio = pf; cfg->encoder_ppr = (int)e;
    computeStepsPerDegree(cfg);
    return 0;
}

static int configLoadFromNVS(StewartConfig* cfg) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(CONFIG_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) return -1;
    ConfigNvsBlob blob = {};
    size_t len = sizeof(blob);
    err = nvs_get_blob(handle, CONFIG_NVS_KEY, &blob, &len);
    nvs_close(handle);
    if (err != ESP_OK || len != sizeof(blob) || blob.version != CONFIG_NVS_VERSION)
        return -1;
    cfg->RD = blob.RD; cfg->PD = blob.PD;
    cfg->ServoArmLengthL1 = blob.L1; cfg->ConnectingArmLengthL2 = blob.L2;
    cfg->platformHeight = blob.height;
    cfg->theta_r = blob.theta_r; cfg->theta_p = blob.theta_p;
    return 0;
}

/* ESP-IDF replacements for Arduino timing functions */
static inline int64_t micros() {
    return esp_timer_get_time();
}

static inline int64_t millis() {
    return esp_timer_get_time() / 1000;
}

// Initialize debounce button
void initDebounceButton(debounce_button_t *btn, gpio_num_t pin, int debounce_ms) {
    btn->pin = pin;
    btn->debounce_time_us = debounce_ms * 1000;
    
    // Configure GPIO as input with pull-up BEFORE reading initial state,
    // otherwise floating pin may read LOW and trigger false E-stop.
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Now read initial state with pull-up active
    btn->current_state = gpio_get_level(pin);
    btn->debounced_state = btn->current_state;
    btn->last_change_time = esp_timer_get_time();
}

// Update debounce button state, returns true if state changed
bool updateDebounceButton(debounce_button_t *btn) {
    bool current = gpio_get_level(btn->pin);
    int64_t now = esp_timer_get_time();
    
    if (current != btn->current_state) {
        btn->current_state = current;
        btn->last_change_time = now;
    }
    
    if ((now - btn->last_change_time) > btn->debounce_time_us) {
        if (current != btn->debounced_state) {
            btn->debounced_state = current;
            return true;  // State changed
        }
    }
    
    return false;  // No change
}

static bool estop_input_active() {
    return gpio_get_level((gpio_num_t)ESTOP_PIN) == ESTOP_ACTIVE_STATE ||
           estop_button.debounced_state == ESTOP_ACTIVE_STATE;
}

static bool estop_motion_is_blocked() {
#if defined(R14_FAIL_CLOSED) && PCB_VERSION == 2
    if (r14::admission_blocks_motion(r14MotionAdmission, r14DriveAlarms)) return true;
#endif
    return estop_motion_blocked(isPausedEStop.load(),
                                gpio_get_level((gpio_num_t)ESTOP_PIN) == ESTOP_ACTIVE_STATE,
                                estop_button.debounced_state == ESTOP_ACTIVE_STATE);
}

#if defined(R14_FAIL_CLOSED) && PCB_VERSION == 2
// Task-context ingress: R14Hardware samples GPIO1/2/15/16/18/40. The measured
// R13 interface is LOW healthy / HIGH alarm, off or open; verify all R14 cables.
// Hardware clears only the faulted torque latch; common pulse stop uses GPIO47.
struct R14FaultStopOps {
    void pause() const { isPausedEStop.store(true); }
    void bench_stop(const char* reason) const {
#if defined(R13_BENCH_ACTIVE)
        r13_bench_stop(reason);
#else
        (void)reason;
#endif
    }
    void inhibit() const {
        gpio_set_level(GPIO_NUM_47, 1); // existing output-latch inhibit
        gpio_set_level(GPIO_NUM_48, 0);
    }
    bool initialized() const { return StepDriver_isInitialized(); }
    void stop() const { StepDriver_stop(); }
    void discard_targets() const { StepDriver_emergencyStop(); }
};

extern "C" void r14_report_normalized_drive_alarm(int axis) {
    r14::report_alarm_and_stop(r14MotionAdmission, r14DriveAlarms, axis, R14FaultStopOps{});
}

// These task-context entrypoints are reserved for a *local*, qualified IO
// task. They are deliberately absent from the network/serial command parser.
// MotionAdmission<false> denies both requests in the current R14 image.
extern "C" bool r14_request_local_arm(uint64_t now_us) {
    return r14MotionAdmission.request_operator_arm(now_us);
}
extern "C" bool r14_request_local_home(uint64_t now_us) {
    return r14MotionAdmission.request_operator_home(now_us);
}

extern "C" void r14_report_transport_reconnect(void) {
    r14::report_reconnect_and_stop(r14MotionAdmission, R14FaultStopOps{});
}
#endif

void setPos() {  
    float position[6];
    // Copy volatile arr under implicit atomicity (single writer)
    for (int i = 0; i < 6; i++) {
        position[i] = arr[i];
    }

    float servoAngles[6];
    calculateAllServoAngles(position, &stewartConfig, servoAngles);
    memcpy((void*)lastServoAngles, servoAngles, sizeof(lastServoAngles));

    xSemaphoreTake(xMutex, portMAX_DELAY);
    for (int i = 0; i < 6; i++) {
        long x = (long)(servoAngles[i] * IK_RAD_TO_DEG * stewartConfig.steps_per_degree);
        if (!motor_setTarget(i, x)) {
            DEBUG_PRINTF("Motor %d position error\n", i);
        }
    }
    xSemaphoreGive(xMutex);
}

#if PCB_VERSION == 1
void setupMotorPins() {
    // PCBv1: Initialize MCP23S17 SPI GPIO expander for step/dir output
    outputBank = new MCP23S17((spi_host_device_t)MCP_SPI_HOST,
                              (gpio_num_t)MCP_CS_PIN, 0);
    if (!outputBank->begin(MCP_SPI_MOSI, MCP_SPI_MISO, MCP_SPI_CLK, 8000000)) {
        DEBUG_PRINTLN("FATAL: MCP23S17 outputBank init failed");
        return;
    }
    // All 12 pins (6 step + 6 dir) are outputs — set via begin() default
    outputBank->allOutput();
    motorOutputReg = 0;
    outputBank->writeAll(0);  // all LOW

    for (int i = 0; i < 6; i++) {
        motorCurrentPos[i] = 0;
        motorTargetPos[i] = 0;
    }
    motorInitialized = true;
    stepPinState = false;
    DEBUG_PRINTLN("Motors initialized: 6 via MCP23S17 SPI expander (PCBv1)");
}
#else
// Step-engine soft limits, ±MOTOR_SOFT_LIMIT_DEG at the arm, converted with the drive's
// electronic gear (steps_per_degree) so they follow DRIVE:gear= instead of a step count
// that would be ±144° at 2:1. R13 arm: hard stop near −100°/+80°, test window ±60°.
#define MOTOR_SOFT_LIMIT_DEG 72.0f
static int32_t motorSoftLimitSteps() { return (int32_t)(MOTOR_SOFT_LIMIT_DEG * stewartConfig.steps_per_degree); }

void setupMotorPins() {
    // PCBv2: Initialize via whichever StepDriver backend is selected.
    // Change STEP_DRIVER_SSE / STEP_DRIVER_MCPWM in main/CMakeLists.txt to switch.
    StepDriverMotorConfig cfg[STEP_DRIVER_NUM_MOTORS];
    for (int i = 0; i < STEP_DRIVER_NUM_MOTORS; i++) {
        cfg[i].stepPin          = stepPins[i];
        cfg[i].dirPin           = dirPins[i];
        cfg[i].invertDir        = false;
        // stewartConfig (incl. the drive gear from NVS) is loaded before this task starts
        cfg[i].softLimitMin     = -motorSoftLimitSteps();
        cfg[i].softLimitMax     =  motorSoftLimitSteps();
        cfg[i].enableSoftLimits = true;
    }
    if (!StepDriver_init(cfg)) {
        DEBUG_PRINTLN("FATAL: StepDriver_init failed");
        return;
    }
#if defined(R14_FAIL_CLOSED)
    // R14 diagnostics do not grant measured homing/field qualification. The
    // backend is initialized with all STEP pins low but its timer never starts.
    for (int i = 0; i < STEP_DRIVER_NUM_MOTORS; ++i)
        gpio_set_level(stepPins[i], 0);
    DEBUG_PRINTLN("R14 fail-closed: six STEP outputs locked at boot");
#else
    if (!StepDriver_start()) {
        DEBUG_PRINTLN("FATAL: StepDriver_start failed");
        return;
    }
#endif
    // setupMotorPins may race the E-stop task at boot. Never leave a newly
    // started pulse engine running while the boot pause or NC input is active.
    if (estop_motion_is_blocked()) {
        StepDriver_stop();
        for (int i = 0; i < 6; i++) motor_emergencyStop(i);
    }
    StepDriver_report();
#if defined(R13_BENCH_ACTIVE)
    // Bench step 12: the six-axis gate stays closed; only console BENCH: commands drive the
    // engine, after an explicit BENCH:ARM with the stop loop closed. Initialise the engine
    // here, on core 1 like the production path, so its level-3 MCPWM ISR is allocated on
    // core 1 (a core-0 start from the console task crashed the board on 2026-10-20). In
    // bench builds start() leaves the period timer stopped (the ISR would starve this task
    // during the rest of boot); BENCH:ARM resumes it. The bench owns the engine from here on.
    if (!StepDriver_start()) DEBUG_PRINTLN("bench: StepDriver_start failed");
    static const R13BenchOps benchOps = {
        [](int i, int32_t p) { return StepDriver_setTarget(i, p); },
        [](int i) { return StepDriver_getPosition(i); },
        []() { StepDriver_resetPosition(0); StepDriver_resume(); return true; },   // ARM: axis 0 position := 0, then resume (soft limits ±72 deg)
        []() { StepDriver_stop(); },
        []() { StepDriver_emergencyStop(); },
        []() { return gpio_get_level((gpio_num_t)ESTOP_PIN) == ESTOP_ACTIVE_STATE; },
    };
    r13_bench_init(&benchOps);
#endif
}
#endif

#if PCB_VERSION == 1
void handleStepDirection() {
    // PCBv1: Batch step/dir via MCP23S17 SPI GPIO expander.
    // Alternates between step-HIGH and step-LOW phases each GPTimer tick.
    // Each motor advances 1 step per 2 ticks = 10kHz max step rate at 50µs interval.
    // SPI writes are batched: all 6 motors updated in 2-3 SPI transactions per tick.
    if (!motorInitialized || !outputBank) return;

    xSemaphoreTake(xMutex, portMAX_DELAY);

    if (stepPinState) {
        // Phase A: Bring all step pins LOW (end of pulse)
        for (int i = 0; i < 6; i++) {
            BIT_CLEAR(motorOutputReg, mcpStepPins[i]);
        }
        outputBank->writeAll(motorOutputReg);
        stepPinState = false;
    } else {
        // Phase B: Set direction pins, then raise step pins for active motors
        bool anyActive = false;

        // Set direction bits based on movement direction
        for (int i = 0; i < 6; i++) {
            int32_t delta = motorTargetPos[i] - motorCurrentPos[i];
            if (delta != 0) {
                bool dir = (delta > 0);
                if (motorInverted[i]) dir = !dir;
                if (dir) {
                    BIT_SET(motorOutputReg, mcpDirPins[i]);
                } else {
                    BIT_CLEAR(motorOutputReg, mcpDirPins[i]);
                }
            }
        }
        // Write direction first (dir setup time provided by SPI transaction gap)
        outputBank->writeAll(motorOutputReg);

        // Now set step pins HIGH for motors that need to move
        uint16_t withStep = motorOutputReg;
        for (int i = 0; i < 6; i++) {
            if (motorCurrentPos[i] < motorTargetPos[i]) {
                motorCurrentPos[i]++;
                BIT_SET(withStep, mcpStepPins[i]);
                anyActive = true;
            } else if (motorCurrentPos[i] > motorTargetPos[i]) {
                motorCurrentPos[i]--;
                BIT_SET(withStep, mcpStepPins[i]);
                anyActive = true;
            }
        }
        if (anyActive) {
            outputBank->writeAll(withStep);
            stepPinState = true;
        }
    }

    xSemaphoreGive(xMutex);
}
#else
void handleStepDirection() {
    // PCBv2: delegate to the active StepDriver backend.
    // SSE:   no-op here — the ISR drives all motors autonomously.
    // MCPWM: starts continuous moves + polls completion.
    xSemaphoreTake(xMutex, portMAX_DELAY);
#if defined(R13_BENCH_ACTIVE)
    if (r13_bench_owns_engine()) {          // armed bench axis drives the engine
        xSemaphoreGive(xMutex);
        return;
    }
#endif
    if (estop_motion_is_blocked()) {
        StepDriver_stop();
        xSemaphoreGive(xMutex);
        return;
    }
    StepDriver_handleStep();
    xSemaphoreGive(xMutex);
}
#endif

static uint32_t binPktCount = 0;

void InterfaceMonitorTask(void * pvParameters) {
    int64_t lastDbg = 0;
    for(;;) {
        // COBS transport: read bytes, decode frames, dispatch to registered handlers
        // (data_handler → serial_packet_handler, cmd_handler → process_data)
        int got = cobs_read_process(5);

        // Debug output runs here (not in GPIOLoopTask) so fflush can't stall motors
        int64_t now = micros();
        if (now - lastDbg >= DEBUG_OUTPUT_INTERVAL) {
            lastDbg = now;
            outputDebugData();
        }

        // Yield when no data — prevents tight busy-loop from starving core 0
        if (got == 0)
            vTaskDelay(1);
    }
}

void GPIOLoopTask(void * pvParameters) {
    esp_task_wdt_add(NULL);

    // Initialize motors on core 1 — keeps motor ISRs away from WiFi/BLE core 0.
    setupMotorPins();
    xGPIOLoopHandle = xTaskGetCurrentTaskHandle();

#if PCB_VERSION == 1
    // PCBv1: GPTimerScheduler fires every MICRO_INTERVAL_FAST µs, notifies this task.
    DEBUG_PRINTLN("GPIOLoop: Initializing GPTimer scheduler");
    if (!GPTimerScheduler::begin(MICRO_INTERVAL_FAST)) {
        DEBUG_PRINTLN("ERROR: GPTimer init failed, falling back to vTaskDelayUntil");
        TickType_t xLastWakeTime = xTaskGetTickCount();
        for (;;) {
            esp_task_wdt_reset();
            currentMicros = micros();
            if (currentMicros - previousMicros >= microInterval) {
                previousMicros = currentMicros;
                handleStepDirection();
            }
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
        }
        return;
    }
    if (!GPTimerScheduler::start(xGPIOLoopHandle)) {
        DEBUG_PRINTLN("ERROR: GPTimer start failed");
        return;
    }
    if (estop_motion_is_blocked()) GPTimerScheduler::stop();
    DEBUG_PRINTLN("GPIOLoop: GPTimer started (PCBv1)");
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        esp_task_wdt_reset();
        currentMicros = micros();
        if (currentMicros - previousMicros >= microInterval) {
            int64_t loopStart = currentMicros;
            previousMicros = currentMicros;
            handleStepDirection();
            uint32_t dur = (uint32_t)(micros() - loopStart);
            loopCount++;
            loopSum_us += dur;
            if (dur < loopMin_us) loopMin_us = dur;
            if (dur > loopMax_us) loopMax_us = dur;
        }
    }

#else
    // PCBv2 / StepDriver backend:
    // SSE:   ISR drives all motors autonomously — this loop just feeds WDT
    //        and calls handleStepDirection() (no-op for SSE) as a keep-alive.
    // MCPWM: handleStepDirection() starts continuous moves + polls done.
    DEBUG_PRINTLN("GPIOLoop: entering StepDriver loop (PCBv2)");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        esp_task_wdt_reset();
        currentMicros = micros();
        if (currentMicros - previousMicros >= microInterval) {
            int64_t loopStart = currentMicros;
            previousMicros = currentMicros;
            handleStepDirection();
            uint32_t dur = (uint32_t)(micros() - loopStart);
            loopCount++;
            loopSum_us += dur;
            if (dur < loopMin_us) loopMin_us = dur;
            if (dur > loopMax_us) loopMax_us = dur;
        }
        // Yield 1ms so InterfaceMonitorTask (core 0) can process serial/commands.
        // SSE ISR continues firing during this yield — zero motion interruption.
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
#endif
}

void EStopMonitorTask(void * pvParameters) {
    // Subscribe this task to the watchdog
    esp_task_wdt_add(NULL);
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    bool prev_state = estop_button.debounced_state;
    // Boot-active NC loop must stop an already initialized engine too.
    if (prev_state == ESTOP_ACTIVE_STATE) {
        isPausedEStop = true;
#if PCB_VERSION == 1
        GPTimerScheduler::stop();
#else
        if (StepDriver_isInitialized()) StepDriver_stop();
#endif
        if (motor_isInit(0)) {
            for (int i = 0; i < 6; i++) motor_emergencyStop(i);
        }
        DEBUG_PRINTLN("E-STOP active at boot - holding");
    }
    
    for(;;) {
        // Update watchdog to indicate E-stop monitoring is alive
        esp_task_wdt_reset();

        // Latch an open NC loop on the raw sample. Debounce is still required
        // before reset, but must not delay stopping an active pulse engine.
        if (gpio_get_level((gpio_num_t)ESTOP_PIN) == ESTOP_ACTIVE_STATE) {
            bool newly_active = !isPausedEStop.exchange(true);
#if PCB_VERSION == 1
            GPTimerScheduler::stop();
#else
            StepDriver_stop();
#endif
            for (int i = 0; i < 6; i++) motor_emergencyStop(i);
#if defined(R13_BENCH_ACTIVE)
            r13_bench_stop("LOOP_OPEN");
#endif
            if (newly_active) DEBUG_PRINTLN("E-STOP raw NC loop opened");
        }
        
        // Update button status through debounce filter
        if (updateDebounceButton(&estop_button)) {
            bool current_state = estop_button.debounced_state;
            
            // Transition INTO the active state (E-stop activated); polarity per ESTOP_ACTIVE_STATE
            if (prev_state != ESTOP_ACTIVE_STATE && current_state == ESTOP_ACTIVE_STATE) {
                // SAFETY: stop motor engine immediately
#if PCB_VERSION == 1
                GPTimerScheduler::stop();
#else
                StepDriver_stop();
#endif
                for (int i = 0; i < 6; i++) motor_emergencyStop(i);
                isPausedEStop = true;
                DEBUG_PRINTLN("E-STOP ACTIVATED");
            }

            // Releasing the switch never restarts motion. An explicit reset
            // must first discard targets and recheck both input states.
            if (prev_state == ESTOP_ACTIVE_STATE && current_state != ESTOP_ACTIVE_STATE) {
                DEBUG_PRINTLN("E-STOP RELEASED - Reset required");
            }
            
            prev_state = current_state;
        }
        
        // Check E-stop state more frequently than debounce time
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ESTOP_CHECK_INTERVAL_MS));
    }
}

// Shared motion-value handler: applies rate limiting, updates arr[], calls setPos()
void applyMotionValues(float values[6]) {
    if (estop_motion_is_blocked()) {
        DEBUG_PRINTLN("Motion paused - E-Stop active");
        return;
    }

    if (isRateLimiting) {
        bool withinLimit = true;
        float limited[6];
        for (int i = 0; i < 6; i++) {
            limited[i] = rateLimit(values[i], arr[i]);
            if (fabs(values[i] - arr[i]) > 0.01f)
                withinLimit = false;
        }
        if (withinLimit) {
            isRateLimiting = false;
            DEBUG_PRINTLN("Rate limiting disabled - within limits");
        }
        memcpy((void*)arr, limited, sizeof(arr));
    } else {
        memcpy((void*)arr, values, sizeof(arr));
    }
    setPos();
}

// ── COBS motion payload decode (CH_DATA18) ──
// Tests:  High-throughput motion data path from SimTools/dashboard
// Proves: COBS channel dispatch + payload decode works,
//         mapRawToPosition() axis scaling, processMotionCueing() biquad
//         filter chain, applyMotionValues() rate limiter → setPos() pipeline,
//         full data path from USB RX → IK → MCPWM at ~30-60 Hz packet rate
void send_telemetry(const float angles[6]) {
    // Binary telemetry via COBS TEL channel: 12 x float32 LE = 48 bytes
    // [angles[0..5], positions[0..5]] — no ASCII parsing, no sscanf
    cobs_send_telemetry(angles, (const float*)arr);
}

void process_binary_packet(const uint8_t *payload, int len) {
    float raw[6], mapped[6];
    if (!controller::decode_motion_payload(payload, len, raw)) return;
    binPktCount++;

    mapRawToPosition(raw, &axisScales, maxRawInput, mapped);
    // No MCA here — the SIL server already applies motion cueing before
    // encoding raw values. Applying it again causes double-filtering and
    // filter instability at low packet rates.
    applyMotionValues(mapped);
}

// ASCII command parser (fed from COBS CH_CMD)
void process_data(char * data) {
    // These commands can create motion or change its position reference.
    // Keep them out of a physical/software E-stop regardless of transport.
    if (estop_motion_is_blocked() &&
        (strncmp(data, "RATETEST", 8) == 0 ||
         strncmp(data, "FREQTEST:", 9) == 0 ||
         strcmp(data, "PINTEST") == 0 ||
         strncmp(data, "SIGTEST:", 8) == 0 ||
         strcmp(data, "MTEST") == 0 ||
         (strncmp(data, "MTEST:", 6) == 0 && strcmp(data, "MTEST:RESET") != 0) ||
         strcmp(data, "ESTOP:SOFT") == 0 ||
         strcmp(data, "ZERO") == 0)) {
        serial_printf("ERROR:ESTOP_ACTIVE\r\n");
        return;
    }
    // ── DBG:1 / DBG:0 — Toggle verbose debug output ──────────────────
    // Tests:  Serial RX command parsing, bidirectional USB Serial JTAG link
    // Proves: VFS stdin→cobs_read_process pipeline, printf→USB TX path,
    //         debug_uart.h macro gating (DEBUG_PRINTLN only emits when enabled)
    if (strcmp(data, DEBUG_ENABLE_CMD) == 0) {
        debugEnabled = true;
        DEBUG_PRINTLN("Debug output enabled");
        return;
    } else if (strcmp(data, DEBUG_DISABLE_CMD) == 0) {
        debugEnabled = false;
        DEBUG_PRINTLN("Debug output disabled");
        return;
    }

    // ── HIGH-PRIORITY: Handshake commands ──────────────────────────────
    // These must be at the top so the app's handshake completes instantly.
    // Any delay here blocks motion — the app won't send packets until
    // the handshake reaches Ready.

    const controller::HandshakeIdentity identity = {
        [](uint8_t* mac) { esp_efuse_mac_get_default(mac); },
        FW_VERSION_STRING, FW_PROTOCOL_VERSION, FW_BUILD_DATE, FW_BUILD_TIME
    };
    if (controller::handshake_query(data, stewartConfig, inputBitRange, maxRawInput,
                                    identity, serial_printf)) return;

    // ── Input source selection ───────────────────────────────────────
    if (strcmp(data, "INPUT?") == 0) {
        serial_printf("INPUT:%s\r\n", inputSourceName(activeInputSource));
        return;
    }
#ifdef ENABLE_ETHERNET
    // PEERMAC:aa:bb:cc:dd:ee:ff — the host's own MAC, so replies can use the W5500's SEND_MAC
    // (no ARP). Needed for hosts on Wi-Fi that never answer the board's ARP requests.
    if (strncmp(data, "PEERMAC:", 8) == 0) {
        unsigned m[6];
        if (sscanf(data + 8, "%x:%x:%x:%x:%x:%x", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) == 6) {
            uint8_t mac[6];
            for (int i = 0; i < 6; i++) mac[i] = (uint8_t)m[i];
            ethernet_transport_set_peer_mac(mac);
            serial_printf("PEERMAC:%02x:%02x:%02x:%02x:%02x:%02x\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        } else {
            serial_printf("PEERMAC:ERR\r\n");
        }
        return;
    }
#endif
    if (strcmp(data, "INPUT_STAT") == 0) {
        cobs_motion_stats_t ms;
        cobs_motion_stats(&ms);
        serial_printf("INPUT_STAT:active=%s,serial=%lu/%lu,eth=%lu/%lu,wifi=%lu/%lu,ble=%lu/%lu,seq=%lu,gaps=%lu,lost=%lu,age_ms=%lu\r\n",
            inputSourceName(activeInputSource),
            (unsigned long)pktCount_serial, (unsigned long)pktDrop_serial,
            (unsigned long)pktCount_eth, (unsigned long)pktDrop_eth,
            (unsigned long)pktCount_wifi, (unsigned long)pktDrop_wifi,
            (unsigned long)pktCount_ble, (unsigned long)pktDrop_ble,
            (unsigned long)ms.seq_frames, (unsigned long)ms.seq_gaps, (unsigned long)ms.seq_lost,
            (unsigned long)ms.last_rx_age_ms);
#ifdef ENABLE_ETHERNET
        eth_stats_t es;
        ethernet_transport_stats(&es);
        serial_printf("ETH_STAT:link=%d,sock=%d,ip=%u.%u.%u.%u,rx=%lu,bad=%lu,tx=%lu,txfail=%lu,reopens=%lu,age_ms=%lu,peer=%u.%u.%u.%u:%u,polls=%lu,rxmax_us=%lu,txmax_us=%lu\r\n",
            es.link ? 1 : 0, es.sock_open ? 1 : 0,
            (unsigned)(es.ip & 0xff), (unsigned)((es.ip >> 8) & 0xff), (unsigned)((es.ip >> 16) & 0xff), (unsigned)((es.ip >> 24) & 0xff),
            (unsigned long)es.rx_frames, (unsigned long)es.rx_bad, (unsigned long)es.tx_frames, (unsigned long)es.tx_fail,
            (unsigned long)es.sock_reopens, (unsigned long)es.rx_age_ms,
            es.peer_ip[0], es.peer_ip[1], es.peer_ip[2], es.peer_ip[3], es.peer_port,
            (unsigned long)es.polls, (unsigned long)es.rx_max_us, (unsigned long)es.tx_max_us);
        serial_printf("SPI_STAT:calls=%lu,eth_calls=%lu,wait_max_us=%lu,xfer_max_us=%lu,wait_sum_us=%lu,slow=%lu,phycfgr=0x%02x,lwip_stopped=%d\r\n",
            (unsigned long)es.spi_calls, (unsigned long)es.eth_spi_calls, (unsigned long)es.spi_wait_max_us,
            (unsigned long)es.spi_xfer_max_us, (unsigned long)es.spi_wait_sum_us, (unsigned long)es.spi_slow,
            es.phycfgr, es.lwip_stopped ? 1 : 0);
        serial_printf("TX_STAT:timeout=%lu,dropped=%lu,pending=%d,sn_ir=0x%02x,dhar=%02x:%02x:%02x:%02x:%02x:%02x\r\n",
            (unsigned long)es.tx_timeout, (unsigned long)es.tx_dropped, es.send_pending ? 1 : 0, es.sn_ir,
            es.dhar[0], es.dhar[1], es.dhar[2], es.dhar[3], es.dhar[4], es.dhar[5]);
#endif
        return;
    }
    if (strncmp(data, "INPUT:", 6) == 0) {
        const char* src = data + 6;
        InputSource prev = activeInputSource;
        if (strcmp(src, "SERIAL") == 0)        activeInputSource = INPUT_SOURCE_SERIAL;
        else if (strcmp(src, "ETHERNET") == 0) activeInputSource = INPUT_SOURCE_ETHERNET;
        else if (strcmp(src, "WIFI") == 0)     activeInputSource = INPUT_SOURCE_WIFI;
        else if (strcmp(src, "BLE") == 0)      activeInputSource = INPUT_SOURCE_BLE;
        else {
            serial_printf("ERROR:INPUT unknown source '%s'\r\n", src);
            return;
        }
        serial_printf("INPUT:%s\r\n", inputSourceName(activeInputSource));
        if (activeInputSource != prev) {
            ESP_LOGI(TAG, "Input source changed: %s -> %s",
                inputSourceName(prev), inputSourceName(activeInputSource));
            // Save to NVS
            nvs_handle_t nvs;
            if (nvs_open("stewart", NVS_READWRITE, &nvs) == ESP_OK) {
                nvs_set_u8(nvs, "input_src", (uint8_t)activeInputSource);
                nvs_commit(nvs);
                nvs_close(nvs);
            }
        }
        return;
    }

    // ── Motor timing diagnostics ──────────────────────────────────────

    // ── TICKRATE? — Query current ISR tick period and derived rates ────
    // Response: TICKRATE:<us>,hz=<tick_hz>,max_step=<max_hz>
    if (strcmp(data, "TICKRATE?") == 0) {
#if !defined(STEP_DRIVER_MCPWM)
        serial_printf("TICKRATE:%lu,hz=%lu,max_step=%lu\r\n",
            (unsigned long)StepDriver_getTickUs(),
            (unsigned long)(1000000UL / StepDriver_getTickUs()),
            (unsigned long)StepDriver_getMaxStepHz());
#else
        serial_printf("TICKRATE:4,hz=250000,max_step=250000 (hardware-fixed)\r\n");
#endif
        return;
    }

    // ── TICKRATE:N — Set ISR tick period to N microseconds ────────────
    // Valid range: 4–100 µs (250 kHz … 5 kHz tick rate).
    // 4µs is the hard WDT-safe floor on ESP32-S3.
    // Change takes effect immediately (hot-reconfigure, no motor stop needed).
    // Persisted to NVS — survives reboot.
    // Response: TICKRATE:<us>,hz=<tick_hz>,max_step=<max_hz>
    //           TICKRATE:ERR:<reason>
    if (strncmp(data, "TICKRATE:", 9) == 0 && data[9] != '?') {
#if !defined(STEP_DRIVER_MCPWM)
        int us = atoi(data + 9);
        if (us < 4 || us > 100) {
            serial_printf("TICKRATE:ERR range 4-100 us (requested %d)\r\n", us);
            return;
        }
        if (!StepDriver_setTickRate((uint32_t)us)) {
            serial_printf("TICKRATE:ERR setTickRate failed (requested %d)\r\n", us);
            return;
        }
        serial_printf("TICKRATE:%lu,hz=%lu,max_step=%lu\r\n",
            (unsigned long)StepDriver_getTickUs(),
            (unsigned long)(1000000UL / StepDriver_getTickUs()),
            (unsigned long)StepDriver_getMaxStepHz());
        // Persist to NVS
        nvs_handle_t nvs;
        if (nvs_open("stewart", NVS_READWRITE, &nvs) == ESP_OK) {
            nvs_set_u8(nvs, "tick_us", (uint8_t)us);
            nvs_commit(nvs);
            nvs_close(nvs);
        }
#else
        serial_printf("TICKRATE:ERR MCPWM backend uses hardware-fixed 250 kHz rate\r\n");
#endif
        return;
    }

    // ── MSTAT — Report loop timing + per-motor step statistics ────────
    // Tests:  GPIOLoopTask scheduling, MCPWM motor driver state
    // Proves: FreeRTOS task is running (loopCount > 0),
    //         GPTimer ISR fires at correct interval (loop_us ~ microInterval),
    //         all 6 MCPWMMotorControl instances initialized (init=1),
    //         step timing jitter within bounds (min/avg/max step_us)
    if (strcmp(data, "MSTAT") == 0) {
        // Loop timing
        uint32_t cnt = loopCount;
        float avg = cnt > 0 ? (float)loopSum_us / cnt : 0;
        serial_printf("MSTAT:loop_count=%lu,loop_us(min/avg/max)=%lu/%.1f/%lu,interval=%d\r\n",
            cnt, (unsigned long)loopMin_us, avg, (unsigned long)loopMax_us, microInterval);
        // Per-motor stats — backend-specific detail via StepDriver_report()
        for (int i = 0; i < 6; i++) {
            serial_printf("  M%d: pos=%ld tgt=%ld init=%d\r\n",
                i, (long)motor_getPos(i), (long)motor_getTarget(i),
                motor_isInit(i) ? 1 : 0);
        }
#if PCB_VERSION != 1
        StepDriver_report();
#endif
        return;
    }

    // ── RATETEST:STEPS[:MOTORS] — Measure actual MCPWM stepping rate ──
    // Two modes: BURST (one-shot round-robin) and CONTINUOUS (hardware free-run).
    // MOTORS: 1 = single motor, 6 = all (default 6).
    if (strncmp(data, "RATETEST", 8) == 0) {
#if defined(STEP_DRIVER_MCPWM)
        int test_steps = 50000;
        int num_motors = 6;
        if (data[8] == ':') {
            sscanf(data + 9, "%d:%d", &test_steps, &num_motors);
        }
        if (test_steps < 100 || test_steps > 1000000) {
            serial_printf("RATETEST:ERR steps=100-1000000\r\n");
            return;
        }
        if (num_motors < 1 || num_motors > 6) num_motors = 6;

        uint32_t period_us = 4;  // SSE tick * 2; MCPWM: stepPulseWidth_us * 2
        float theoretical_max = 1000000.0f / period_us;

        serial_printf("RATETEST:START steps=%d motors=%d period=%lu us (theoretical_max=%.0f Hz)\r\n",
            test_steps, num_motors, (unsigned long)period_us, theoretical_max);

        // ── Test 1: Pipeline test (set target → GPTimer → continuous MCPWM) ──
        // Oscillates within soft limits to accumulate the requested step count.
        // Queries each motor's limit range, swings between endpoints, stays safe.
        {
            int32_t savedPos[6];
            for (int i = 0; i < 6; i++) {
                savedPos[i] = motor_getPos(i);
            }

            int32_t limit_min = -100000;
            int32_t limit_max =  100000;
            int32_t swing = (limit_max - limit_min);  // full range per leg
            if (swing < 1000) swing = 1000;

            int legs = (test_steps + swing - 1) / swing;
            if (legs < 1) legs = 1;

            serial_printf("RATETEST:PIPELINE range=[%ld,%ld] swing=%ld legs=%d\r\n",
                (long)limit_min, (long)limit_max, (long)swing, legs);

            // Move to starting end (limit_min) before timing begins
            xSemaphoreTake(xMutex, portMAX_DELAY);
            for (int i = 0; i < 6; i++) {
                if (i >= num_motors) continue;
                motor_setTarget(i, limit_min);
            }
            xSemaphoreGive(xMutex);
            {
                bool setup_done = false;
                while (!setup_done) {
                    vTaskDelay(pdMS_TO_TICKS(1));
                    setup_done = true;
                    for (int i = 0; i < 6; i++) {
                        if (i >= num_motors) continue;
                        if (motor_getTarget(i) != motor_getPos(i))
                            { setup_done = false; break; }
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10)); // brief settle

            // Now oscillate — every leg is a full swing
            int32_t total_steps_actual = 0;
            int64_t t_start = esp_timer_get_time();
            float timeout_sec = (float)test_steps / 1000.0f + 15.0f;
            bool timed_out = false;

            for (int leg = 0; leg < legs && !timed_out; leg++) {
                // Alternate: odd legs → max, even legs → min
                int32_t target = (leg % 2 == 0) ? limit_max : limit_min;

                xSemaphoreTake(xMutex, portMAX_DELAY);
                for (int i = 0; i < 6; i++) {
                    if (i >= num_motors) continue;
                    motor_setTarget(i, target);
                }
                xSemaphoreGive(xMutex);

                bool done = false;
                while (!done) {
                    vTaskDelay(pdMS_TO_TICKS(1));
                    done = true;
                    for (int i = 0; i < 6; i++) {
                        if (i >= num_motors) continue;
                        if (motor_getTarget(i) != motor_getPos(i)) {
                            done = false; break;
                        }
                    }
                    if ((esp_timer_get_time() - t_start) > (int64_t)(timeout_sec * 1000000.0f)) {
                        timed_out = true; break;
                    }
                }
                total_steps_actual += swing;
            }

            int64_t t_pipe = esp_timer_get_time() - t_start;
            float pipe_rate = (t_pipe > 0)
                ? (float)total_steps_actual / ((float)t_pipe / 1000000.0f) : 0.0f;

            serial_printf("RATETEST:PIPELINE %d steps in %lld us (%.0f steps/s/motor, %.1f%% of max)%s\r\n",
                total_steps_actual, t_pipe, pipe_rate, pipe_rate / theoretical_max * 100.0f,
                timed_out ? " TIMEOUT" : "");
            for (int i = 0; i < 6; i++) {
                if (i >= num_motors) continue;
                serial_printf("  M%d: pos=%ld (target was %ld, error=%ld)\r\n",
                    i, (long)motor_getPos(i),
                    (long)motor_getTarget(i),
                    (long)(motor_getPos(i) - motor_getTarget(i)));
            }

            // Return to starting position
            xSemaphoreTake(xMutex, portMAX_DELAY);
            for (int i = 0; i < 6; i++) {
                if (i >= num_motors) continue;
                motor_setTarget(i, savedPos[i]);
            }
            xSemaphoreGive(xMutex);
            bool done = false;
            while (!done) {
                vTaskDelay(pdMS_TO_TICKS(1));
                done = true;
                for (int i = 0; i < 6; i++) {
                    if (i >= num_motors) continue;
                    if (motor_getTarget(i) != motor_getPos(i)) {
                        done = false; break;
                    }
                }
                if ((esp_timer_get_time() - t_start) > (int64_t)(timeout_sec * 2.0f * 1000000.0f)) {
                    break;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // settle

        // ── Test 2: Hardware-counted continuous mode (all motors, 250 kHz) ──
        // Motors 0-3: MCPWM + PCNT, Motors 4-5: RMT TX loop counting.
        // All hardware-counted, zero CPU per pulse.
        {
            // Temporarily remove GPIOLoopTask from WDT — it will be blocked
            // on the mutex for the duration of this test, unable to feed WDT.
            if (xGPIOLoopHandle) esp_task_wdt_delete(xGPIOLoopHandle);

            xSemaphoreTake(xMutex, portMAX_DELAY);

            int64_t t_start = esp_timer_get_time();
            for (int i = 0; i < num_motors; i++) {
                _mcpwm_motors[i]->startContinuousSteps(test_steps);
            }

            bool anyRunning = true;
            while (anyRunning) {
                anyRunning = false;
                for (int i = 0; i < num_motors; i++) {
                    if (!_mcpwm_motors[i]->checkContinuousDone()) anyRunning = true;
                }
            }

            int64_t t_cont = esp_timer_get_time() - t_start;
            float cont_rate = (float)test_steps / ((float)t_cont / 1000000.0f);

            serial_printf("RATETEST:CONTINUOUS %d steps in %lld us (%.0f steps/s/motor, %.1f%% of max)\r\n",
                test_steps, t_cont, cont_rate, cont_rate / theoretical_max * 100.0f);
            for (int i = 0; i < num_motors; i++) {
                serial_printf("  M%d: error=%ld steps (%s)\r\n",
                    i, (long)_mcpwm_motors[i]->getLastStepError(),
                    _mcpwm_motors[i]->hasPcnt() ? "PCNT" : (_mcpwm_motors[i]->hasRmt() ? "RMT" : "ISR"));
            }

            // Return via continuous mode (reverse)
            for (int i = 0; i < num_motors; i++) {
                _mcpwm_motors[i]->startContinuousSteps(-test_steps);
            }
            anyRunning = true;
            while (anyRunning) {
                anyRunning = false;
                for (int i = 0; i < num_motors; i++) {
                    if (!_mcpwm_motors[i]->checkContinuousDone()) anyRunning = true;
                }
            }

            xSemaphoreGive(xMutex);

            // Re-subscribe GPIOLoopTask to WDT now that mutex is released
            if (xGPIOLoopHandle) esp_task_wdt_add(xGPIOLoopHandle);
        }

        serial_printf("RATETEST:DONE motors=%d\r\n", num_motors);
#else
        serial_printf("RATETEST:NOT_SUPPORTED\r\n");
#endif
        return;
    }

    // ── FREQTEST:M:F:D — Generate step pulses at exact frequency ───────
    // M=motor(0-5), F=freq_hz, D=duration_ms
    // Uses direct GPIO toggle for precise frequency control.
    // NOTE: Takes STEP pin from MCPWM — reboot after.
    if (strncmp(data, "FREQTEST:", 9) == 0) {
#if PCB_VERSION == 2
        int motor = 0, freq_hz = 0, dur_ms = 1000;
        if (sscanf(data + 9, "%d:%d:%d", &motor, &freq_hz, &dur_ms) < 2) {
            serial_printf("FREQTEST:ERR usage FREQTEST:M:F[:D]\r\n");
            return;
        }
        if (motor < 0 || motor > 5 || freq_hz < 1 || freq_hz > 300000) {
            serial_printf("FREQTEST:ERR motor=0-5 freq=1-300000\r\n");
            return;
        }

        // Take STEP pin from MCPWM
        gpio_reset_pin(stepPins[motor]);
        gpio_set_direction(stepPins[motor], GPIO_MODE_OUTPUT);
        gpio_set_level(stepPins[motor], 0);

        // Set DIR HIGH so analyzer can distinguish real vs crosstalk
        gpio_set_level(dirPins[motor], 1);

        int64_t period_us = 1000000LL / freq_hz;
        int64_t pulse_us = 2; // 2µs pulse width (minimum for stepper drivers)
        if (period_us < pulse_us * 2) period_us = pulse_us * 2;
        int64_t low_us = period_us - pulse_us;

        int64_t end_time = esp_timer_get_time() + (int64_t)dur_ms * 1000;
        int32_t step_count = 0;

        serial_printf("FREQTEST:START motor=%d freq=%d dur=%d period_us=%lld\r\n",
            motor, freq_hz, dur_ms, period_us);

        // Pulse generation — busy-wait for accuracy at high frequencies
        while (esp_timer_get_time() < end_time && !estop_motion_is_blocked()) {
            gpio_set_level(stepPins[motor], 1);
            int64_t t0 = esp_timer_get_time();
            while ((esp_timer_get_time() - t0) < pulse_us) {}
            gpio_set_level(stepPins[motor], 0);
            step_count++;
            int64_t t1 = esp_timer_get_time();
            int64_t remain = low_us - (t1 - t0 - pulse_us);
            if (remain > 1000) {
                vTaskDelay(pdMS_TO_TICKS(remain / 1000));
            } else if (remain > 0) {
                while ((esp_timer_get_time() - t1) < remain) {}
            }
        }

        gpio_set_level(dirPins[motor], 0);

        int64_t actual_us = dur_ms * 1000LL;
        float actual_freq = step_count * 1000000.0f / actual_us;
        serial_printf("FREQTEST:DONE motor=%d steps=%ld actual_freq=%.1f\r\n",
            motor, (long)step_count, actual_freq);
#else
        serial_printf("FREQTEST:NOT_SUPPORTED\r\n");
#endif
        return;
    }

    // ── PINTEST — Static pin-by-pin HIGH/LOW for wiring validation ────
    // Sets each DIR and STEP pin HIGH individually with 500ms pause,
    // allowing analyzer PINS command to verify correct mapping.
    // NOTE: STEP pins are temporarily taken from MCPWM — reboot after.
    if (strcmp(data, "PINTEST") == 0) {
#if PCB_VERSION == 2
        serial_printf("PINTEST:START\r\n");
        // Test DIR pins (already simple GPIO outputs)
        for (int i = 0; i < 6; i++) {
            gpio_set_level(dirPins[i], 1);
            serial_printf("PINTEST:DIR_%d:GPIO%d:ON\r\n", i, dirPins[i]);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(dirPins[i], 0);
            serial_printf("PINTEST:DIR_%d:OFF\r\n", i);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        // Test STEP pins (take from MCPWM, use as GPIO)
        for (int i = 0; i < 6; i++) {
            if (estop_motion_is_blocked()) break;
            gpio_reset_pin(stepPins[i]);
            gpio_set_direction(stepPins[i], GPIO_MODE_OUTPUT);
            gpio_set_level(stepPins[i], 1);
            serial_printf("PINTEST:STEP_%d:GPIO%d:ON\r\n", i, stepPins[i]);
            for (int ms = 0; ms < 500 && !estop_motion_is_blocked(); ms += 10)
                vTaskDelay(pdMS_TO_TICKS(10));
            gpio_set_level(stepPins[i], 0);
            serial_printf("PINTEST:STEP_%d:OFF\r\n", i);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        serial_printf("PINTEST:DONE (reboot recommended)\r\n");
#else
        serial_printf("PINTEST:NOT_SUPPORTED\r\n");
#endif
        return;
    }

    // ── SIGTEST:M:N:R:D — Logic analyzer signal validation ────────────
    // Fire exactly N steps on motor M at R Hz in direction D (0=neg,1=pos).
    // Reports: commanded count, hardware-counted result, pulse timing stats.
    // Designed for logic analyzer capture: probe STEP + DIR pins on motor M.
    // Usage: SIGTEST:0:1000:250000:1  (motor 0, 1000 steps, 250kHz, forward)
    //        SIGTEST:0:500:125000:0   (motor 0, 500 steps, 125kHz, reverse)
    // After SIGTEST:DONE, SIGTEST:RETURN fires the same count in reverse.
    if (strncmp(data, "SIGTEST:", 8) == 0) {
#if defined(STEP_DRIVER_MCPWM) || defined(STEP_DRIVER_ISR_BACKEND)
        int motor = 0, dir = 1;
        int32_t steps = 1000;
        int32_t rate_hz = 250000;
        int parsed = sscanf(data + 8, "%d:%ld:%ld:%d", &motor, &steps, &rate_hz, &dir);
        if (parsed < 2 || motor < 0 || motor > 5 || steps < 1 || steps > 500000
                       || rate_hz < 1 || rate_hz > 250000) {
            serial_printf("SIGTEST:ERR usage SIGTEST:M:N[:R[:D]]  M=0-5 N=steps R=hz D=0/1\r\n");
            return;
        }

        // Verify backend is initialized
        if (!StepDriver_isInitialized()) {
            serial_printf("SIGTEST:ERR stepper backend not initialized\r\n");
            return;
        }

        // Reset stats so we get a clean count for this run
        StepDriver_resetStats(motor);

        int32_t start_pos = StepDriver_getPosition(motor);
        int32_t target    = start_pos + (dir ? steps : -steps);

        // Clamp to soft limits — report if clamped
        if (estop_motion_is_blocked() || !StepDriver_setTarget(motor, target)) {
            serial_printf("SIGTEST:ERR stopped or target outside limits\r\n");
            return;
        }
        int32_t actual_target = StepDriver_getTarget(motor);
        int32_t commanded = abs(actual_target - start_pos);

        serial_printf("SIGTEST:START motor=%d steps=%ld rate=%ld dir=%d gpio_step=%d gpio_dir=%d\r\n",
            motor, (long)commanded, (long)rate_hz,
            dir, (int)stepPins[motor], (int)dirPins[motor]);

        // Wait for motor to finish, timeout 2× expected duration
        int64_t expected_us = (int64_t)commanded * 1000000LL / rate_hz;
        int64_t timeout_us  = expected_us * 2 + 500000; // +500ms margin
        int64_t t_start = esp_timer_get_time();

        while (StepDriver_getPosition(motor) != actual_target) {
            if ((esp_timer_get_time() - t_start) > timeout_us) {
                serial_printf("SIGTEST:TIMEOUT after %lld us\r\n",
                    esp_timer_get_time() - t_start);
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        int64_t elapsed_us = esp_timer_get_time() - t_start;
        int32_t hw_steps   = (int32_t)StepDriver_getTotalSteps(motor);
        int32_t pos_error  = (int32_t)(StepDriver_getPosition(motor) - actual_target);
        float   actual_hz  = (elapsed_us > 0)
                             ? (float)hw_steps * 1000000.0f / (float)elapsed_us
                             : 0.0f;

        serial_printf("SIGTEST:DONE motor=%d commanded=%ld hw_counted=%ld pos_error=%ld "
                      "elapsed_us=%lld actual_hz=%.0f %s\r\n",
            motor, (long)commanded, (long)hw_steps, (long)pos_error,
            elapsed_us, actual_hz,
            (pos_error == 0 && hw_steps == commanded) ? "PASS" : "FAIL");

        // Return to start
        if (!estop_motion_is_blocked()) StepDriver_setTarget(motor, start_pos);
        t_start = esp_timer_get_time();
        while (StepDriver_getPosition(motor) != start_pos) {
            if ((esp_timer_get_time() - t_start) > timeout_us) break;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        serial_printf("SIGTEST:RETURN motor=%d pos=%ld\r\n",
            motor, (long)StepDriver_getPosition(motor));
#else
        serial_printf("SIGTEST:NOT_SUPPORTED (requires STEP_DRIVER_MCPWM or STEP_DRIVER_SHARED_MCPWM)\r\n");
#endif
        return;
    }

    // ── MTEST:RESET — Clear all timing/step statistics ────────────────
    // Tests:  Stats reset path for loop counters and per-motor accumulators
    // Proves: resetStats() zeroes all MCPWMMotorControl::Stats fields,
    //         loop timing counters reset cleanly (no stale data in MSTAT)
    if (strcmp(data, "MTEST:RESET") == 0) {
        loopCount = 0; loopMin_us = UINT32_MAX; loopMax_us = 0; loopSum_us = 0;
#if defined(STEP_DRIVER_MCPWM)
        for (int i = 0; i < 6; i++) {
            if (_mcpwm_motors[i]) _mcpwm_motors[i]->resetStats();
        }
#endif
        serial_printf("MTEST:RESET=OK\r\n");
        return;
    }

    // ── MTEST / MTEST:N — Motor self-test: 200 steps forward, 200 back ─
    // MTEST   = test all 6 motors simultaneously
    // MTEST:N = test single motor N (0-5) for incremental wiring validation
    // Tests:  Full MCPWM stepper drive pipeline, end-to-end
    // Proves: (1) MCPWM timer→operator→comparator→generator chain fires pulses
    //         (2) setTargetPosition() / getCurrentPosition() track correctly
    //         (3) Motors return to exact start position (no lost steps)
    //         (4) Step rate throughput (steps/s) meets real-time requirements
    //         (5) Mutex arbitration between InterfaceMonitorTask and GPIOLoopTask
    //         (6) Loop timing during load (MTEST:LOOP min/avg/max)
    //         (7) PASS/FAIL verdict: position error = 0
    if (strcmp(data, "MTEST") == 0 || (strncmp(data, "MTEST:", 6) == 0 && data[6] >= '0' && data[6] <= '5')) {
        int testMotor = -1; // -1 = all motors
        if (data[5] == ':') testMotor = data[6] - '0';
        if (motorTestRunning) {
            serial_printf("MTEST:BUSY\r\n");
            return;
        }
        motorTestRunning = true;
        const int TEST_STEPS = 200;
        if (testMotor >= 0)
            serial_printf("MTEST:START motor=%d steps=%d\r\n", testMotor, TEST_STEPS);
        else
            serial_printf("MTEST:START steps=%d\r\n", TEST_STEPS);

        // Save current positions
        int32_t savedPos[6];
        for (int i = 0; i < 6; i++) {
            savedPos[i] = motor_getPos(i);
        }

        // Set forward targets, THEN reset stats so setup overhead is excluded
        xSemaphoreTake(xMutex, portMAX_DELAY);
        for (int i = 0; i < 6; i++) {
            if (testMotor >= 0 && i != testMotor) continue;
            motor_setTarget(i, savedPos[i] + TEST_STEPS);
        }
        loopCount = 0; loopMin_us = UINT32_MAX; loopMax_us = 0; loopSum_us = 0;
#if defined(STEP_DRIVER_MCPWM)
        for (int i = 0; i < 6; i++) {
            if (_mcpwm_motors[i]) _mcpwm_motors[i]->resetStats();
        }
#endif
        xSemaphoreGive(xMutex);

        // Wait for forward motion to complete (timeout 2s)
        int64_t t0 = micros();
        bool done = false;
        while (!done && (micros() - t0) < 2000000) {
            done = true;
            for (int i = 0; i < 6; i++) {
                if (testMotor >= 0 && i != testMotor) continue;
                if (motor_getPos(i) != motor_getTarget(i))
                    done = false;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        int64_t fwd_us = micros() - t0;

        // Capture forward stats before resetting for back phase
        uint32_t fwd_loops = loopCount;
        float fwd_avg = fwd_loops > 0 ? (float)loopSum_us / fwd_loops : 0;
        uint32_t fwd_loop_min = loopMin_us, fwd_loop_max = loopMax_us;

        // Set back targets, then reset stats for back phase
        xSemaphoreTake(xMutex, portMAX_DELAY);
        for (int i = 0; i < 6; i++) {
            if (testMotor >= 0 && i != testMotor) continue;
            motor_setTarget(i, savedPos[i]);
        }
        loopCount = 0; loopMin_us = UINT32_MAX; loopMax_us = 0; loopSum_us = 0;
#if defined(STEP_DRIVER_MCPWM)
        for (int i = 0; i < 6; i++) {
            if (_mcpwm_motors[i]) _mcpwm_motors[i]->resetStats();
        }
#endif
        xSemaphoreGive(xMutex);

        t0 = micros();
        done = false;
        while (!done && (micros() - t0) < 2000000) {
            done = true;
            for (int i = 0; i < 6; i++) {
                if (testMotor >= 0 && i != testMotor) continue;
                if (motor_getPos(i) != motor_getTarget(i))
                    done = false;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        int64_t back_us = micros() - t0;

        // Report timing
        serial_printf("MTEST:FWD %d steps in %lld us (%.1f steps/s)\r\n",
            TEST_STEPS, fwd_us, TEST_STEPS * 1000000.0f / fwd_us);
        serial_printf("MTEST:FWD_LOOP loops=%lu dur_us(min/avg/max)=%lu/%.1f/%lu\r\n",
            (unsigned long)fwd_loops, (unsigned long)fwd_loop_min, fwd_avg, (unsigned long)fwd_loop_max);
        serial_printf("MTEST:BACK %d steps in %lld us (%.1f steps/s)\r\n",
            TEST_STEPS, back_us, TEST_STEPS * 1000000.0f / back_us);
        uint32_t back_loops = loopCount;
        float back_avg = back_loops > 0 ? (float)loopSum_us / back_loops : 0;
        serial_printf("MTEST:BACK_LOOP loops=%lu dur_us(min/avg/max)=%lu/%.1f/%lu\r\n",
            (unsigned long)back_loops, (unsigned long)loopMin_us, back_avg, (unsigned long)loopMax_us);

        // Check motors returned to original position
        bool allOk = true;
        for (int i = 0; i < 6; i++) {
            if (testMotor >= 0 && i != testMotor) continue;
            int32_t diff = motor_getPos(i) - savedPos[i];
            if (diff != 0) {
                serial_printf("MTEST:M%d POSITION ERROR off=%ld\r\n", i, (long)diff);
                allOk = false;
            }
        }

        // Per-motor step stats (back phase only — most recent clean window)
#if PCB_VERSION == 1
        for (int i = 0; i < 6; i++) {
            if (testMotor >= 0 && i != testMotor) continue;
            serial_printf("  M%d: pos=%ld tgt=%ld\r\n",
                i, (long)motor_getPos(i), (long)motor_getTarget(i));
        }
#elif defined(STEP_DRIVER_MCPWM)
        for (int i = 0; i < 6; i++) {
            if (testMotor >= 0 && i != testMotor) continue;
            if (!_mcpwm_motors[i]) continue;
            const MCPWMMotorControl::Stats& s = _mcpwm_motors[i]->getStats();
            float savg = s.intervalSamples > 0 ? (float)s.sumInterval_us / s.intervalSamples : 0;
            uint32_t smin = s.minInterval_us == UINT32_MAX ? 0 : s.minInterval_us;
            serial_printf("  M%d: steps=%lu errs=%lu step_us(min/avg/max)=%lu/%.1f/%lu\r\n",
                i, (unsigned long)s.totalSteps, (unsigned long)s.stepErrors,
                (unsigned long)smin, savg, (unsigned long)s.maxInterval_us);
        }
#else
        // SSE / MCPWM_ISR: report via StepDriver API (backend-agnostic)
        for (int i = 0; i < 6; i++) {
            if (testMotor >= 0 && i != testMotor) continue;
            serial_printf("  M%d: pos=%ld tgt=%ld steps=%lu\r\n",
                i, (long)motor_getPos(i), (long)motor_getTarget(i),
                (unsigned long)StepDriver_getTotalSteps(i));
        }
#endif

        serial_printf("MTEST:%s\r\n", allOk ? "PASS" : "FAIL");
        motorTestRunning = false;
        return;
    }

    // ── LED:R,G,B — Set entity RGB color on status LED ─────────────────
    // App sends this after handshake to show the entity's UI color.
    // LED breathes in this color until an error/warning overrides it.
    if (strncmp(data, "LED:", 4) == 0) {
        int r = 0, g = 0, b = 0;
        if (sscanf(data + 4, "%d,%d,%d", &r, &g, &b) == 3) {
            led_status_set_entity_color((uint8_t)r, (uint8_t)g, (uint8_t)b);
            serial_printf("LED:%d,%d,%d\r\n", r, g, b);
        } else {
            serial_printf("ERR:LED format LED:R,G,B (0-255)\r\n");
        }
        return;
    }

    // ── E-Stop serial commands ─────────────────────────────────────────

    // ── ESTOP:SOFT — Graceful stop: return all motors to home position ──
    // Motors smoothly travel to center. GPTimer keeps running.
    // Use when you want to safely park the platform without a hard cut.
    if (strcmp(data, "ESTOP:SOFT") == 0) {
#if PCB_VERSION == 2
        // R13/R14 have no qualified home reference in this firmware. A
        // "soft stop" must never command an unreferenced arm toward zero.
        serial_printf("ESTOP:SOFT:ERR no qualified home; use ESTOP:FULL\r\n");
        return;
#else
        // Set all motor targets to home (position 0 = center)
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            for (int i = 0; i < 6; i++) {
                arr[i] = 0;
                motor_setTarget(i, 0);
            }
            xSemaphoreGive(xMutex);
        }
        serial_printf("ESTOP:SOFT — Motors homing to center\r\n");
        return;
#endif
    }

    // ── ESTOP:FULL — Hard emergency stop: kill all motion immediately ───
    // Stops GPTimer, emergency-stops all motors, blocks applyMotionValues().
    // Identical to pressing the physical E-Stop button.
    // Requires ESTOP:RESET to resume operation.
    if (strcmp(data, "ESTOP:FULL") == 0) {
        isPausedEStop = true;
#if defined(R13_BENCH_ACTIVE)
        r13_bench_stop("ESTOP_FULL");
#endif
#if PCB_VERSION == 1
        GPTimerScheduler::stop();
#else
        StepDriver_stop();
#endif
        for (int i = 0; i < 6; i++) motor_emergencyStop(i);
        serial_printf("ESTOP:FULL — motor engine stopped, all motors killed. Send ESTOP:RESET to recover.\r\n");
        return;
    }

    // ── ESTOP:RESET — Recover from FULL E-Stop ─────────────────────────
    // Clears E-Stop flag, enables rate limiting for safe ramp-up,
    // restarts GPTimer. Does nothing if not in E-Stop state.
    if (strcmp(data, "ESTOP:RESET") == 0) {
#if defined(R14_FAIL_CLOSED) && PCB_VERSION == 2
        // Serial reset supplies neither a fresh physical local-arm edge nor
        // proof of six sensors, six drives, bottom start and complete homing.
        serial_printf("ESTOP:RESET - R14 motion gate unqualified; reset denied\r\n");
        return;
#endif
        if (!isPausedEStop) {
            serial_printf("ESTOP:RESET — Not in E-Stop state, nothing to do\r\n");
            return;
        }
        if (!estop_reset_allowed(
                gpio_get_level((gpio_num_t)ESTOP_PIN) == ESTOP_ACTIVE_STATE,
                estop_button.debounced_state == ESTOP_ACTIVE_STATE)) {
            serial_printf("ESTOP:RESET — NC loop still active; reset denied\r\n");
            return;
        }
        // A diagnostic may have queued a target while the engine was stopped.
        // Discard every target before allowing another STEP pulse.
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            serial_printf("ESTOP:RESET — Motor mutex timeout; reset denied\r\n");
            return;
        }
        for (int i = 0; i < 6; i++) motor_emergencyStop(i);
        if (estop_input_active()) {
            xSemaphoreGive(xMutex);
            serial_printf("ESTOP:RESET — NC loop active; reset denied\r\n");
            return;
        }
        isPausedEStop = false;
        isRateLimiting = true;
#if PCB_VERSION == 1
        if (xGPIOLoopHandle != NULL) GPTimerScheduler::start(xGPIOLoopHandle);
#else
        StepDriver_resume();
#endif
        xSemaphoreGive(xMutex);
        if (estop_input_active()) {
#if PCB_VERSION == 1
            GPTimerScheduler::stop();
#else
            StepDriver_stop();
#endif
            for (int i = 0; i < 6; i++) motor_emergencyStop(i);
            isPausedEStop = true;
            serial_printf("ESTOP:RESET — NC loop changed; reset denied\r\n");
            return;
        }
        serial_printf("ESTOP:RESET — E-Stop cleared, motor engine restarted (rate-limited)\r\n");
        return;
    }

    // ── ESTOP? — Query current E-Stop state ─────────────────────────────
    if (strcmp(data, "ESTOP?") == 0) {
        serial_printf("ESTOP:state=%s\r\n", isPausedEStop ? "ACTIVE" : "OK");
        return;
    }

    // ── ZERO — Reset all motor step counters to 0 (post-homing sync) ──
    // Call after AASD-15A servo drives complete their homing sequence.
    // This synchronizes the ESP32's internal position tracking with the
    // physical zero established by the drives.
    // Tests:  Position counter reset, mutex safety
    // Proves: All 6 motors report pos=0/tgt=0 after reset
    if (strcmp(data, "ZERO") == 0) {
#if PCB_VERSION == 2
        // Without a proven Hall edge or other physical reference, resetting
        // counts would falsely label the present arm position as home.
        serial_printf("ZERO:ERR no qualified physical home reference\r\n");
        return;
#else
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            for (int i = 0; i < 6; i++) {
                motor_resetPosition(i);
#if defined(STEP_DRIVER_MCPWM)
                if (_mcpwm_motors[i]) _mcpwm_motors[i]->resetStats();
#endif
                arr[i] = 0;
            }
            xSemaphoreGive(xMutex);
            serial_printf("ZERO:OK — All motor positions reset to 0\r\n");
        } else {
            serial_printf("ZERO:ERR — Mutex timeout\r\n");
        }
        return;
#endif
    }

    // ── ZERO? — Query sync state (all motor current/target positions) ──
    if (strcmp(data, "ZERO?") == 0) {
        bool allZero = true;
        for (int i = 0; i < 6; i++) {
            int32_t pos = motor_getPos(i);
            int32_t tgt = motor_getTarget(i);
            if (pos != 0 || tgt != 0) allZero = false;
            serial_printf("  M%d: pos=%ld tgt=%ld\r\n", i, (long)pos, (long)tgt);
        }
        serial_printf("ZERO:sync=%s\r\n", allZero ? "OK" : "DRIFT");
        return;
    }

    // ── SCALE? — Query current axis scaling factors (geometry-derived) ─
    // Scales are computed from platform dimensions, not manually set.
    if (strcmp(data, "SCALE?") == 0) {
        serial_printf("SCALE:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
            axisScales.scale[0], axisScales.scale[1], axisScales.scale[2],
            axisScales.scale[3], axisScales.scale[4], axisScales.scale[5]);
        return;
    }

    // ── BITS:N — Set input bit depth (determines max_raw for axis scaling) ─
    if (controller::handshake_set_bits(data, inputBitRange, maxRawInput,
            [] { resetMotionCueing(&mcaConfig); }, serial_printf)) return;

    // NOTE: FINGERPRINT?, CONFIG?, BITS?, VERSION? are handled at the top
    // of process_data() for handshake priority. Only CONFIG:key=value (setter)
    // and SCALE? remain here.

    // ── CONFIG:key=value — Set platform geometry parameter ──────────────
    // Recomputes axis scales from geometry after change.
    // Keys: RD, PD, L1, L2, height, theta_r, theta_p
    if (strncmp(data, "CONFIG:", 7) == 0) {
        char *param = data + 7;
        char *eq = strchr(param, '=');
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
                configSaveToNVS(&stewartConfig);
                serial_printf("CONFIG:OK %s=%.4f (scales recomputed)\r\n", param, val);
            }
        }
        return;
    }

    // ── DRIVE:key=value — Set drive train parameter ────────────────────
    // Recomputes steps_per_degree after change.
    // Keys: encoder, gear, planetary
    if (strncmp(data, "DRIVE:", 6) == 0) {
        char *param = data + 6;
        char *eq = strchr(param, '=');
        if (eq) {
#if defined(STEP_DRIVER_SHARED_MCPWM)
            if (SharedMcpwmStepEngine::instance().isRunning()) {
                serial_printf("DRIVE:ERR disarm before changing scale\r\n"); return;
            }
#endif
            *eq = '\0';
            char* end = nullptr;
            float val = strtof(eq + 1, &end);
            if (end == eq + 1 || *end || !drive_finite(val)) {
                serial_printf("DRIVE:ERR invalid numeric value\r\n"); return;
            }
            StewartConfig next = stewartConfig;
            bool changed = true;
            // Same ranges as driveLoadFromNVS(): a 0 or absurd value would give an infinite
            // steps_per_degree and, because the value is persisted, survive a reboot.
            if (strcmp(param, "encoder") == 0) {
                if (val >= 1 && val <= 100000 && val == floorf(val)) next.encoder_ppr = (int)val; else changed = false;
            } else if (strcmp(param, "gear") == 0) {
                if (val > 0 && val <= 1000) next.virtual_gear = val; else changed = false;
            } else if (strcmp(param, "planetary") == 0) {
                if (val > 0 && val <= 1000) next.planetary_ratio = val; else changed = false;
            } else { serial_printf("DRIVE:ERR unknown key '%s'\r\n", param); return; }
            if (!changed || !valid_drive_scale(next.encoder_ppr,next.virtual_gear,next.planetary_ratio)) { serial_printf("DRIVE:ERR %s=%.4f out of range\r\n", param, val); return; }
            computeStepsPerDegree(&next);
#if defined(STEP_DRIVER_SHARED_MCPWM)
            const int32_t next_limit = (int32_t)(72.0f * next.steps_per_degree);
            if (StepDriver_isInitialized()) for (int i=0;i<6;++i) {
                if (StepDriver_getPosition(i) < -next_limit || StepDriver_getPosition(i) > next_limit ||
                    StepDriver_getTarget(i) < -next_limit || StepDriver_getTarget(i) > next_limit) {
                    serial_printf("DRIVE:ERR new limit excludes existing count; recalibration required\r\n"); return;
                }
            }
#endif
            stewartConfig = next;
            int saved = driveSaveToNVS(&stewartConfig);
#if defined(STEP_DRIVER_SHARED_MCPWM)
            // Soft limits are in degrees: follow the new scale now, not at the next boot.
            if (StepDriver_isInitialized()) for (int i = 0; i < STEP_DRIVER_NUM_MOTORS; i++)
                StepDriver_setSoftLimits(i, -motorSoftLimitSteps(), motorSoftLimitSteps());
#endif
            serial_printf("DRIVE:OK %s=%.4f → steps_deg=%.4f soft_limit=±%ld steps (±%.0f deg)%s\r\n", param, val,
                          stewartConfig.steps_per_degree, (long)motorSoftLimitSteps(), (double)MOTOR_SOFT_LIMIT_DEG,
                          saved == 0 ? " (saved)" : " (NVS save failed)");
        } else if (strcmp(param, "?") == 0) {
            serial_printf("DRIVE:encoder=%d,gear=%.4f,planetary=%.2f,steps_deg=%.4f,pulses_per_motor_rev=%.1f,pulses_per_output_rev=%.1f\r\n",
                stewartConfig.encoder_ppr, stewartConfig.virtual_gear, stewartConfig.planetary_ratio,
                stewartConfig.steps_per_degree,
                stewartConfig.encoder_ppr * 4.0f / stewartConfig.virtual_gear,
                stewartConfig.encoder_ppr * 4.0f / stewartConfig.virtual_gear * stewartConfig.planetary_ratio);
        }
        return;
    }

    // ── Motion Cueing serial commands ────────────────────────────────

    // ── MCA? — Query full Motion Cueing Algorithm config ─────────────
    // Tests:  MCA configuration readback, multi-line serial output
    // Proves: mcaConfig struct is coherent, preset name resolves correctly,
    //         all 6 axis channel filters + tilt coordination params serialize,
    //         serial_printf handles large multi-line output without truncation
    if (strcmp(data, "MCA?") == 0) {
        serial_printf("MCA:preset=%s,enabled=%d,sr=%.0f\r\n",
            mcaPresetName(mcaConfig.preset), mcaConfig.enabled, mcaConfig.sample_rate);
        const char* ax_names[] = {"surge","sway","heave","roll","pitch","yaw"};
        for (int j = 0; j < 6; j++) {
            AxisChannelFilter* ch = &mcaConfig.channels[j];
            serial_printf("  %s: hp=%d(fc=%.2f,Q=%.3f) lp=%d(fc=%.1f,Q=%.3f) gain=%.2f rl=%.2f\r\n",
                ax_names[j], ch->hp_enabled, ch->hp.fc, ch->hp.Q,
                ch->lp_enabled, ch->lp.fc, ch->lp.Q, ch->gain, ch->rate_limit);
        }
        serial_printf("  tilt: en=%d surge_g=%.4f sway_g=%.4f fc=%.2f Q=%.3f\r\n",
            mcaConfig.tilt.enabled, mcaConfig.tilt.surge_gain,
            mcaConfig.tilt.sway_gain, mcaConfig.tilt.fc, mcaConfig.tilt.Q);
        return;
    }

    // ── MCA:preset:<name> — Apply MCA preset ────────────────────────
    // Tests:  Preset loading and filter coefficient initialization
    // Proves: setMotionCueingPreset() populates all 6 channel biquad configs,
    //         resetMotionCueing() clears filter history before applying new preset,
    //         valid presets: off, gentle, moderate, aggressive, race_pro
    if (strncmp(data, "MCA:preset:", 11) == 0) {
        const char* name = data + 11;
        int preset = -1;
        if (strcmp(name, "off") == 0) preset = MCA_OFF;
        else if (strcmp(name, "gentle") == 0) preset = MCA_GENTLE;
        else if (strcmp(name, "moderate") == 0) preset = MCA_MODERATE;
        else if (strcmp(name, "aggressive") == 0) preset = MCA_AGGRESSIVE;
        else if (strcmp(name, "race_pro") == 0) preset = MCA_RACE_PRO;
        if (preset >= 0) {
            resetMotionCueing(&mcaConfig);
            setMotionCueingPreset(&mcaConfig, preset);
            serial_printf("MCA:preset=%s\r\n", mcaPresetName(mcaConfig.preset));
        } else {
            serial_printf("MCA:ERR unknown preset '%s'\r\n", name);
        }
        return;
    }

    // ── MCA:enabled:<0|1> — Master enable/disable for motion cueing ──
    // Tests:  MCA bypass path
    // Proves: When disabled (0), processMotionCueing() passes input through
    //         unfiltered; when enabled (1), biquad washout filters are active
    if (strncmp(data, "MCA:enabled:", 12) == 0) {
        mcaConfig.enabled = atoi(data + 12) ? 1 : 0;
        serial_printf("MCA:enabled=%d\r\n", mcaConfig.enabled);
        return;
    }

    // ── MCA:ch:<axis>:<param>:<value> — Per-channel filter parameter ─
    // Tests:  Individual axis filter tuning over serial
    // Proves: Runtime biquad coefficient recalculation (hp_fc, hp_Q, lp_fc,
    //         lp_Q), per-axis gain and rate limiter update, command parser
    //         handles nested colon-delimited format correctly
    //         e.g. MCA:ch:0:hp_fc:1.5  MCA:ch:3:gain:0.8
    if (strncmp(data, "MCA:ch:", 7) == 0) {
        int axis = -1;
        char param[16] = {0};
        float val = 0;
        char* p = data + 7;
        axis = atoi(p);
        p = strchr(p, ':');
        if (p) {
            p++;
            char* colon2 = strchr(p, ':');
            if (colon2) {
                int plen = colon2 - p;
                if (plen > 0 && plen < 16) {
                    strncpy(param, p, plen);
                    param[plen] = 0;
                }
                val = atof(colon2 + 1);
            }
        }
        if (axis >= 0 && axis < 6 && param[0]) {
            if (strcmp(param, "hp_fc") == 0) mcaSetChannelHpFc(&mcaConfig, axis, val);
            else if (strcmp(param, "hp_Q") == 0) mcaSetChannelHpQ(&mcaConfig, axis, val);
            else if (strcmp(param, "lp_fc") == 0) mcaSetChannelLpFc(&mcaConfig, axis, val);
            else if (strcmp(param, "lp_Q") == 0) mcaSetChannelLpQ(&mcaConfig, axis, val);
            else if (strcmp(param, "gain") == 0) mcaSetChannelGain(&mcaConfig, axis, val);
            else if (strcmp(param, "rate_limit") == 0) mcaSetChannelRateLimit(&mcaConfig, axis, val);
            else if (strcmp(param, "hp_en") == 0) mcaSetChannelHpEnabled(&mcaConfig, axis, (int)val);
            else if (strcmp(param, "lp_en") == 0) mcaSetChannelLpEnabled(&mcaConfig, axis, (int)val);
            else { serial_printf("MCA:ERR unknown param '%s'\r\n", param); return; }
            serial_printf("MCA:ch:%d:%s=%.4f\r\n", axis, param, val);
        } else {
            serial_printf("MCA:ERR bad ch command\r\n");
        }
        return;
    }

    // ── MCA:tilt:<param>:<value> — Tilt coordination parameters ──────
    // Tests:  Tilt coordination subsystem configuration
    // Proves: Sustained-acceleration tilt comp can be tuned live (enabled,
    //         surge_gain, sway_gain, fc, Q), filter state updates immediately
    if (strncmp(data, "MCA:tilt:", 9) == 0) {
        char* p = data + 9;
        char* colon = strchr(p, ':');
        if (colon) {
            char param[16] = {0};
            int plen = colon - p;
            if (plen > 0 && plen < 16) strncpy(param, p, plen);
            float val = atof(colon + 1);
            if (strcmp(param, "enabled") == 0) mcaSetTiltEnabled(&mcaConfig, (int)val);
            else if (strcmp(param, "surge_gain") == 0) mcaSetTiltSurgeGain(&mcaConfig, val);
            else if (strcmp(param, "sway_gain") == 0) mcaSetTiltSwayGain(&mcaConfig, val);
            else if (strcmp(param, "fc") == 0) mcaSetTiltFc(&mcaConfig, val);
            else if (strcmp(param, "Q") == 0) mcaSetTiltQ(&mcaConfig, val);
            else { serial_printf("MCA:ERR unknown tilt param '%s'\r\n", param); return; }
            serial_printf("MCA:tilt:%s=%.4f\r\n", param, val);
        }
        return;
    }

    // ── MCA:save — Persist MCA config to Non-Volatile Storage ────────
    // Tests:  NVS write path, config serialization
    // Proves: mcaSaveToNVS() writes full config blob to flash,
    //         survives power cycle, NVS partition is functional
    if (strcmp(data, "MCA:save") == 0) {
        int ret = mcaSaveToNVS(&mcaConfig);
        serial_printf("MCA:save=%s\r\n", ret == 0 ? "OK" : "FAIL");
        return;
    }

    // ── MCA:load — Load MCA config from NVS ─────────────────────────
    // Tests:  NVS read path, schema version check, fallback behavior
    // Proves: mcaLoadFromNVS() deserializes stored config correctly,
    //         falls back to initMotionCueing() defaults on version mismatch
    if (strcmp(data, "MCA:load") == 0) {
        int ret = mcaLoadFromNVS(&mcaConfig);
        if (ret != 0) {
            initMotionCueing(&mcaConfig, mcaConfig.sample_rate);
            serial_printf("MCA:load=FAIL (using defaults)\r\n");
        } else {
            serial_printf("MCA:load=OK preset=%s\r\n", mcaPresetName(mcaConfig.preset));
        }
        return;
    }

    // ── MCA:reset — Reset filter state (clear history, keep params) ──
    // Tests:  Biquad filter state reset without losing configuration
    // Proves: resetMotionCueing() zeroes all delay-line history (x[n-1],
    //         x[n-2], y[n-1], y[n-2]) so filters restart from silence
    if (strcmp(data, "MCA:reset") == 0) {
        resetMotionCueing(&mcaConfig);
        serial_printf("MCA:reset=OK\r\n");
        return;
    }

    // ── MCA:defaults — Reinitialize to factory defaults ──────────────
    // Tests:  Full MCA config reset path
    // Proves: initMotionCueing() restores all channels, tilt, preset to
    //         compile-time defaults; useful after bad manual tuning
    if (strcmp(data, "MCA:defaults") == 0) {
        initMotionCueing(&mcaConfig, mcaConfig.sample_rate);
        serial_printf("MCA:defaults=OK\r\n");
        return;
    }

    // ── WiFi serial commands ─────────────────────────────────────────
#ifdef ENABLE_WIFI
    // ── WIFI? — Query WiFi state (connection, IP, RSSI, SSID) ────────
    if (strcmp(data, "WIFI?") == 0) {
        serial_printf("WIFI:state=%s,ip=%s,rssi=%d,ssid=%s,udp_port=%d\r\n",
            wifi_transport_state_str(),
            wifi_transport_get_ip(),
            wifi_transport_get_rssi(),
            wifi_transport_get_ssid(),
            ETH_UDP_PORT);
        return;
    }

    // ── WIFI:SSID:<ssid> — Set WiFi SSID ─────────────────────────────
    if (strncmp(data, "WIFI:SSID:", 10) == 0) {
        const char *ssid = data + 10;
        wifi_transport_set_credentials(ssid, NULL);  // preserve existing password
        serial_printf("WIFI:SSID=%s\r\n", ssid);
        return;
    }

    // ── WIFI:PASS:<password> — Set WiFi password ─────────────────────
    if (strncmp(data, "WIFI:PASS:", 10) == 0) {
        const char *pass = data + 10;
        // Set password while keeping current SSID
        char current_ssid[33];
        strncpy(current_ssid, wifi_transport_get_ssid(), sizeof(current_ssid) - 1);
        current_ssid[sizeof(current_ssid) - 1] = '\0';
        wifi_transport_set_credentials(current_ssid, pass);
        serial_printf("WIFI:PASS=SET (%d chars)\r\n", (int)strlen(pass));
        return;
    }

    // ── WIFI:CRED:<ssid>:<password> — Set both SSID and password ─────
    if (strncmp(data, "WIFI:CRED:", 10) == 0) {
        char buf[98];
        strncpy(buf, data + 10, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';
        char *sep = strchr(buf, ':');
        if (sep) {
            *sep = '\0';
            wifi_transport_set_credentials(buf, sep + 1);
            serial_printf("WIFI:CRED ssid=%s pass=SET\r\n", buf);
        } else {
            wifi_transport_set_credentials(buf, "");
            serial_printf("WIFI:CRED ssid=%s (open network)\r\n", buf);
        }
        return;
    }

    // ── WIFI:CONNECT — Connect with current credentials ──────────────
    if (strcmp(data, "WIFI:CONNECT") == 0) {
        wifi_transport_connect();
        serial_printf("WIFI:CONNECT initiated ssid=%s\r\n", wifi_transport_get_ssid());
        return;
    }

    // ── WIFI:DISCONNECT — Disconnect from WiFi ──────────────────────
    if (strcmp(data, "WIFI:DISCONNECT") == 0) {
        wifi_transport_disconnect();
        serial_printf("WIFI:DISCONNECT=OK\r\n");
        return;
    }

    // ── WIFI:SAVE — Save credentials to NVS for auto-connect on boot ─
    if (strcmp(data, "WIFI:SAVE") == 0) {
        int ret = wifi_transport_save_credentials();
        serial_printf("WIFI:SAVE=%s\r\n", ret == 0 ? "OK" : "FAIL");
        return;
    }
#endif // ENABLE_WIFI

    // ── BLE serial commands ──────────────────────────────────────────
#ifdef ENABLE_BLE
    // ── BLE? — Query BLE state ───────────────────────────────────────
    if (strcmp(data, "BLE?") == 0) {
        serial_printf("BLE:state=%s\r\n", ble_transport_state_str());
        return;
    }
#endif // ENABLE_BLE
}

void outputDebugData() {
    // Caller (InterfaceMonitorTask) already throttles at DEBUG_OUTPUT_INTERVAL (10Hz).
    // Telemetry is always sent (dashboard needs ESP32's IK angles for arm viz).
    // Debug text is only sent when debugEnabled.
    send_telemetry((const float*)lastServoAngles);

    // Text debug line (only when enabled)
    if (debugEnabled) {
        serial_printf("DEBUG,%lld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
            (long long)micros(),
            (double)arr[0], (double)arr[1], (double)arr[2],
            (double)arr[3], (double)arr[4], (double)arr[5]);
    }
}

// ESP-IDF entry point (replaces Arduino setup/loop)
extern "C" void app_main(void)
{
#if PCB_VERSION == 2
    // r13 drive-enable latch (F02, minimal): GPIO47 MCU_INHIBIT must be HIGH and GPIO48
    // ARM_PULSE LOW before any peripheral takes ownership. Nothing in this firmware arms the
    // outputs yet; the latch stays cleared, so the AM26LS31s stay high-impedance.
    {
        gpio_config_t io = {};
        io.pin_bit_mask = (1ULL << 47) | (1ULL << 48);
        io.mode = GPIO_MODE_INPUT_OUTPUT;
        io.pull_up_en = GPIO_PULLUP_DISABLE;
        io.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io.intr_type = GPIO_INTR_DISABLE;
        gpio_set_level(GPIO_NUM_47, 1);
        gpio_set_level(GPIO_NUM_48, 0);
        gpio_config(&io);
        gpio_set_level(GPIO_NUM_47, 1);
        gpio_set_level(GPIO_NUM_48, 0);
    }
#endif
    // Initialize NVS (Non-Volatile Storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize COBS transport on UART0 — must be before any serial output
    cobs_transport_init(921600);
    // Register COBS channel handlers:
    //   DATA / DATA18 channels → serial_packet_handler (gates on activeInputSource → IK)
    //   CMD channel  → process_data (ASCII command handler)
    cobs_set_data_handler([](const uint8_t *payload, int len, int source) {
#ifdef ENABLE_ETHERNET
        if (source == COBS_SRC_ETHERNET) { ethernet_packet_handler(payload, len); return; }
#endif
        (void)source;
        serial_packet_handler(payload, len);
    });
    cobs_set_cmd_handler([](const char *cmd, int source) {
        (void)source;   // replies go to the console and are mirrored to the Ethernet peer
#if defined(R13_BENCH_ACTIVE)
        if (r13_bench_command(cmd, source)) return;   // console-only, refuses network sources
#endif
#if defined(R14_BOARD_IO)
        if (r14_hardware_command(cmd, source)) return;
#endif
        // process_data takes non-const char* for legacy reasons
        char buf[256];
        int n = snprintf(buf, sizeof(buf), "%s", cmd);
        if (n > 0) process_data(buf);
    });

    ESP_LOGI(TAG, "Stewart Platform Controller v%s (%s %s)",
        FW_VERSION_STRING, FW_BUILD_DATE, FW_BUILD_TIME);
  
    // Reconfigure watchdog (already auto-initialized by ESP-IDF 5.2 startup)
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT_MS,
        .idle_core_mask = 0,        // Don't watch idle tasks
        .trigger_panic = true       // Panic on timeout
    };
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&wdt_config));
  
    // Initialize Stewart platform configuration (NVS overrides defaults)
    initDefaultStewartConfig(&stewartConfig);
#if PCB_VERSION == 2
    // R13 bench and onward: AASD-15A electronic gear 2:1 (Pn098=2/Pn102=1, owner 2026-10-20),
    // 2500-line encoder, 50:1 planetary — 5 000 pulses per motor turn, 250 000 per arm turn
    // (694.4 per degree), confirmed with the camera against the 1:1 sweep table. At 2:1 the
    // 250 kHz engine ceiling is the motor's rated 3000 r/min = 360 deg/s at the arm.
    // stewart-core's default of 80 is the pre-R13 drive setting. DRIVE:gear= overrides (and persists).
    stewartConfig.virtual_gear = 2.0f;
    computeStepsPerDegree(&stewartConfig);
#endif
    if (configLoadFromNVS(&stewartConfig) == 0) {
        ESP_LOGI(TAG, "Geometry: Loaded from NVS (RD=%.2f, PD=%.2f)", stewartConfig.RD, stewartConfig.PD);
    } else {
        ESP_LOGI(TAG, "Geometry: Using factory defaults");
    }
    if (driveLoadFromNVS(&stewartConfig) == 0) {
        ESP_LOGI(TAG, "Drive train: loaded from NVS");
    }
    // Load saved input source + tick rate from NVS
    {
        nvs_handle_t nvs;
        if (nvs_open("stewart", NVS_READONLY, &nvs) == ESP_OK) {
            uint8_t src = 0;
            if (nvs_get_u8(nvs, "input_src", &src) == ESP_OK && src <= 3) {
                activeInputSource = (InputSource)src;
                ESP_LOGI(TAG, "Input source: %s (from NVS)", inputSourceName(activeInputSource));
            }
#if !defined(STEP_DRIVER_MCPWM)
            uint8_t tick_us = 0;
            if (nvs_get_u8(nvs, "tick_us", &tick_us) == ESP_OK && tick_us >= 4 && tick_us <= 100) {
                StepDriver_setTickRate((uint32_t)tick_us);
                ESP_LOGI(TAG, "Tick rate: %d µs (from NVS)", (int)tick_us);
            }
#endif
            nvs_close(nvs);
        }
    }

    // Derive axis scales from actual IK workspace (no magic numbers)
    computeAxisScalesFromGeometry(&axisScales, &stewartConfig, 0.90f);
    ESP_LOGI(TAG, "Axis scales (from geometry, 90%% margin): %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
        axisScales.scale[0], axisScales.scale[1], axisScales.scale[2],
        axisScales.scale[3], axisScales.scale[4], axisScales.scale[5]);
    ESP_LOGI(TAG, "Drive train: encoder=%d, gear=%.1f, planetary=%.1f → %.2f steps/deg",
        stewartConfig.encoder_ppr, stewartConfig.virtual_gear,
        stewartConfig.planetary_ratio, stewartConfig.steps_per_degree);

    // Initialize motion cueing — try NVS first, fall back to defaults
    initMotionCueing(&mcaConfig, 60.0f);  // default 60Hz sample rate
    if (mcaLoadFromNVS(&mcaConfig) == 0) {
        DEBUG_PRINTF("MCA: Loaded saved config (preset=%s)\n", mcaPresetName(mcaConfig.preset));
    } else {
        DEBUG_PRINTLN("MCA: Using factory defaults");
    }

    // E-Stop hardware initialization
#if PCB_VERSION == 1
    // PCBv1: E-stop on GPIO 22 — safe to use, enable hardware monitoring
    initDebounceButton(&estop_button, (gpio_num_t)ESTOP_PIN, ESTOPDEBOUNCETIME);
#else
    // PCBv2: E-stop on GPIO 21; GPIO 20 remains dedicated to USB D+.
    initDebounceButton(&estop_button, (gpio_num_t)ESTOP_PIN, ESTOPDEBOUNCETIME);
#endif
    
    // Motor init moved to GPIOLoopTask (core 1) so MCPWM ISRs are pinned
    // to core 1 — no WiFi/serial/BLE interrupt contention. See GPIOLoopTask().
    
    // Create mutex for thread safety
    xMutex = xSemaphoreCreateMutex();
    
    // Both board revisions have a dedicated E-stop GPIO.
    xTaskCreatePinnedToCore(
        EStopMonitorTask, "EStopMonitor", 4096, NULL,
        configMAX_PRIORITIES-1, NULL, 0);
    
    // Create tasks for interface monitoring and GPIO control with proper stack sizes
    xTaskCreatePinnedToCore(
        InterfaceMonitorTask,    /* Task function. */
        "InterfaceMonitor",      /* name of task. */
        8192,                    /* Stack size of task */
        NULL,                    /* parameter of the task */
        2,                       /* priority of the task */
        NULL,                    /* Task handle */
        0);                      /* pin task to core 0 */
    
    xTaskCreatePinnedToCore(
        GPIOLoopTask,            /* Task function. */
        "GPIOLoop",              /* name of task. */
        8192,                    /* Stack size of task */
        NULL,                    /* parameter of the task */
        3,                       /* priority of the task */
        NULL,                    /* Task handle */
        1);                      /* pin task to core 1 */

#if defined(R14_BOARD_IO)
    r14_hardware_start();
#endif
    
#ifdef ENABLE_ETHERNET
    // Initialize W5500 SPI Ethernet + UDP listener
    // UDP packets gated by activeInputSource
    if (!ethernet_transport_init()) {
        ESP_LOGW(TAG, "Ethernet init failed — serial-only mode");
    }
#endif

#ifdef ENABLE_WIFI
    // Initialize WiFi STA + UDP listener
    // WiFi credentials loaded from NVS (saved via WIFI:SAVE command)
    // UDP packets gated by activeInputSource
    if (!wifi_transport_init(wifi_packet_handler)) {
        ESP_LOGW(TAG, "WiFi init failed — serial-only mode");
    }
#endif

#ifdef ENABLE_BLE
    // Initialize BLE GATT server
    // Advertises as "StewartPlatform", accepts binary motion packets via BLE write
    // Gated by activeInputSource
    if (!ble_transport_init(ble_packet_handler)) {
        ESP_LOGW(TAG, "BLE init failed — serial/WiFi only");
    }
#endif

    // LED status indicator — white solid during boot, green breathe when ready.
    // Not on the r13 board: GPIO48 is the drive-enable ARM input there (F02), so the WS2812
    // waveform must never be generated. led_status_set/clear stay harmless no-ops.
#if PCB_VERSION != 2
    led_status_init(STATUS_LED_GPIO);
#endif

    led_status_clear(LED_STATE_BOOT);
    led_status_set(LED_STATE_READY);
    ESP_LOGI(TAG, "Setup Complete!");
    serial_println("READY");
    
    // app_main returns, tasks continue running
}
