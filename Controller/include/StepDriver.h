#pragma once

/*
 * StepDriver — Compile-time pluggable stepper backend
 *
 * Select backend via STEP_DRIVER define (in CMakeLists.txt or as -D flag):
 *
 *   STEP_DRIVER_SSE    — SimpleStepEngine
 *                        One GPTimer ISR @ 4µs ticks drives all 6 motors.
 *                        Max rate: 125 kHz/motor. Minimal, zero MCPWM/PCNT/RMT.
 *                        Stable. Low ISR budget overhead (~180 cycles/tick).
 *
 *   STEP_DRIVER_MCPWM  — MCPWMMotorControl
 *                        6 MCPWM timers (2 groups × 3), mixed counting:
 *                          Motors 0-3: MCPWM generator → step pin + PCNT hw counter
 *                            (ESP32-S3 has 4 PCNT units — zero CPU per pulse)
 *                          Motors 4-5: MCPWM timer + RMT TX → step pin + loop_count
 *                            (no PCNT left; RMT handles pulse gen + counting)
 *                          Fallback:  MCPWM generator + TEZ ISR (if RMT also fails)
 *                        Max rate: 250 kHz/motor (all hardware-timed, no ISR bottleneck).
 *                        Complex driver stack. Currently the active/tested path.
 *
 *   STEP_DRIVER_MCPWM_ISR — Single MCPWM timer TEZ ISR drives all 6 motors.
 *                        Uses ONE MCPWM timer (group 0, timer 0) at 4µs period.
 *                        TEZ (timer-empty) ISR fires every 4µs, services all 6
 *                        motors per tick with direct GPIO register writes —
 *                        same algorithm as SSE but using MCPWM instead of GPTimer.
 *                        Advantage over SSE: MCPWM TEZ ISRs are placed in IRAM by
 *                        ESP-IDF automatically; no CONFIG_GPTIMER_ISR_IRAM_SAFE
 *                        sdkconfig flag needed.
 *                        Max rate: 125 kHz/motor. No PCNT/RMT needed.
 *                        12 GPIO pins only. Zero per-motor MCPWM resources.
 *
 *   STEP_DRIVER_SHARED_MCPWM — 1 MCPWM timer, 6 hardware comparator/generator chains.
 *                        One MCPWM timer (group 0) → 3 operators → 2 comparators each
 *                        → 6 generators wired directly to STEP GPIOs in silicon.
 *                        Pulse generation is 100% hardware — no per-tick ISR.
 *                        A lightweight comparator ISR fires only when a step actually
 *                        occurs (not at 250 kHz idle rate) to count steps and re-arm.
 *                        Max rate: 250 kHz/motor. Zero CPU per pulse. No PCNT/RMT.
 *
 * Default: STEP_DRIVER_SSE (SimpleStepEngine).
 * To switch: edit main/CMakeLists.txt and change the target_compile_definitions line.
 *
 * Public API (same for all backends):
 *
 *   StepDriver_init(configs)        — configure & init all 6 motors
 *   StepDriver_start()              — start the timing engine
 *   StepDriver_stop()               — emergency stop (step pins LOW)
 *   StepDriver_resume()             — resume after stop
 *   StepDriver_setTarget(i, pos)    — set target position for motor i
 *   StepDriver_getPosition(i)       — read current position of motor i
 *   StepDriver_getTarget(i)         — read target position of motor i
 *   StepDriver_resetPosition(i)     — zero current + target for motor i
 *   StepDriver_emergencyStop()      — freeze all motors at current position
 *   StepDriver_handleStep()         — called each control tick (task context)
 *                                     SSE: no-op (ISR drives motors autonomously)
 *                                     MCPWM: starts continuous moves, polls done
 *   StepDriver_isInitialized()      — true after successful init
 *   StepDriver_report()             — print backend name + stats to DEBUG_PRINTF
 *   StepDriver_setTickRate(µs)      — hot-adjust ISR tick period (µs); returns false
 *                                     if out of per-backend range or not supported.
 *                                     SSE:  4–100 µs  (250 kHz … 5 kHz tick)
 *                                     MSE:  4–100 µs  (same, MCPWM timer)
 *                                     SMSE: 4–100 µs  (must be > SMSE_PULSE_US=2)
 *                                     MCPWM: always returns false (hardware-fixed)
 *   StepDriver_getTickUs()          — current tick period in µs
 *   StepDriver_getMaxStepHz()       — current max step frequency per motor
 *
 * Motor config (common fields used by all backends):
 *   StepDriverMotorConfig { stepPin, dirPin, invertDir, softLimitMin, softLimitMax }
 */

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "debug_uart.h"

// ── Backend selection ────────────────────────────────────────────────
#if !defined(STEP_DRIVER_SSE) && !defined(STEP_DRIVER_MCPWM) && !defined(STEP_DRIVER_MCPWM_ISR) && !defined(STEP_DRIVER_SHARED_MCPWM)
    #define STEP_DRIVER_SSE   // default
#endif

#define STEP_DRIVER_NUM_MOTORS  6

// ── Common motor configuration struct ───────────────────────────────
struct StepDriverMotorConfig {
    gpio_num_t stepPin;
    gpio_num_t dirPin;
    bool       invertDir;
    int32_t    softLimitMin;
    int32_t    softLimitMax;
    bool       enableSoftLimits;

    StepDriverMotorConfig() :
        stepPin(GPIO_NUM_NC),
        dirPin(GPIO_NUM_NC),
        invertDir(false),
        softLimitMin(-100000),
        softLimitMax(100000),
        enableSoftLimits(true)
    {}
};

// ── Backend includes ─────────────────────────────────────────────────
#if defined(STEP_DRIVER_SSE)
    #include "SimpleStepEngine.h"
#elif defined(STEP_DRIVER_MCPWM)
    #include "MCPWMMotorControl.h"
#elif defined(STEP_DRIVER_MCPWM_ISR)
    #include "McpwmStepEngine.h"
#elif defined(STEP_DRIVER_SHARED_MCPWM)
    #include "SharedMcpwmStepEngine.h"
#endif

// ── Backend implementations ──────────────────────────────────────────

#if defined(STEP_DRIVER_SSE)
// ════════════════════════════════════════════════════════════════════
//  SimpleStepEngine backend
//  One GPTimer ISR at 4µs ticks, all 6 motors serviced per tick.
//  Max 125 kHz/motor. ISR runs in IRAM, direct register GPIO writes.
// ════════════════════════════════════════════════════════════════════

static inline bool StepDriver_init(const StepDriverMotorConfig configs[STEP_DRIVER_NUM_MOTORS]) {
    SimpleStepEngine::MotorConfig sse_cfg[SSE_NUM_MOTORS];
    for (int i = 0; i < STEP_DRIVER_NUM_MOTORS; i++) {
        sse_cfg[i].stepPin          = configs[i].stepPin;
        sse_cfg[i].dirPin           = configs[i].dirPin;
        sse_cfg[i].invertDir        = configs[i].invertDir;
        sse_cfg[i].softLimitMin     = configs[i].softLimitMin;
        sse_cfg[i].softLimitMax     = configs[i].softLimitMax;
        sse_cfg[i].enableSoftLimits = configs[i].enableSoftLimits;
    }
    return SimpleStepEngine::instance().init(sse_cfg);
}

static inline bool    StepDriver_start()                           { return SimpleStepEngine::instance().start(); }
static inline void    StepDriver_stop()                            { SimpleStepEngine::instance().stop(); }
static inline void    StepDriver_resume()                          { SimpleStepEngine::instance().resume(); }
static inline bool    StepDriver_setTarget(int i, int32_t pos)    { return SimpleStepEngine::instance().setTarget(i, pos); }
static inline int32_t StepDriver_getPosition(int i)               { return SimpleStepEngine::instance().getPosition(i); }
static inline int32_t StepDriver_getTarget(int i)                 { return SimpleStepEngine::instance().getTarget(i); }
static inline void    StepDriver_resetPosition(int i)             { SimpleStepEngine::instance().resetPosition(i); }
static inline bool    StepDriver_isInitialized()                  { return SimpleStepEngine::instance().isInitialized(); }
static inline void    StepDriver_emergencyStop() {
    SimpleStepEngine::instance().emergencyStop();
}

static inline uint32_t StepDriver_getTotalSteps(int i)          { return SimpleStepEngine::instance().getTotalSteps(i); }

// SSE is ISR-driven — handleStep() is a no-op.
// The ISR fires autonomously every 4µs; no task-level polling needed.
static inline void StepDriver_handleStep() { /* ISR-autonomous */ }

static inline void StepDriver_report() {
    auto& sse = SimpleStepEngine::instance();
    DEBUG_PRINTF("[StepDriver] Backend: SimpleStepEngine (SSE)\n");
    DEBUG_PRINTF("  Tick: %lu µs | Max step rate: %lu Hz/motor\n",
        (unsigned long)sse.getTickUs(),
        (unsigned long)sse.getMaxStepHz());
    DEBUG_PRINTF("  Tick count: %llu\n", (unsigned long long)sse.getTickCount());
    for (int i = 0; i < STEP_DRIVER_NUM_MOTORS; i++) {
        DEBUG_PRINTF("  M%d: pos=%ld  target=%ld  steps=%lu  dirChanges=%lu\n",
            i,
            (long)sse.getPosition(i),
            (long)sse.getTarget(i),
            (unsigned long)sse.getTotalSteps(i),
            (unsigned long)sse.getDirChanges(i));
    }
}

// ── Tick rate tuning — common API, all ISR-driven backends ───────────
// Range: SSE_TICK_US_MIN (4µs) … SSE_TICK_US_MAX (100µs).
// 4µs is the hard WDT-safe floor on ESP32-S3 — do not go below it.
static inline bool     StepDriver_setTickRate(uint32_t tick_us) { return SimpleStepEngine::instance().setTickRate(tick_us); }
static inline uint32_t StepDriver_getTickUs()                   { return SimpleStepEngine::instance().getTickUs(); }
static inline uint32_t StepDriver_getMaxStepHz()                { return SimpleStepEngine::instance().getMaxStepHz(); }

// ────────────────────────────────────────────────────────────────────
#elif defined(STEP_DRIVER_MCPWM)
// ════════════════════════════════════════════════════════════════════
//  MCPWMMotorControl backend
//  6 independent MCPWM timers + PCNT/RMT hardware counting.
//  Max 250 kHz/motor (hardware-timed, truly parallel).
// ════════════════════════════════════════════════════════════════════

// Module-level motor array (file-scope, backend-private)
static MCPWMMotorControl* _mcpwm_motors[STEP_DRIVER_NUM_MOTORS] = {};

static inline bool StepDriver_init(const StepDriverMotorConfig configs[STEP_DRIVER_NUM_MOTORS]) {
    MCPWMMotorControl::Config mcfg;
    mcfg.stepPulseWidth_us  = 2;
    mcfg.dirSetupTime_us    = 5;
    mcfg.minStepInterval_us = 4;
    mcfg.maxStepRate        = 250000;
    mcfg.maxAcceleration    = 100000;
    mcfg.enableSoftLimits   = true;

    int ok = 0;
    for (int i = 0; i < STEP_DRIVER_NUM_MOTORS; i++) {
        mcfg.invertDirection = configs[i].invertDir;
        mcfg.softLimitMin    = configs[i].softLimitMin;
        mcfg.softLimitMax    = configs[i].softLimitMax;
        _mcpwm_motors[i] = new MCPWMMotorControl(configs[i].stepPin, configs[i].dirPin);
        if (_mcpwm_motors[i]->begin(mcfg)) {
            ok++;
        } else {
            DEBUG_PRINTF("[StepDriver] MCPWM M%d init failed: err=%d\n",
                i, _mcpwm_motors[i]->getLastError());
        }
    }
    DEBUG_PRINTF("[StepDriver] Backend: MCPWMMotorControl — %d/6 OK\n", ok);
    return ok == STEP_DRIVER_NUM_MOTORS;
}

// MCPWM has no separate start/stop concept per-engine; motors start on first move.
static inline bool StepDriver_start()   { return true; }
static inline void StepDriver_stop() {
    for (int i = 0; i < STEP_DRIVER_NUM_MOTORS; i++)
        if (_mcpwm_motors[i]) _mcpwm_motors[i]->emergencyStop();
}
static inline void StepDriver_resume()  { /* MCPWM resumes on next setTarget */ }

static inline bool StepDriver_setTarget(int i, int32_t pos) {
    return (_mcpwm_motors[i]) ? _mcpwm_motors[i]->setTargetPosition(pos) : false;
}
static inline int32_t StepDriver_getPosition(int i) {
    return (_mcpwm_motors[i]) ? _mcpwm_motors[i]->getCurrentPosition() : 0;
}
static inline int32_t StepDriver_getTarget(int i) {
    return (_mcpwm_motors[i]) ? _mcpwm_motors[i]->getTargetPosition() : 0;
}
static inline void StepDriver_resetPosition(int i) {
    if (_mcpwm_motors[i]) _mcpwm_motors[i]->resetPosition();
}
static inline bool StepDriver_isInitialized() {
    for (int i = 0; i < STEP_DRIVER_NUM_MOTORS; i++)
        if (!_mcpwm_motors[i] || !_mcpwm_motors[i]->isInitialized()) return false;
    return true;
}
static inline void StepDriver_emergencyStop() {
    for (int i = 0; i < STEP_DRIVER_NUM_MOTORS; i++)
        if (_mcpwm_motors[i]) _mcpwm_motors[i]->emergencyStop();
}
static inline uint32_t StepDriver_getTotalSteps(int i) {
    if (i < 0 || i >= STEP_DRIVER_NUM_MOTORS || !_mcpwm_motors[i]) return 0;
    return _mcpwm_motors[i]->getStats().totalSteps;
}

// MCPWM: start continuous moves then poll completion (task context, not ISR).
static inline void StepDriver_handleStep() {
    bool anyStarted = false;
    for (int i = 0; i < STEP_DRIVER_NUM_MOTORS; i++) {
        if (!_mcpwm_motors[i]) continue;
        int32_t delta = _mcpwm_motors[i]->getTargetPosition()
                      - _mcpwm_motors[i]->getCurrentPosition();
        if (delta == 0) continue;
        _mcpwm_motors[i]->startContinuousSteps(delta);
        anyStarted = true;
    }
    if (anyStarted) {
        bool anyRunning = true;
        while (anyRunning) {
            anyRunning = false;
            for (int i = 0; i < STEP_DRIVER_NUM_MOTORS; i++) {
                if (_mcpwm_motors[i] && !_mcpwm_motors[i]->checkContinuousDone())
                    anyRunning = true;
            }
        }
    }
}

static inline bool     StepDriver_setTickRate(uint32_t tick_us) {
    DEBUG_PRINTF("[StepDriver] MCPWM backend: tick rate is hardware-fixed at 250 kHz, cannot change (requested %lu µs)\n", (unsigned long)tick_us);
    return false;
}
static inline uint32_t StepDriver_getTickUs()    { return 4; }   // hardware-fixed 4µs pulse period
static inline uint32_t StepDriver_getMaxStepHz() { return 250000UL; }

static inline void StepDriver_report() {
    DEBUG_PRINTF("[StepDriver] Backend: MCPWMMotorControl (MCPWM)\n");
    DEBUG_PRINTF("  Max step rate: 250000 Hz/motor (hardware-timed, fixed rate)\n");
    for (int i = 0; i < STEP_DRIVER_NUM_MOTORS; i++) {
        if (!_mcpwm_motors[i]) continue;
        const auto& st = _mcpwm_motors[i]->getStats();
        DEBUG_PRINTF("  M%d: pos=%ld  target=%ld  steps=%lu  errors=%lu\n",
            i,
            (long)_mcpwm_motors[i]->getCurrentPosition(),
            (long)_mcpwm_motors[i]->getTargetPosition(),
            (unsigned long)st.totalSteps,
            (unsigned long)st.stepErrors);
    }
}

// ────────────────────────────────────────────────────────────────────
#elif defined(STEP_DRIVER_MCPWM_ISR)
// ════════════════════════════════════════════════════════════════════
//  McpwmStepEngine backend
//  One MCPWM TEZ ISR at 4µs ticks, all 6 motors serviced per tick.
//  Max 125 kHz/motor. MCPWM TEZ ISRs are IRAM-placed by ESP-IDF
//  automatically — no sdkconfig flag needed (unlike GPTimer/SSE).
// ════════════════════════════════════════════════════════════════════

static inline bool StepDriver_init(const StepDriverMotorConfig configs[STEP_DRIVER_NUM_MOTORS]) {
    McpwmStepEngine::MotorConfig mse_cfg[MSE_NUM_MOTORS];
    for (int i = 0; i < STEP_DRIVER_NUM_MOTORS; i++) {
        mse_cfg[i].stepPin          = configs[i].stepPin;
        mse_cfg[i].dirPin           = configs[i].dirPin;
        mse_cfg[i].invertDir        = configs[i].invertDir;
        mse_cfg[i].softLimitMin     = configs[i].softLimitMin;
        mse_cfg[i].softLimitMax     = configs[i].softLimitMax;
        mse_cfg[i].enableSoftLimits = configs[i].enableSoftLimits;
    }
    return McpwmStepEngine::instance().init(mse_cfg);
}

static inline bool    StepDriver_start()                        { return McpwmStepEngine::instance().start(); }
static inline void    StepDriver_stop()                         { McpwmStepEngine::instance().stop(); }
static inline void    StepDriver_resume()                       { McpwmStepEngine::instance().resume(); }
static inline bool    StepDriver_setTarget(int i, int32_t pos)  { return McpwmStepEngine::instance().setTarget(i, pos); }
static inline int32_t StepDriver_getPosition(int i)             { return McpwmStepEngine::instance().getPosition(i); }
static inline int32_t StepDriver_getTarget(int i)               { return McpwmStepEngine::instance().getTarget(i); }
static inline void    StepDriver_resetPosition(int i)           { McpwmStepEngine::instance().resetPosition(i); }
static inline bool    StepDriver_isInitialized()                { return McpwmStepEngine::instance().isInitialized(); }
static inline void    StepDriver_emergencyStop()                { McpwmStepEngine::instance().emergencyStop(); }
static inline uint32_t StepDriver_getTotalSteps(int i)          { return McpwmStepEngine::instance().getTotalSteps(i); }
static inline void    StepDriver_resetStats(int i)              { McpwmStepEngine::instance().resetStats(i); }
static inline void    StepDriver_handleStep()                   { /* ISR-autonomous */ }

static inline bool     StepDriver_setTickRate(uint32_t tick_us) { return McpwmStepEngine::instance().setTickRate(tick_us); }
static inline uint32_t StepDriver_getTickUs()                    { return McpwmStepEngine::instance().getTickUs(); }
static inline uint32_t StepDriver_getMaxStepHz()                 { return McpwmStepEngine::instance().getMaxStepHz(); }

static inline void StepDriver_report() {
    auto& mse = McpwmStepEngine::instance();
    DEBUG_PRINTF("[StepDriver] Backend: McpwmStepEngine (MCPWM_ISR)\n");
    DEBUG_PRINTF("  Tick: %lu µs | Tick rate: %lu Hz | Max step rate: %lu Hz/motor\n",
        (unsigned long)mse.getTickUs(),
        (unsigned long)mse.getTickHz(),
        (unsigned long)mse.getMaxStepHz());
    for (int i = 0; i < STEP_DRIVER_NUM_MOTORS; i++) {
        DEBUG_PRINTF("  M%d: pos=%ld  target=%ld  steps=%lu  dirChanges=%lu\n",
            i,
            (long)mse.getPosition(i),
            (long)mse.getTarget(i),
            (unsigned long)mse.getTotalSteps(i),
            (unsigned long)mse.getDirChanges(i));
    }
}

#elif defined(STEP_DRIVER_SHARED_MCPWM)
// ════════════════════════════════════════════════════════════════════
//  SharedMcpwmStepEngine backend
//  1 MCPWM timer (group 0) → 3 operators → 6 comparator/generator chains.
//  Hybrid: TEZ ISR (250 kHz) drives rising edge via w1ts GPIO register write;
//          hardware comparator/generator drives falling edge (zero CPU).
//  DIR/STEP sync: dirPending 2-tick guard = 8µs setup (meets AASD-15A ≥2µs).
//  Max 250 kHz/motor. One ISR per tick (not per pulse-edge).
// ════════════════════════════════════════════════════════════════════

static inline bool StepDriver_init(const StepDriverMotorConfig configs[STEP_DRIVER_NUM_MOTORS]) {
    SharedMcpwmStepEngine::MotorConfig cfg[STEP_DRIVER_NUM_MOTORS];
    for (int i = 0; i < STEP_DRIVER_NUM_MOTORS; i++) {
        cfg[i].stepPin          = configs[i].stepPin;
        cfg[i].dirPin           = configs[i].dirPin;
        cfg[i].invertDir        = configs[i].invertDir;
        cfg[i].softLimitMin     = configs[i].softLimitMin;
        cfg[i].softLimitMax     = configs[i].softLimitMax;
        cfg[i].enableSoftLimits = configs[i].enableSoftLimits;
    }
    return SharedMcpwmStepEngine::instance().init(cfg);
}

static inline bool    StepDriver_start()                        { return SharedMcpwmStepEngine::instance().start(); }
static inline void    StepDriver_stop()                         { SharedMcpwmStepEngine::instance().stop(); }
static inline void    StepDriver_resume()                       { SharedMcpwmStepEngine::instance().resume(); }
static inline bool    StepDriver_setTarget(int i, int32_t pos)  { return SharedMcpwmStepEngine::instance().setTarget(i, pos); }
static inline int32_t StepDriver_getPosition(int i)             { return SharedMcpwmStepEngine::instance().getPosition(i); }
static inline int32_t StepDriver_getTarget(int i)               { return SharedMcpwmStepEngine::instance().getTarget(i); }
static inline void    StepDriver_resetPosition(int i)           { SharedMcpwmStepEngine::instance().resetPosition(i); }
static inline bool    StepDriver_isInitialized()                { return SharedMcpwmStepEngine::instance().isInitialized(); }
static inline void    StepDriver_emergencyStop()                { SharedMcpwmStepEngine::instance().emergencyStop(); }
static inline uint32_t StepDriver_getTotalSteps(int i)          { return SharedMcpwmStepEngine::instance().getTotalSteps(i); }
static inline void    StepDriver_resetStats(int i)              { SharedMcpwmStepEngine::instance().resetStats(i); }
static inline void    StepDriver_handleStep()                   { /* ISR-autonomous */ }

static inline bool     StepDriver_setTickRate(uint32_t tick_us) { return SharedMcpwmStepEngine::instance().setTickRate(tick_us); }
static inline uint32_t StepDriver_getTickUs()                   { return SharedMcpwmStepEngine::instance().getTickUs(); }
static inline uint32_t StepDriver_getMaxStepHz()                { return SharedMcpwmStepEngine::instance().getMaxStepHz(); }

static inline void StepDriver_report() {
    auto& e = SharedMcpwmStepEngine::instance();
    DEBUG_PRINTF("[StepDriver] Backend: SharedMcpwmStepEngine (SHARED_MCPWM)\n");
    DEBUG_PRINTF("  Period: %lu µs | Pulse: %d µs | Max step rate: %lu Hz/motor\n",
        (unsigned long)e.getTickUs(), SMSE_PULSE_US, (unsigned long)e.getMaxStepHz());
    for (int i = 0; i < STEP_DRIVER_NUM_MOTORS; i++) {
        DEBUG_PRINTF("  M%d: pos=%ld  target=%ld  steps=%lu  dirChanges=%lu\n",
            i,
            (long)e.getPosition(i),
            (long)e.getTarget(i),
            (unsigned long)e.getTotalSteps(i),
            (unsigned long)e.getDirChanges(i));
    }
}

#endif // backend selection
