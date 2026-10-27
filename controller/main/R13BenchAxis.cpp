// R13BenchAxis.cpp — R13 one-axis bench mode. See R13BenchAxis.h and
// hardware/pcb/6dof2_r13/bringup/M1_OUTPUT_TEST_PLAN.md step 12.
#include "R13BenchAxis.h"
#include "R13BenchPulse.h"
#include "CobsTransport.h"
#include "debug_uart.h"

#include <cstdio>
#include <cstring>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "soc/mcpwm_periph.h"
#include "soc/mcpwm_struct.h"

#ifndef FW_GIT_SHA
#define FW_GIT_SHA "unknown"
#endif

#define BENCH_AXIS          0
#define BENCH_INHIBIT_GPIO  GPIO_NUM_47   // HIGH = inhibit (R61 default)
#define BENCH_ARM_GPIO      GPIO_NUM_48   // rising edge sets U11
#define BENCH_ARM_WAIT_MS   300           // U13 CT release ~107 ms nominal, 158 ms worst
#define BENCH_ARM_PULSE_US  50
#define BENCH_TICK_US       200           // 5 kHz schedule tick; R13BenchPulse batches up to 50 steps/tick at 250 kHz
// Arm envelope and runaway allowances in degrees, converted with the drive's current gear.
#define BENCH_WINDOW_DEG        60.0f   // +/-60 deg test window (hard stop near -100 / +80)
#define BENCH_RUNAWAY_DEG        3.6f   // schedule mode (2500 pulses at 2:1, 5000 at 1:1)
#define BENCH_RUNAWAY_HEAVE_DEG 21.6f   // heave mode (15000 pulses at 2:1, 30000 at 1:1)
#define BENCH_LATCH_RING        48      // SharedMcpwmStepEngine::LATCH_RING

// Engine hooks and the IK helper, all in main.cpp (bench build only).
void r13_engine_isr_stats(uint32_t *count, uint32_t *cycles, uint32_t *maxc, bool reset);
void r13_engine_isr_gap(uint32_t *gap_min, uint32_t *gap_sum);
uint32_t r13_engine_isr_missed(void);
void r13_engine_latch_stats(uint32_t *dneg, uint32_t *dpos, uint32_t *dbig, uint32_t *mpos, uint32_t *smax);
uint32_t r13_engine_latch_events(bool reset);
bool r13_engine_latch_event(uint32_t k, uint32_t n, uint32_t out[5], int32_t *d);
uint32_t r13_engine_period_cycles(void);
void r13_engine_resync_stats(uint32_t *resyncs, uint32_t *skipmax);
void r13_engine_isr_delay(uint32_t cyc);
bool r13_engine_latch_comp(int set);                                                     // -1 = query
int  r13_bench_ik(const float pos[6], float angles_deg[6], long steps[6]);
float r13_bench_steps_per_degree();

static const R13BenchOps *s_ops = nullptr;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static R13BenchPulse s_pulse;
static volatile bool s_armed = false;
static int32_t s_target = 0;
static esp_timer_handle_t s_timer = nullptr;
static TaskHandle_t s_report_task = nullptr;
static volatile bool s_done_pending = false;
// On-controller heave sine through the production IK (BENCH:HEAVE), stepped in bench_tick.
static volatile bool  s_heave_active = false, s_heave_done = false;
static float s_heave_zpos = 0, s_heave_zneg = 0, s_heave_t = 0, s_heave_end = 0;
// Composite waveform (BENCH:WAVE): u(t) = sum w[i] * sin(2 pi f[i] t + ph[i]), weights sum to 1.
static int   s_wave_n = 1;
static float s_wave_w[4] = {1, 0, 0, 0}, s_wave_f[4] = {0, 0, 0, 0}, s_wave_ph[4] = {0, 1.3f, 2.9f, 4.4f};
static int   s_heave_div = 0;
static volatile int32_t s_heave_peak_lag = 0;
static volatile uint32_t s_heave_poses = 0;
// Engine throughput measurement: time from the first target step of a BENCH:PULSE schedule to the
// moment the engine position reaches the final target (printed as BENCH:ENGINE:DONE).
static volatile int64_t s_run_t0 = 0, s_run_t1 = 0;
static volatile bool s_run_catchup = false, s_run_report = false;
static volatile int32_t s_run_pulses = 0;
static const char *volatile s_stop_reason = nullptr;

// Pulse counter on the STEP0 pad (GPIO4, routed back in through io_loop_back) with DIR0
// (GPIO10) as the level input: +1 per rising edge while DIR is high, -1 while low. This is
// what the drive sees, counted by hardware, so emitted pulses can be checked against the
// engine's position without a meter or scope. Zeroed on BENCH:ARM.
#define BENCH_STEP_GPIO     GPIO_NUM_4
#define BENCH_DIR_GPIO      GPIO_NUM_10
#define BENCH_PCNT_LIMIT    30000
static pcnt_unit_handle_t s_pcnt = nullptr;
static volatile int32_t s_pcnt_wraps = 0;      // accumulated +/-BENCH_PCNT_LIMIT per limit hit

static bool IRAM_ATTR pcnt_on_reach(pcnt_unit_handle_t, const pcnt_watch_event_data_t *ev, void *) {
    s_pcnt_wraps += ev->watch_point_value;      // counter auto-clears at the limit
    return false;
}

static int32_t pcnt_read(void) {
    int v = 0;
    if (s_pcnt) pcnt_unit_get_count(s_pcnt, &v);
    return s_pcnt_wraps + v;
}

static void pcnt_zero(void) {
    if (s_pcnt) pcnt_unit_clear_count(s_pcnt);
    s_pcnt_wraps = 0;
}

static void pcnt_setup(void) {
    pcnt_unit_config_t u = {};
    u.high_limit = BENCH_PCNT_LIMIT;
    u.low_limit = -BENCH_PCNT_LIMIT;
    u.flags.accum_count = 0;
    if (pcnt_new_unit(&u, &s_pcnt) != ESP_OK) { s_pcnt = nullptr; return; }
    pcnt_chan_config_t c = {};
    c.edge_gpio_num = BENCH_STEP_GPIO;
    c.level_gpio_num = BENCH_DIR_GPIO;
    c.flags.io_loop_back = 1;                   // keep both pads driven by their outputs
    pcnt_channel_handle_t ch = nullptr;
    if (pcnt_new_channel(s_pcnt, &c, &ch) != ESP_OK) { pcnt_del_unit(s_pcnt); s_pcnt = nullptr; return; }
    pcnt_channel_set_edge_action(ch, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_level_action(ch, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_unit_add_watch_point(s_pcnt, BENCH_PCNT_LIMIT);
    pcnt_unit_add_watch_point(s_pcnt, -BENCH_PCNT_LIMIT);
    pcnt_event_callbacks_t cb = {};
    cb.on_reach = pcnt_on_reach;
    pcnt_unit_register_event_callbacks(s_pcnt, &cb, nullptr);
    pcnt_unit_enable(s_pcnt);
    pcnt_unit_clear_count(s_pcnt);
    pcnt_unit_start(s_pcnt);
    // pcnt_new_channel ran gpio_config() on GPIO4, which re-points the pad's output at the
    // plain GPIO signal; put the MCPWM generator (group 0, operator 0, gen A = STEP0) back.
    esp_rom_gpio_connect_out_signal(BENCH_STEP_GPIO,
        mcpwm_periph_signals.groups[0].operators[0].generators[0].pwm_sig, false, false);
}

static void inhibit_outputs(void) {
    gpio_set_level(BENCH_INHIBIT_GPIO, 1);
    gpio_set_level(BENCH_ARM_GPIO, 0);
}

void r13_bench_stop(const char *reason) {
    s_heave_active = false;
    inhibit_outputs();                    // hardware first
    bool was = s_armed;
    s_armed = false;
    portENTER_CRITICAL(&s_lock);
    s_pulse.stop();
    portEXIT_CRITICAL(&s_lock);
    if (s_ops) {
        s_ops->engineStop();
        s_ops->clearTargets();
    }
    if (was) {
        s_stop_reason = reason;
        if (s_report_task) xTaskNotifyGive(s_report_task);
    }
}

bool r13_bench_owns_engine(void) { return s_armed; }

// esp_timer task context: keep short, no printing.
static void bench_tick(void *) {
    if (!s_armed) return;
    if (s_ops->loopOpen()) {             // U11 is already cleared in hardware
        r13_bench_stop("LOOP_OPEN");
        return;
    }
    portENTER_CRITICAL(&s_lock);
    bool was_active = s_pulse.active();
    int32_t d = s_pulse.due(esp_timer_get_time());
    bool now_done = was_active && !s_pulse.active();
    s_target += d;
    int32_t t = s_target;
    portEXIT_CRITICAL(&s_lock);
    if (d != 0 && !s_ops->setTarget(BENCH_AXIS, t)) {
        r13_bench_stop("TARGET_REJECTED");
        return;
    }
    // Heave sine: every 20 ticks (250 Hz) a new pose through the production IK.
    if (s_heave_active && ++s_heave_div >= 20) {
        s_heave_div = 0;
        s_heave_t += 0.004f;
        if (s_heave_t >= s_heave_end) {
            // End of the waveform: hold the last target (a composite does not end at zero and
            // a jump to 0 from +55 deg trips the runaway guard); the host brings the arm home.
            s_heave_active = false; s_heave_done = true;
            if (s_report_task) xTaskNotifyGive(s_report_task);
        } else {
            float u = 0.0f;
            for (int k = 0; k < s_wave_n; k++) u += s_wave_w[k] * sinf(6.28318530718f * s_wave_f[k] * s_heave_t + s_wave_ph[k]);
            if (u > 1.0f) u = 1.0f;
            if (u < -1.0f) u = -1.0f;
            // Amplitude ramp over the first 1.5 s: a composite with phase offsets does not start
            // at zero, and a step from home to u(0) tripped the runaway guard (2026-10-20).
            if (s_heave_t < 1.5f) u *= s_heave_t / 1.5f;
            float pos6[6] = {0, 0, (u >= 0 ? s_heave_zpos : -s_heave_zneg) * u, 0, 0, 0};
            float deg[6]; long st[6];
            r13_bench_ik(pos6, deg, st);
            long tgt = st[0];
            const long lim = (long)(BENCH_WINDOW_DEG * r13_bench_steps_per_degree());
            if (tgt > lim) tgt = lim;          // +/-60 deg arm envelope
            if (tgt < -lim) tgt = -lim;
            portENTER_CRITICAL(&s_lock);
            s_target = (int32_t)tgt; t = s_target;
            portEXIT_CRITICAL(&s_lock);
            if (!s_ops->setTarget(BENCH_AXIS, t)) { r13_bench_stop("TARGET_REJECTED"); return; }
            s_heave_poses++;
        }
    }
    // Runaway guard (2026-10-20: an engine accounting bug once drove the arm into the
    // drive's alarm): the engine must never be further from the bench target than one
    // schedule tick can explain. Soft limits check targets only, not a corrupted position.
    // In heave mode the sine may outrun the engine (250 kHz cap), so the allowance is wider
    // and the worst lag is recorded for the report.
    {
        int32_t pos = s_ops->getPosition(BENCH_AXIS);
        int32_t lag = pos > t ? pos - t : t - pos;
        if (s_heave_active && lag > s_heave_peak_lag) s_heave_peak_lag = lag;
        const int32_t allow = (int32_t)((s_heave_active ? BENCH_RUNAWAY_HEAVE_DEG : BENCH_RUNAWAY_DEG) * r13_bench_steps_per_degree());
        if (lag > allow) { r13_bench_stop("RUNAWAY"); return; }
    }
    if (now_done) {
        s_done_pending = true;
        s_run_catchup = true;
        if (s_report_task) xTaskNotifyGive(s_report_task);
    }
    if (s_run_catchup && s_ops->getPosition(BENCH_AXIS) == t) {
        s_run_catchup = false; s_run_t1 = esp_timer_get_time(); s_run_report = true;
        if (s_report_task) xTaskNotifyGive(s_report_task);
    }
}

static void report_task(void *) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const char *r = s_stop_reason;
        if (r) {
            s_stop_reason = nullptr;
            serial_printf("BENCH:STOPPED reason=%s pos=%ld\r\n", r,
                          (long)s_ops->getPosition(BENCH_AXIS));
        }
        if (s_heave_done) {
            s_heave_done = false;
            serial_printf("BENCH:HEAVE:DONE poses=%lu peak_lag=%ld pos=%ld pcnt=%ld\r\n", (unsigned long)s_heave_poses,
                          (long)s_heave_peak_lag, (long)s_ops->getPosition(BENCH_AXIS), (long)pcnt_read());
        }
        if (s_run_report) {
            s_run_report = false;
            int64_t us = s_run_t1 - s_run_t0;
            serial_printf("BENCH:ENGINE:DONE pulses=%ld elapsed_us=%lld eff_khz=%.1f\r\n", (long)s_run_pulses, (long long)us,
                          us > 0 ? (double)(s_run_pulses < 0 ? -s_run_pulses : s_run_pulses) * 1000.0 / (double)us : 0.0);
        }
        if (s_done_pending) {
            s_done_pending = false;
            vTaskDelay(pdMS_TO_TICKS(5));  // let the engine emit the last step
            serial_printf("BENCH:PULSE:DONE emitted=%ld pos=%ld target=%ld pcnt=%ld\r\n",
                          (long)s_pulse.emitted(), (long)s_ops->getPosition(BENCH_AXIS),
                          (long)s_target, (long)pcnt_read());
        }
    }
}

void r13_bench_init(const R13BenchOps *ops) {
    s_ops = ops;
    // Make the latch control pins readable so BENCH:STATUS reports the real inhibit level
    // (an output-only GPIO reads back 0 regardless of its level).
    gpio_set_direction(BENCH_INHIBIT_GPIO, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(BENCH_ARM_GPIO, GPIO_MODE_INPUT_OUTPUT);
    inhibit_outputs();
    pcnt_setup();
    xTaskCreatePinnedToCore(report_task, "r13_bench_rpt", 3072, nullptr, 3, &s_report_task, 0);
    esp_timer_create_args_t a = {};
    a.callback = bench_tick;
    a.name = "r13_bench";
    esp_timer_create(&a, &s_timer);
    esp_timer_start_periodic(s_timer, BENCH_TICK_US);
    serial_printf("BENCH:READY R13_BENCH_ONE_AXIS=ON R14_FAIL_CLOSED=ON sha=%s axis=0 (J3 MOTOR 0) "
                  "disarmed; console-only BENCH: commands (see R13BenchAxis.cpp); pcnt=%s\r\n", FW_GIT_SHA,
                  s_pcnt ? "on" : "FAILED");
}

// BENCH:STATUS. Note: the ISR cycle statistics (isr=, avg_cyc=, max_cyc=, gap_*) are
// reset by every STATUS, so they describe the interval since the previous query.
static void print_status(void) {
    uint32_t n = 0, cyc = 0, mx = 0, gmin = 0, gsum = 0;
    r13_engine_isr_gap(&gmin, &gsum);
    r13_engine_isr_stats(&n, &cyc, &mx, true);
    serial_printf("BENCH:STATUS option=R13_BENCH_ONE_AXIS sha=%s armed=%d loop=%s inhibit=%d "
                  "pos=%ld target=%ld pulse_active=%d emitted=%ld/%ld pcnt=%ld heave=%d heave_lag=%ld heave_poses=%lu isr=%lu avg_cyc=%lu max_cyc=%lu gap_avg=%lu gap_min=%lu missed=%lu\r\n",
                  FW_GIT_SHA, s_armed ? 1 : 0, s_ops->loopOpen() ? "OPEN" : "CLOSED",
                  gpio_get_level(BENCH_INHIBIT_GPIO), (long)s_ops->getPosition(BENCH_AXIS),
                  (long)s_target, s_pulse.active() ? 1 : 0, (long)s_pulse.emitted(),
                  (long)s_pulse.total(), (long)pcnt_read(), s_heave_active ? 1 : 0, (long)s_heave_peak_lag, (unsigned long)s_heave_poses, (unsigned long)n,
                  (unsigned long)(n ? cyc / n : 0), (unsigned long)mx,
                  (unsigned long)(n ? gsum / n : 0), (unsigned long)(n ? gmin : 0),
                  (unsigned long)r13_engine_isr_missed());
}

static void do_arm(void) {
    if (s_ops->loopOpen()) {
        serial_printf("BENCH:ARM:ERR stop loop open (ESTOP_MCU high): arm refused\r\n");
        return;
    }
    if (s_armed) { serial_printf("BENCH:ARM:ERR already armed\r\n"); return; }
    // Claim the engine BEFORE starting it: handleStepDirection() stops the engine whenever
    // motion is blocked and the bench does not own it, which it would do during the 300 ms
    // supervisor wait below (seen 2026-10-20: "resumed" immediately followed by "stopped").
    // Clear the schedule and the bench target BEFORE arming: the 5 kHz tick runs as soon as
    // s_armed is set and its runaway guard compares the engine position (reset to 0 by
    // engineStart) with s_target — a stale target from the previous run tripped it at ARM.
    portENTER_CRITICAL(&s_lock);
    s_pulse.stop();
    s_target = 0;
    portEXIT_CRITICAL(&s_lock);
    s_heave_active = false;
    s_armed = true;
    if (!s_ops->engineStart()) { s_armed = false; serial_printf("BENCH:ARM:ERR step engine start failed\r\n"); return; }
    s_ops->clearTargets();
    portENTER_CRITICAL(&s_lock);
    s_target = s_ops->getPosition(BENCH_AXIS);
    portEXIT_CRITICAL(&s_lock);
    pcnt_zero();

    gpio_set_level(BENCH_ARM_GPIO, 0);
    gpio_set_level(BENCH_INHIBIT_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(BENCH_ARM_WAIT_MS));
    if (s_ops->loopOpen()) {
        s_armed = false;
        inhibit_outputs();
        s_ops->engineStop();
        serial_printf("BENCH:ARM:ERR stop loop opened during supervisor wait\r\n");
        return;
    }
    gpio_set_level(BENCH_ARM_GPIO, 1);
    esp_rom_delay_us(BENCH_ARM_PULSE_US);
    gpio_set_level(BENCH_ARM_GPIO, 0);
    serial_printf("BENCH:ARM:OK pulsed GPIO48 %d us after %d ms; DRIVE_ENABLE has no readback "
                  "(needs 12 V, VCC5 >= ~4.96 V)\r\n", BENCH_ARM_PULSE_US, BENCH_ARM_WAIT_MS);
}

bool r13_bench_command(const char *cmd, int source) {
    if (strncmp(cmd, "BENCH:", 6) != 0) return false;
    const char *a = cmd + 6;
    if (!s_ops) { serial_printf("BENCH:ERR not initialized\r\n"); return true; }
    serial_printf("BENCH:CMD '%s' src=%d sha=%s\r\n", cmd, source, FW_GIT_SHA);
    if (source != COBS_SRC_SERIAL) {
        serial_printf("BENCH:ERR console only; refused from source %d\r\n", source);
        return true;
    }
    if (strcmp(a, "STOP") == 0) {
        r13_bench_stop("STOP_CMD");
        serial_printf("BENCH:STOP:OK inhibit=%d\r\n", gpio_get_level(BENCH_INHIBIT_GPIO));
        return true;
    }
    if (strcmp(a, "STATUS") == 0) { print_status(); return true; }
    if (strcmp(a, "ARM") == 0)    { do_arm(); return true; }
    long lc = -1;
    if (strncmp(a, "LATCH", 5) == 0 && strncmp(a, "LATCHDUMP", 9) != 0) {
        // BENCH:LATCH ?  -> statistics only;  BENCH:LATCH 0|1 -> disable|enable the latch accounting
        // (a query must never change the state: 2026-10-20 a test helper that polled "LATCH 0"
        // silently switched the correction off for several runs).
        if (sscanf(a, "LATCH %ld", &lc) != 1) lc = -1;
        uint32_t dneg, dpos, dbig, mpos, smax;
        bool on = r13_engine_latch_comp((int)lc);
        r13_engine_latch_stats(&dneg, &dpos, &dbig, &mpos, &smax);
        uint32_t rs, sk; r13_engine_resync_stats(&rs, &sk);
        serial_printf("BENCH:ENGINE period_cycles=%lu\r\n", (unsigned long)r13_engine_period_cycles());
        serial_printf("BENCH:LATCH comp=%d d-1=%lu d+1=%lu d_other=%lu m>0=%lu since_max=%lu resyncs=%lu skip_max=%lu\r\n", on ? 1 : 0,
                      (unsigned long)dneg, (unsigned long)dpos, (unsigned long)dbig, (unsigned long)mpos, (unsigned long)smax,
                      (unsigned long)rs, (unsigned long)sk);
        return true;
    }
    if (strcmp(a, "LATCHDUMP") == 0) {
        uint32_t n = r13_engine_latch_events(false);
        serial_printf("BENCH:LATCHDUMP events=%lu (tick since m skip d hist0)\r\n", (unsigned long)n);
        for (uint32_t k = (n > BENCH_LATCH_RING ? n - BENCH_LATCH_RING : 0); k < n; k++) {
            uint32_t o[5]; int32_t d;
            if (!r13_engine_latch_event(k, n, o, &d)) continue;
            serial_printf("BENCH:LE %lu %lu %lu %lu %ld %lu\r\n", (unsigned long)o[0], (unsigned long)o[1],
                          (unsigned long)o[2], (unsigned long)o[3], (long)d, (unsigned long)o[4]);
        }
        r13_engine_latch_events(true);
        return true;
    }
    long dly = -1;
    if (sscanf(a, "ISRDELAY %ld", &dly) == 1 && dly >= 0 && dly <= 2000) {
        r13_engine_isr_delay((uint32_t)dly);
        serial_printf("BENCH:ISRDELAY %ld cycles before the gate writes\r\n", dly);
        return true;
    }
    // BENCH:POSE x y z roll pitch yaw  (mm, deg): production IK -> axis-0 target (relative to the
    // home pose), the other five axes are reported only. BENCH:POSE? ... computes without moving.
    // Each pose is one target step for the engine, so the host streams small increments (the
    // runaway guard trips above BENCH_RUNAWAY_DEG of lag); the arm envelope is +/-60 deg.
    {
        float px[6]; bool query = (strncmp(a, "POSE?", 5) == 0);
        const char *pa = query ? a + 5 : (strncmp(a, "POSE", 4) == 0 ? a + 4 : nullptr);
        if (pa && sscanf(pa, "%f %f %f %f %f %f", &px[0], &px[1], &px[2], &px[3], &px[4], &px[5]) == 6) {
            for (int i = 3; i < 6; i++) px[i] *= 3.14159265f / 180.0f;
            float deg[6]; long st[6];
            int ok = r13_bench_ik(px, deg, st);
            const long lim = (long)(BENCH_WINDOW_DEG * r13_bench_steps_per_degree());   // 60 deg at the current gear
            bool inWin = st[0] >= -lim && st[0] <= lim;
            serial_printf("BENCH:POSE%s ik=%d axis0 %.3f deg -> %ld steps | arms %.2f %.2f %.2f %.2f %.2f %.2f deg | steps %ld %ld %ld %ld %ld %ld%s\r\n",
                          query ? "?" : "", ok, deg[0], st[0], deg[0], deg[1], deg[2], deg[3], deg[4], deg[5],
                          st[0], st[1], st[2], st[3], st[4], st[5], inWin ? "" : " OUT OF WINDOW");
            if (query) return true;
            if (!s_armed) { serial_printf("BENCH:POSE:ERR not armed\r\n"); return true; }
            if (s_pulse.active()) { serial_printf("BENCH:POSE:ERR pulse schedule busy\r\n"); return true; }
            if (!ok || !inWin) { serial_printf("BENCH:POSE:ERR refused (unreachable or outside +/-60 deg)\r\n"); return true; }
            portENTER_CRITICAL(&s_lock);
            s_target = (int32_t)st[0];
            portEXIT_CRITICAL(&s_lock);
            if (!s_ops->setTarget(BENCH_AXIS, (int32_t)st[0])) r13_bench_stop("TARGET_REJECTED");
            return true;
        }
    }
    // BENCH:WAVE <z+ mm> <z- mm> <seconds> <n> <w1 f1> ... <wn fn>  (n <= 4): composite heave,
    // u(t) = sum w_i sin(2 pi f_i t + ph_i) with the same asymmetric z scale as HEAVE.
    {
        float wz_p, wz_n, wz_s; int wn = 0; float ww[4] = {0}, wf[4] = {0};
        if (strncmp(a, "WAVE ", 5) == 0) {
            int got = sscanf(a + 5, "%f %f %f %d %f %f %f %f %f %f %f %f", &wz_p, &wz_n, &wz_s, &wn,
                             &ww[0], &wf[0], &ww[1], &wf[1], &ww[2], &wf[2], &ww[3], &wf[3]);
            if (got < 4 || wn < 1 || wn > 4 || got < 4 + 2 * wn) { serial_printf("BENCH:WAVE:ERR usage\r\n"); return true; }
            if (!s_armed) { serial_printf("BENCH:WAVE:ERR not armed\r\n"); return true; }
            if (s_pulse.active()) { serial_printf("BENCH:WAVE:ERR pulse schedule busy\r\n"); return true; }
            float wsum = 0; for (int k = 0; k < wn; k++) wsum += ww[k] < 0 ? -ww[k] : ww[k];
            if (wz_s <= 0 || wz_s > 7200 || wz_p < 0 || wz_p > 400 || wz_n > 0 || wz_n < -400 || wsum <= 0) {
                serial_printf("BENCH:WAVE:ERR range\r\n"); return true;
            }
            portENTER_CRITICAL(&s_lock);
            s_heave_zpos = wz_p; s_heave_zneg = wz_n; s_heave_t = 0.0f; s_heave_end = wz_s; s_heave_div = 0;
            s_heave_peak_lag = 0; s_heave_poses = 0; s_wave_n = wn;
            for (int k = 0; k < 4; k++) { s_wave_w[k] = (k < wn) ? ww[k] / wsum : 0.0f; s_wave_f[k] = (k < wn) ? wf[k] : 0.0f; }
            s_heave_active = true;
            portEXIT_CRITICAL(&s_lock);
            serial_printf("BENCH:WAVE:START z+%.1f z%.1f mm for %.0f s, %d components:", wz_p, wz_n, wz_s, wn);
            for (int k = 0; k < wn; k++) serial_printf(" %.2f@%.3fHz", ww[k] / wsum, wf[k]);
            serial_printf("\r\n");
            return true;
        }
    }
    // BENCH:HEAVE <z+ mm> <z- mm> <freq Hz> <seconds>: on-controller heave sine through the
    // production IK, 250 Hz pose rate, axis 0 only. Asymmetric amplitude (z+ for the positive
    // half-wave, z- for the negative) so the host can map both arm extremes. BENCH:HEAVE stop
    // ends it early. The arm envelope (+/-60 deg) is clamped in the tick.
    {
        float hz_p = 0, hz_n = 0, hz_f = 0, hz_s = 0;
        if (strncmp(a, "HEAVE", 5) == 0) {
            if (strcmp(a + 5, " stop") == 0 || strcmp(a + 5, " STOP") == 0) {
                s_heave_active = false; serial_printf("BENCH:HEAVE:STOPPED\r\n"); return true;
            }
            if (sscanf(a + 5, "%f %f %f %f", &hz_p, &hz_n, &hz_f, &hz_s) == 4) {
                if (!s_armed) { serial_printf("BENCH:HEAVE:ERR not armed\r\n"); return true; }
                if (s_pulse.active()) { serial_printf("BENCH:HEAVE:ERR pulse schedule busy\r\n"); return true; }
                if (hz_f <= 0 || hz_f > 5 || hz_s <= 0 || hz_s > 7200 || hz_p < 0 || hz_p > 400 || hz_n > 0 || hz_n < -400) {
                    serial_printf("BENCH:HEAVE:ERR usage: HEAVE <z+ 0..400> <z- -400..0> <freq 0..5> <seconds>\r\n"); return true;
                }
                portENTER_CRITICAL(&s_lock);
                s_heave_zpos = hz_p; s_heave_zneg = hz_n; s_heave_t = 0.0f;
                s_wave_n = 1; s_wave_w[0] = 1.0f; s_wave_f[0] = hz_f; s_wave_ph[0] = 0.0f;
                s_heave_end = hz_s; s_heave_div = 0; s_heave_peak_lag = 0; s_heave_poses = 0; s_heave_active = true;
                portEXIT_CRITICAL(&s_lock);
                serial_printf("BENCH:HEAVE:START z+%.1f z%.1f mm %.3f Hz for %.0f s (IK at 250 Hz)\r\n", hz_p, hz_n, hz_f, hz_s);
                return true;
            }
        }
    }
    if (strcmp(a, "ISRRATE") == 0) {
        // Cross-check the engine ISR rate against two clocks: esp_rom_delay_us (CPU cycle
        // based) and esp_timer (systimer). Engine must be running (BENCH:ARM first).
        uint32_t n0, c0, m0, n1, c1, m1;
        r13_engine_isr_stats(&n0, &c0, &m0, false);
        int64_t t0 = esp_timer_get_time();
        esp_rom_delay_us(1000);
        int64_t t1 = esp_timer_get_time();
        r13_engine_isr_stats(&n1, &c1, &m1, false);
        serial_printf("BENCH:ISRRATE rom_delay_us=1000 esp_timer_us=%lld isr=%lu avg_cyc=%lu\r\n",
                      (long long)(t1 - t0), (unsigned long)(n1 - n0),
                      (unsigned long)((n1 - n0) ? (c1 - c0) / (n1 - n0) : 0));
        return true;
    }
    if (strcmp(a, "PADSAMPLE") == 0) {
        // Self-test without a meter: sample the real pad levels (MCPWM generator has io_loop_back
        // in this build) in a tight loop. A running train shows highs and transitions on STEP0.
        const int N = 200000;
        int hi4 = 0, tr4 = 0, hi10 = 0;
        int last = gpio_get_level(BENCH_STEP_GPIO);
        int64_t t0 = esp_timer_get_time();
        for (int i = 0; i < N; i++) {
            int v = gpio_get_level(BENCH_STEP_GPIO);
            hi4 += v; if (v != last) { tr4++; last = v; }
            hi10 += gpio_get_level(BENCH_DIR_GPIO);
        }
        int64_t dt = esp_timer_get_time() - t0;
        serial_printf("BENCH:PADSAMPLE n=%d us=%lld step0_high=%d step0_transitions=%d dir0_high=%d "
                      "inhibit47=%d arm48=%d loop21=%d pcnt=%ld\r\n", N, (long long)dt, hi4, tr4, hi10,
                      gpio_get_level(BENCH_INHIBIT_GPIO), gpio_get_level(BENCH_ARM_GPIO), gpio_get_level(GPIO_NUM_21),
                      (long)pcnt_read());
        return true;
    }
    long hz = 0, n = 0;
    if (sscanf(a, "PULSE %ld %ld", &hz, &n) == 2 || sscanf(a, "PULSE:%ld:%ld", &hz, &n) == 2) {
        if (!s_armed) { serial_printf("BENCH:PULSE:ERR not armed\r\n"); return true; }
        if (s_pulse.active()) { serial_printf("BENCH:PULSE:ERR busy; BENCH:STOP first\r\n"); return true; }
        portENTER_CRITICAL(&s_lock);
        bool ok = s_pulse.start((int32_t)hz, (int32_t)n, esp_timer_get_time());
        portEXIT_CRITICAL(&s_lock);
        if (!ok) {
            serial_printf("BENCH:PULSE:ERR hz 1-%d, n +/-1-%d\r\n", R13_BENCH_MAX_HZ, R13_BENCH_MAX_COUNT);
            return true;
        }
        s_run_t0 = esp_timer_get_time(); s_run_pulses = (int32_t)n; s_run_catchup = false;
        serial_printf("BENCH:PULSE:START axis=0 hz=%ld n=%ld from pos=%ld\r\n", hz, n,
                      (long)s_ops->getPosition(BENCH_AXIS));
        return true;
    }
    serial_printf("BENCH:ERR usage: ARM | STOP | STATUS | PULSE <hz> <n> | POSE[?] x y z r p y | HEAVE z+ z- f s | "
                  "WAVE z+ z- s n (w f).. | LATCH ?|0|1 | LATCHDUMP | ISRDELAY <cyc> | ISRRATE | PADSAMPLE\r\n");
    return true;
}
