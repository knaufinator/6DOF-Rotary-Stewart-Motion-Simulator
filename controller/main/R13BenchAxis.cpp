// R13BenchAxis.cpp — R13 one-axis bench mode. See R13BenchAxis.h and
// hardware/pcb/6dof2_r13/bringup/M1_OUTPUT_TEST_PLAN.md step 12.
#include "R13BenchAxis.h"
#include "R13BenchPulse.h"
#include "CobsTransport.h"
#include "debug_uart.h"

#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#ifndef FW_GIT_SHA
#define FW_GIT_SHA "unknown"
#endif

#define BENCH_AXIS          0
#define BENCH_INHIBIT_GPIO  GPIO_NUM_47   // HIGH = inhibit (R61 default)
#define BENCH_ARM_GPIO      GPIO_NUM_48   // rising edge sets U11
#define BENCH_ARM_WAIT_MS   300           // U13 CT release ~107 ms nominal, 158 ms worst
#define BENCH_ARM_PULSE_US  50
#define BENCH_TICK_US       200           // 5 kHz schedule tick = max pulse rate

static const R13BenchOps *s_ops = nullptr;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static R13BenchPulse s_pulse;
static volatile bool s_armed = false;
static int32_t s_target = 0;
static esp_timer_handle_t s_timer = nullptr;
static TaskHandle_t s_report_task = nullptr;
static volatile bool s_done_pending = false;
static const char *volatile s_stop_reason = nullptr;

static void inhibit_outputs(void) {
    gpio_set_level(BENCH_INHIBIT_GPIO, 1);
    gpio_set_level(BENCH_ARM_GPIO, 0);
}

void r13_bench_stop(const char *reason) {
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
    if (now_done) {
        s_done_pending = true;
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
        if (s_done_pending) {
            s_done_pending = false;
            vTaskDelay(pdMS_TO_TICKS(5));  // let the engine emit the last step
            serial_printf("BENCH:PULSE:DONE emitted=%ld pos=%ld target=%ld\r\n",
                          (long)s_pulse.emitted(), (long)s_ops->getPosition(BENCH_AXIS),
                          (long)s_target);
        }
    }
}

void r13_bench_init(const R13BenchOps *ops) {
    s_ops = ops;
    inhibit_outputs();
    xTaskCreatePinnedToCore(report_task, "r13_bench_rpt", 3072, nullptr, 3, &s_report_task, 0);
    esp_timer_create_args_t a = {};
    a.callback = bench_tick;
    a.name = "r13_bench";
    esp_timer_create(&a, &s_timer);
    esp_timer_start_periodic(s_timer, BENCH_TICK_US);
    serial_printf("BENCH:READY R13_BENCH_ONE_AXIS=ON R14_FAIL_CLOSED=ON sha=%s axis=0 (J3 MOTOR 0) "
                  "disarmed; console-only BENCH:ARM/PULSE/STOP/STATUS\r\n", FW_GIT_SHA);
}

static void print_status(void) {
    serial_printf("BENCH:STATUS option=R13_BENCH_ONE_AXIS sha=%s armed=%d loop=%s inhibit=%d "
                  "pos=%ld target=%ld pulse_active=%d emitted=%ld/%ld\r\n",
                  FW_GIT_SHA, s_armed ? 1 : 0, s_ops->loopOpen() ? "OPEN" : "CLOSED",
                  gpio_get_level(BENCH_INHIBIT_GPIO), (long)s_ops->getPosition(BENCH_AXIS),
                  (long)s_target, s_pulse.active() ? 1 : 0, (long)s_pulse.emitted(),
                  (long)s_pulse.total());
}

static void do_arm(void) {
    if (s_ops->loopOpen()) {
        serial_printf("BENCH:ARM:ERR stop loop open (ESTOP_MCU high): arm refused\r\n");
        return;
    }
    if (s_armed) { serial_printf("BENCH:ARM:ERR already armed\r\n"); return; }
    if (!s_ops->engineStart()) { serial_printf("BENCH:ARM:ERR step engine start failed\r\n"); return; }
    s_ops->clearTargets();
    portENTER_CRITICAL(&s_lock);
    s_pulse.stop();
    s_target = s_ops->getPosition(BENCH_AXIS);
    portEXIT_CRITICAL(&s_lock);

    gpio_set_level(BENCH_ARM_GPIO, 0);
    gpio_set_level(BENCH_INHIBIT_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(BENCH_ARM_WAIT_MS));
    if (s_ops->loopOpen()) {
        inhibit_outputs();
        s_ops->engineStop();
        serial_printf("BENCH:ARM:ERR stop loop opened during supervisor wait\r\n");
        return;
    }
    gpio_set_level(BENCH_ARM_GPIO, 1);
    esp_rom_delay_us(BENCH_ARM_PULSE_US);
    gpio_set_level(BENCH_ARM_GPIO, 0);
    s_armed = true;
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
        serial_printf("BENCH:PULSE:START axis=0 hz=%ld n=%ld from pos=%ld\r\n", hz, n,
                      (long)s_ops->getPosition(BENCH_AXIS));
        return true;
    }
    serial_printf("BENCH:ERR usage: BENCH:ARM | BENCH:PULSE <hz> <n> | BENCH:STOP | BENCH:STATUS\r\n");
    return true;
}
