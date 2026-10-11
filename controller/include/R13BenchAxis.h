// R13BenchAxis.h — R13 one-axis bench mode (build option R13_BENCH_ONE_AXIS).
//
// Implements M1_OUTPUT_TEST_PLAN.md step 12 (hardware/pcb/6dof2_r13/bringup/):
// R14_FAIL_CLOSED stays ON, boot stays disarmed, every network motion path is
// unchanged, and three console-only commands drive axis 0 (bench "motor 1",
// DB25 J3 silkscreen MOTOR 0) through the production step engine:
//
//   BENCH:ARM              refuses unless ESTOP_MCU (GPIO21) is low; GPIO47 low,
//                          300 ms supervisor wait, 50 us pulse on GPIO48.
//                          DRIVE_ENABLE has no readback.
//   BENCH:PULSE <hz> <n>   |n| pulses at hz on axis 0, sign of n = direction;
//                          hz <= 5000, |n| <= 10000. Requires BENCH:ARM.
//   BENCH:STOP             GPIO47 high, engine stop, targets cleared.
//   BENCH:STATUS           option state, firmware SHA, armed, loop, axis-0 position.
//
// Accepted from the USB console only (COBS_SRC_SERIAL); refused from Ethernet,
// Wi-Fi and BLE. Transport reconnect, stop-loop opening and drive-alarm reports
// stop the bench through the existing r14_report_* paths.
#ifndef R13_BENCH_AXIS_H
#define R13_BENCH_AXIS_H

#include <stdint.h>

struct R13BenchOps {
    bool    (*setTarget)(int axis, int32_t pos);
    int32_t (*getPosition)(int axis);
    bool    (*engineStart)(void);     // start or resume the shared step engine
    void    (*engineStop)(void);      // stop pulse generation, STEP pins low
    void    (*clearTargets)(void);    // all targets = current positions
    bool    (*loopOpen)(void);        // raw ESTOP_MCU, true = open (stop)
};

void r13_bench_init(const R13BenchOps *ops);
// Returns true if the command was a BENCH: command (handled or refused).
bool r13_bench_command(const char *cmd, int source);
bool r13_bench_owns_engine(void);
void r13_bench_stop(const char *reason);   // safe from any task

#endif // R13_BENCH_AXIS_H
