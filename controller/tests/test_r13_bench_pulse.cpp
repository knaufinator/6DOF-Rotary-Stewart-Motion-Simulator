#include "test_framework.h"
#include "R13BenchPulse.h"

#define ASSERT_TRUE(c) ASSERT_MSG((c), #c)

// Drive the schedule with a 200 us tick (the firmware's esp_timer period).
static int32_t run(R13BenchPulse &p, int64_t t0, int64_t span_us, int *max_per_tick) {
    int32_t sum = 0;
    *max_per_tick = 0;
    for (int64_t t = t0; t <= t0 + span_us; t += 200) {
        int32_t d = p.due(t);
        int a = d < 0 ? -d : d;
        if (a > *max_per_tick) *max_per_tick = a;
        sum += d;
    }
    return sum;
}

TEST(r13_bench_pulse_rejects_out_of_range) {
    R13BenchPulse p;
    ASSERT_TRUE(!p.start(0, 10, 0));
    ASSERT_TRUE(!p.start(5001, 10, 0));
    ASSERT_TRUE(!p.start(1000, 0, 0));
    ASSERT_TRUE(!p.start(1000, 10001, 0));
    ASSERT_TRUE(!p.start(1000, -10001, 0));
    ASSERT_TRUE(!p.active());
    ASSERT_TRUE(p.start(5000, 10000, 0));
    ASSERT_TRUE(p.start(1, -10000, 0));
}

TEST(r13_bench_pulse_emits_exact_count_and_stops) {
    R13BenchPulse p;
    int mx;
    ASSERT_TRUE(p.start(1000, 10, 1000));
    int32_t sum = run(p, 1000, 1000000, &mx);
    ASSERT_EQ(sum, 10);
    ASSERT_TRUE(!p.active());
    ASSERT_EQ(p.due(5000000), 0);              // nothing after completion
    ASSERT_TRUE(p.start(5000, -10000, 0));
    sum = run(p, 0, 3000000, &mx);
    ASSERT_EQ(sum, -10000);
    ASSERT_TRUE(mx <= 1);                      // 5 kHz at a 200 us tick: one step per tick
}

TEST(r13_bench_pulse_rate_matches_hz) {
    R13BenchPulse p;
    int mx;
    ASSERT_TRUE(p.start(2000, 10000, 0));
    int32_t half = run(p, 0, 1000000, &mx);    // first second
    ASSERT_TRUE(half >= 1999 && half <= 2002);
}

TEST(r13_bench_pulse_stop_halts_immediately) {
    R13BenchPulse p;
    int mx;
    ASSERT_TRUE(p.start(1000, 1000, 0));
    run(p, 0, 100000, &mx);
    p.stop();
    ASSERT_EQ(p.due(900000), 0);
    ASSERT_TRUE(!p.active());
}

void register_r13_bench_pulse_tests(void) {
    RUN_TEST(r13_bench_pulse_rejects_out_of_range);
    RUN_TEST(r13_bench_pulse_emits_exact_count_and_stops);
    RUN_TEST(r13_bench_pulse_rate_matches_hz);
    RUN_TEST(r13_bench_pulse_stop_halts_immediately);
}
