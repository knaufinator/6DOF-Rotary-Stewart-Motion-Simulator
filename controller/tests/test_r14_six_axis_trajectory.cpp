#include "test_framework.h"
#include "R14SixAxisTrajectory.h"
#include <climits>
#include <limits>

using r14::SixAxisTrajectory;

static void fixture(SixAxisTrajectory::AxisLimits limits[6],
                    int32_t start[6], int32_t target[6]) {
    for (int i = 0; i < 6; ++i) {
        limits[i] = {-10000, 10000, 1000.0, 2000.0, 4000.0};
        start[i] = 0;
        target[i] = (i % 2 ? -1 : 1) * (i + 1) * 100;
    }
}

TEST(r14_common_clock_reaches_six_targets_without_overshoot) {
    SixAxisTrajectory p;
    SixAxisTrajectory::AxisLimits limits[6];
    int32_t start[6], target[6], out[6] = {};
    fixture(limits, start, target);
    const uint64_t t0 = 1000000;
    ASSERT(p.begin(start, target, limits, t0, 10000, 100000000));
    ASSERT_EQ(p.state(), SixAxisTrajectory::RUNNING);
    const uint64_t duration = p.duration_us();
    int32_t previous[6] = {};
    for (uint64_t elapsed = 0; elapsed < duration; elapsed += 5000) {
        ASSERT(p.sample(t0 + elapsed, out));
        for (int i = 0; i < 6; ++i) {
            ASSERT(out[i] >= limits[i].min_count && out[i] <= limits[i].max_count);
            ASSERT(out[i] >= (start[i] < target[i] ? start[i] : target[i]));
            ASSERT(out[i] <= (start[i] > target[i] ? start[i] : target[i]));
            ASSERT(target[i] >= start[i] ? out[i] >= previous[i]
                                         : out[i] <= previous[i]);
            previous[i] = out[i];
        }
    }
    ASSERT(p.sample(t0 + duration, out));
    ASSERT_EQ(p.state(), SixAxisTrajectory::COMPLETE);
    for (int i = 0; i < 6; ++i) ASSERT_EQ(out[i], target[i]);
}

TEST(r14_duration_obeys_speed_acceleration_and_jerk_limits) {
    SixAxisTrajectory p;
    SixAxisTrajectory::AxisLimits limits[6];
    int32_t start[6], target[6];
    fixture(limits, start, target);
    target[0] = 800;
    limits[0].max_speed_steps_s = 900.0;
    limits[0].max_accel_steps_s2 = 1500.0;
    limits[0].max_jerk_steps_s3 = 2800.0;
    ASSERT(p.begin(start, target, limits, 0, 5000, 100000000));
    const double t = static_cast<double>(p.duration_us()) / 1000000.0;
    for (int i = 0; i < 6; ++i) {
        const double distance = fabs(static_cast<double>(target[i]) - start[i]);
        ASSERT(2.1875 * distance / t <= limits[i].max_speed_steps_s + 1e-8);
        ASSERT(8.0 * distance / (t * t) <= limits[i].max_accel_steps_s2 + 1e-8);
        ASSERT(52.5 * distance / (t * t * t) <= limits[i].max_jerk_steps_s3 + 1e-8);
    }
    // A common duration follows the slowest axis; short axes cannot finish early.
    int32_t out[6];
    for (uint64_t at = 5000; at <= p.duration_us() / 2; at += 5000)
        ASSERT(p.sample(at, out));
    for (int i = 0; i < 6; ++i) ASSERT(out[i] != target[i]);
}

TEST(r14_rejects_bad_limits_and_out_of_range_targets) {
    SixAxisTrajectory::AxisLimits limits[6];
    int32_t start[6], target[6];
    fixture(limits, start, target);
    limits[4].max_jerk_steps_s3 = std::numeric_limits<double>::quiet_NaN();
    SixAxisTrajectory bad;
    ASSERT(!bad.begin(start, target, limits, 0, 1000, 10000000));
    ASSERT_EQ(bad.error(), SixAxisTrajectory::BAD_CONFIG);
    ASSERT_EQ(bad.state(), SixAxisTrajectory::FAULT);
    fixture(limits, start, target);
    target[5] = 10001;
    SixAxisTrajectory beyond;
    ASSERT(!beyond.begin(start, target, limits, 0, 1000, 10000000));
    ASSERT_EQ(beyond.error(), SixAxisTrajectory::OUT_OF_RANGE);
    fixture(limits, start, target);
    SixAxisTrajectory no_time;
    ASSERT(!no_time.begin(start, target, limits, 0, 1000, 1));
    ASSERT_EQ(no_time.error(), SixAxisTrajectory::DURATION_EXCEEDED);
}

TEST(r14_late_or_reversed_clock_latches_fault_without_output) {
    SixAxisTrajectory::AxisLimits limits[6];
    int32_t start[6], target[6], out[6];
    fixture(limits, start, target);
    SixAxisTrajectory late;
    ASSERT(late.begin(start, target, limits, 100, 1000, 100000000));
    for (int i = 0; i < 6; ++i) out[i] = 12345;
    ASSERT(!late.sample(1101, out));
    ASSERT_EQ(late.error(), SixAxisTrajectory::UPDATE_MISSED);
    for (int i = 0; i < 6; ++i) ASSERT_EQ(out[i], 12345);
    ASSERT(!late.begin(start, target, limits, 1200, 1000, 100000000));
    SixAxisTrajectory reversed;
    ASSERT(reversed.begin(start, target, limits, 100, 1000, 100000000));
    ASSERT(!reversed.sample(99, out));
    ASSERT_EQ(reversed.error(), SixAxisTrajectory::CLOCK_REVERSED);
}

TEST(r14_cancel_is_latched_and_zero_distance_is_stationary) {
    SixAxisTrajectory::AxisLimits limits[6];
    int32_t start[6], target[6], out[6];
    fixture(limits, start, target);
    SixAxisTrajectory cancelled;
    ASSERT(cancelled.begin(start, target, limits, 0, 1000, 100000000));
    cancelled.abort();
    ASSERT_EQ(cancelled.state(), SixAxisTrajectory::FAULT);
    ASSERT_EQ(cancelled.error(), SixAxisTrajectory::ABORTED);
    ASSERT(!cancelled.sample(1, out));
    for (int i = 0; i < 6; ++i) target[i] = start[i];
    SixAxisTrajectory stationary;
    ASSERT(stationary.begin(start, target, limits, 0, 1000, 100000000));
    ASSERT_EQ(stationary.state(), SixAxisTrajectory::COMPLETE);
    ASSERT_EQ(stationary.duration_us(), 0);
    ASSERT(stationary.sample(0, out));
    for (int i = 0; i < 6; ++i) ASSERT_EQ(out[i], start[i]);
}

TEST(r14_extreme_signed_counts_do_not_overflow) {
    SixAxisTrajectory::AxisLimits limits[6];
    int32_t start[6], target[6], out[6];
    fixture(limits, start, target);
    limits[0] = {INT32_MIN, INT32_MAX, 1000000000.0, 1000000000.0, 1000000000.0};
    start[0] = INT32_MIN;
    target[0] = INT32_MAX;
    SixAxisTrajectory p;
    ASSERT(p.begin(start, target, limits, 0, 10000, 100000000));
    ASSERT(p.sample(0, out));
    ASSERT_EQ(out[0], INT32_MIN);
    const uint64_t half = p.duration_us() / 2;
    // Keep the update watchdog live while advancing to the midpoint.
    for (uint64_t at = 5000; at <= half; at += 5000) ASSERT(p.sample(at, out));
    ASSERT(out[0] > INT32_MIN && out[0] < INT32_MAX);
}

void register_r14_trajectory_tests(void) {
    RUN_TEST(r14_common_clock_reaches_six_targets_without_overshoot);
    RUN_TEST(r14_duration_obeys_speed_acceleration_and_jerk_limits);
    RUN_TEST(r14_rejects_bad_limits_and_out_of_range_targets);
    RUN_TEST(r14_late_or_reversed_clock_latches_fault_without_output);
    RUN_TEST(r14_cancel_is_latched_and_zero_distance_is_stationary);
    RUN_TEST(r14_extreme_signed_counts_do_not_overflow);
}
