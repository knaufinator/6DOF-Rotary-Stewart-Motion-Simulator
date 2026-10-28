#include "test_framework.h"
#include "R14MotionAdmission.h"

template <bool Qualified>
static void configure_admission(r14::MotionAdmission<Qualified>& admission) {
    const r14::SeekInterlock::SeekLimits limits = {1, 2, 3, 1000};
    for (int axis = 0; axis < 6; ++axis)
        ASSERT(admission.configure_axis(axis, limits));
}

static r14::SeekInterlock::Inputs admission_inputs(uint64_t at, bool held) {
    r14::SeekInterlock::Inputs in = {};
    in.sampled_us = at;
    in.estop_closed = true;
    in.field_good = true;
    in.local_hold = held;
    in.bottom_verified = true;
    in.calibration_valid = true;
    for (int axis = 0; axis < 6; ++axis) {
        in.hall_closed[axis] = true;
        in.drive_ready[axis] = true;
    }
    return in;
}

TEST(r14_production_unqualified_type_never_admits_motion) {
    r14::MotionAdmission<false> admission(100, 100, 100);
    const r14::SeekInterlock::SeekLimits limits = {1, 2, 3, 1000};
    ASSERT(!admission.hardware_qualified());
    ASSERT(!admission.configure_axis(0, limits));
    ASSERT(!admission.submit_inputs(admission_inputs(10, false), 10));
    ASSERT(!admission.submit_inputs(admission_inputs(20, true), 20));
    ASSERT(!admission.request_operator_arm(20));
    ASSERT(!admission.request_operator_home(20));
    ASSERT(!admission.begin_seek(0, 20));
    ASSERT(!admission.reserve_seek_pulse(0, 1, 21));
    ASSERT(!admission.normal_motion_permitted());
    admission.report_alarm(4);
    ASSERT(admission.faulted());
    ASSERT_EQ(admission.alarm_mask(), 16u);
}

TEST(r14_simulated_arm_and_home_are_separate_fresh_requests) {
    r14::MotionAdmission<true> admission(100, 100, 100);
    configure_admission(admission);
    ASSERT(!admission.request_operator_arm(0));
    ASSERT(!admission.request_operator_home(0));
    ASSERT(admission.submit_inputs(admission_inputs(10, false), 10));
    ASSERT(admission.submit_inputs(admission_inputs(20, true), 20));
    ASSERT(!admission.request_operator_home(20));
    ASSERT(admission.request_operator_arm(20));
    ASSERT(!admission.begin_seek(0, 20));
    ASSERT(admission.request_operator_home(20));
    ASSERT(admission.begin_seek(0, 20));
    ASSERT(admission.reserve_seek_pulse(0, 1, 21));
    ASSERT(admission.reserve_seek_pulse(0, 1, 22));
    ASSERT_EQ(admission.reserved_pulses(0), 2u);
    r14::SeekInterlock::Inputs edge = admission_inputs(23, true);
    edge.hall_closed[0] = false;
    ASSERT(admission.submit_inputs(edge, 23));
    ASSERT(!admission.normal_motion_permitted());
}

TEST(r14_simulated_missing_hall_or_alarm_stops_all_axis_admission) {
    r14::MotionAdmission<true> missing(100, 100, 100);
    configure_admission(missing);
    ASSERT(missing.submit_inputs(admission_inputs(10, false), 10));
    ASSERT(missing.submit_inputs(admission_inputs(20, true), 20));
    ASSERT(missing.request_operator_arm(20));
    ASSERT(missing.request_operator_home(20));
    ASSERT(missing.begin_seek(2, 20));
    ASSERT(missing.begin_seek(3, 20));
    for (int n = 0; n < 3; ++n) ASSERT(missing.reserve_seek_pulse(2, 1, 21 + n));
    ASSERT(!missing.reserve_seek_pulse(2, 1, 24));
    ASSERT(missing.faulted());
    ASSERT(!missing.home_request_pending());
    ASSERT(!missing.reserve_seek_pulse(3, 1, 24));

    r14::MotionAdmission<true> alarm(100, 100, 100);
    configure_admission(alarm);
    ASSERT(alarm.submit_inputs(admission_inputs(10, false), 10));
    ASSERT(alarm.submit_inputs(admission_inputs(20, true), 20));
    ASSERT(alarm.request_operator_arm(20));
    ASSERT(alarm.request_operator_home(20));
    ASSERT(alarm.begin_seek(0, 20));
    r14::SeekInterlock::Inputs alarm_in = admission_inputs(21, true);
    alarm_in.drive_alarm[5] = true;
    ASSERT(!alarm.submit_inputs(alarm_in, 21));
    ASSERT(alarm.faulted());
    ASSERT_EQ(alarm.alarm_mask(), 32u);
    ASSERT(!alarm.reserve_seek_pulse(0, 1, 22));
}

TEST(r14_reconnect_or_reset_invalidates_pending_home) {
    r14::MotionAdmission<true> admission(100, 100, 100);
    configure_admission(admission);
    ASSERT(admission.submit_inputs(admission_inputs(10, false), 10));
    ASSERT(admission.submit_inputs(admission_inputs(20, true), 20));
    ASSERT(admission.request_operator_arm(20));
    ASSERT(admission.request_operator_home(20));
    admission.report_reconnect_or_reset();
    ASSERT(admission.faulted());
    ASSERT(!admission.home_request_pending());
    ASSERT(!admission.begin_seek(0, 21));
    ASSERT(!admission.request_operator_arm(21));
    ASSERT(!admission.normal_motion_permitted());
}

void register_r14_motion_admission_tests(void) {
    RUN_TEST(r14_production_unqualified_type_never_admits_motion);
    RUN_TEST(r14_simulated_arm_and_home_are_separate_fresh_requests);
    RUN_TEST(r14_simulated_missing_hall_or_alarm_stops_all_axis_admission);
    RUN_TEST(r14_reconnect_or_reset_invalidates_pending_home);
}
