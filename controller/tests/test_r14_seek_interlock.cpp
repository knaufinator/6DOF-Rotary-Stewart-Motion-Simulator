#include "test_framework.h"
#include "R14SeekInterlock.h"

using r14::SeekInterlock;

static SeekInterlock::Inputs healthy_inputs(uint64_t at, bool local_hold) {
    SeekInterlock::Inputs in = {};
    in.sampled_us = at;
    in.estop_closed = true;
    in.field_good = true;
    in.local_hold = local_hold;
    in.bottom_verified = true; // latched proof of this startup sector
    in.calibration_valid = true;
    for (int i = 0; i < 6; ++i) {
        in.hall_closed[i] = true;
        in.drive_ready[i] = true;
        in.drive_alarm[i] = false;
    }
    return in;
}

static void configure_six(SeekInterlock& gate) {
    const SeekInterlock::SeekLimits limits = {1, 2, 3, 1000};
    for (int i = 0; i < 6; ++i) ASSERT(gate.configure_axis(i, limits));
}

static bool arm_from_fresh_operator_request(SeekInterlock& gate) {
    return gate.observe(healthy_inputs(20, false), 20) && gate.request_arm(20);
}

TEST(r14_seek_requires_explicit_request_and_all_six_configurations) {
    SeekInterlock gate(100, 50);
    const SeekInterlock::Inputs held = healthy_inputs(10, true);
    ASSERT(gate.observe(held, 10));
    ASSERT(!gate.request_arm(10)); // no calibrated axis configuration; sampling never arms
    ASSERT(gate.observe(healthy_inputs(20, false), 20));
    ASSERT(gate.observe(healthy_inputs(30, true), 30));
    ASSERT(!gate.request_arm(30)); // no axis calibration loaded
    configure_six(gate);
    ASSERT(gate.request_arm(30));
    ASSERT_EQ(gate.state(), SeekInterlock::ARMED_FOR_SEEK);
    ASSERT(!gate.normal_motion_permitted());
    ASSERT(!gate.request_arm(30));
    ASSERT(!gate.begin_seek(0, 81)); // fresh inputs cannot extend an expired request
}

TEST(r14_seek_open_hall_or_unproven_bottom_blocks_boot_arm) {
    SeekInterlock gate(100, 50);
    configure_six(gate);
    ASSERT(gate.observe(healthy_inputs(10, false), 10));
    SeekInterlock::Inputs open = healthy_inputs(20, true);
    open.hall_closed[4] = false;
    ASSERT(gate.observe(open, 20));
    ASSERT(!gate.request_arm(20));
    ASSERT_EQ(gate.state(), SeekInterlock::DISARMED);
    ASSERT(gate.observe(healthy_inputs(30, true), 30));
    ASSERT(gate.request_arm(30)); // new explicit request, no physical hold
    SeekInterlock unknown_gate(100, 50);
    configure_six(unknown_gate);

    SeekInterlock::Inputs unknown_bottom = healthy_inputs(50, true);
    unknown_bottom.bottom_verified = false;
    ASSERT(unknown_gate.observe(unknown_bottom, 50));
    ASSERT(!unknown_gate.request_arm(50));
}

TEST(r14_seek_one_drive_alarm_latches_global_stop) {
    for (int alarm_axis = 0; alarm_axis < 6; ++alarm_axis) {
        SeekInterlock gate(100, 50);
        configure_six(gate);
        ASSERT(arm_from_fresh_operator_request(gate));
        ASSERT(gate.begin_seek(0, 20));
        ASSERT(gate.reserve_seek_pulse(0, 1, 21));
        SeekInterlock::Inputs alarm = healthy_inputs(22, true);
        alarm.drive_alarm[alarm_axis] = true;
        ASSERT(!gate.observe(alarm, 22));
        ASSERT_EQ(gate.state(), SeekInterlock::GLOBAL_FAULT);
        ASSERT_EQ(gate.fault(), SeekInterlock::DRIVE_FAULT);
        ASSERT_EQ(gate.fault_axis(), alarm_axis);
        ASSERT(!gate.reserve_seek_pulse(0, 1, 23));
        ASSERT(!gate.begin_seek(1, 23));
        ASSERT(!gate.normal_motion_permitted());
    }
}

TEST(r14_seek_missing_or_early_hall_edge_is_bounded) {
    SeekInterlock missing(100, 50);
    configure_six(missing);
    ASSERT(arm_from_fresh_operator_request(missing));
    ASSERT(missing.begin_seek(2, 20));
    ASSERT(missing.reserve_seek_pulse(2, 1, 21));
    ASSERT(missing.reserve_seek_pulse(2, 1, 22));
    ASSERT(missing.reserve_seek_pulse(2, 1, 23));
    ASSERT_EQ(missing.reserved_pulses(2), 3u);
    ASSERT(!missing.reserve_seek_pulse(2, 1, 24));
    ASSERT_EQ(missing.fault(), SeekInterlock::SEEK_PULSE_LIMIT);
    ASSERT_EQ(missing.fault_axis(), 2);

    SeekInterlock early(100, 50);
    configure_six(early);
    ASSERT(arm_from_fresh_operator_request(early));
    ASSERT(early.begin_seek(1, 20));
    ASSERT(early.reserve_seek_pulse(1, 1, 21));
    SeekInterlock::Inputs opened = healthy_inputs(22, true);
    opened.hall_closed[1] = false;
    ASSERT(!early.observe(opened, 22));
    ASSERT_EQ(early.fault(), SeekInterlock::HALL_TOO_EARLY);
}

TEST(r14_seek_all_edges_captured_does_not_establish_home) {
    SeekInterlock gate(100, 50);
    configure_six(gate);
    ASSERT(arm_from_fresh_operator_request(gate));
    for (int i = 0; i < 6; ++i) {
        ASSERT(gate.begin_seek(i, 20));
        ASSERT(gate.reserve_seek_pulse(i, 1, 21));
        ASSERT(gate.reserve_seek_pulse(i, 1, 22));
    }
    SeekInterlock::Inputs all_open = healthy_inputs(23, true);
    for (int i = 0; i < 6; ++i) all_open.hall_closed[i] = false;
    ASSERT(gate.observe(all_open, 23));
    ASSERT_EQ(gate.state(), SeekInterlock::ALL_EDGES_CAPTURED);
    ASSERT(!gate.normal_motion_permitted());
    ASSERT(!gate.reserve_seek_pulse(0, 1, 24));
}

TEST(r14_seek_stale_input_field_loss_and_timeout_fault) {
    SeekInterlock stale(5, 50);
    configure_six(stale);
    ASSERT(arm_from_fresh_operator_request(stale));
    ASSERT(stale.begin_seek(0, 20));
    ASSERT(!stale.reserve_seek_pulse(0, 1, 26));
    ASSERT_EQ(stale.fault(), SeekInterlock::CLOCK_OR_INPUT_STALE);

    SeekInterlock released(100, 50);
    configure_six(released);
    ASSERT(arm_from_fresh_operator_request(released));
    ASSERT(released.observe(healthy_inputs(21, false), 21)); // no hold-to-run
    auto bad_field = healthy_inputs(22, false);
    bad_field.field_good = false;
    ASSERT(!released.observe(bad_field, 22));
    ASSERT_EQ(released.fault(), SeekInterlock::INPUT_LOST);

    SeekInterlock timed(100, 50);
    configure_six(timed);
    ASSERT(arm_from_fresh_operator_request(timed));
    ASSERT(timed.begin_seek(3, 20));
    ASSERT(!timed.observe(healthy_inputs(1020, true), 1020));
    ASSERT_EQ(timed.fault(), SeekInterlock::SEEK_TIME_LIMIT);
}

TEST(r14_seek_wrong_direction_and_unexpected_other_hall_fault) {
    SeekInterlock wrong(100, 50);
    configure_six(wrong);
    ASSERT(arm_from_fresh_operator_request(wrong));
    ASSERT(wrong.begin_seek(0, 20));
    ASSERT(!wrong.reserve_seek_pulse(0, -1, 21));
    ASSERT_EQ(wrong.fault(), SeekInterlock::WRONG_DIRECTION);

    SeekInterlock other(100, 50);
    configure_six(other);
    ASSERT(arm_from_fresh_operator_request(other));
    ASSERT(other.begin_seek(0, 20));
    SeekInterlock::Inputs opened = healthy_inputs(21, true);
    opened.hall_closed[4] = false;
    ASSERT(!other.observe(opened, 21));
    ASSERT_EQ(other.fault(), SeekInterlock::HALL_UNEXPECTED);
    ASSERT_EQ(other.fault_axis(), 4);
}

TEST(r14_seek_hall_edge_at_deadline_faults_all_axes) {
    SeekInterlock late(100, 50);
    configure_six(late);
    ASSERT(arm_from_fresh_operator_request(late));
    for (int axis = 0; axis < SeekInterlock::kAxes; ++axis) {
        ASSERT(late.begin_seek(axis, 20));
        ASSERT(late.reserve_seek_pulse(axis, 1, 21));
        ASSERT(late.reserve_seek_pulse(axis, 1, 22));
    }
    SeekInterlock::Inputs opened = healthy_inputs(1020, true);
    for (int axis = 0; axis < SeekInterlock::kAxes; ++axis)
        opened.hall_closed[axis] = false;
    ASSERT(!late.observe(opened, 1020));
    ASSERT_EQ(late.state(), SeekInterlock::GLOBAL_FAULT);
    ASSERT_EQ(late.fault(), SeekInterlock::SEEK_TIME_LIMIT);
    ASSERT_EQ(late.fault_axis(), 0);

    SeekInterlock on_time(100, 50);
    configure_six(on_time);
    ASSERT(arm_from_fresh_operator_request(on_time));
    for (int axis = 0; axis < SeekInterlock::kAxes; ++axis) {
        ASSERT(on_time.begin_seek(axis, 20));
        ASSERT(on_time.reserve_seek_pulse(axis, 1, 21));
        ASSERT(on_time.reserve_seek_pulse(axis, 1, 22));
    }
    opened.sampled_us = 1019;
    ASSERT(on_time.observe(opened, 1019));
    ASSERT_EQ(on_time.state(), SeekInterlock::ALL_EDGES_CAPTURED);
}

void register_r14_seek_interlock_tests(void) {
    RUN_TEST(r14_seek_requires_explicit_request_and_all_six_configurations);
    RUN_TEST(r14_seek_open_hall_or_unproven_bottom_blocks_boot_arm);
    RUN_TEST(r14_seek_one_drive_alarm_latches_global_stop);
    RUN_TEST(r14_seek_missing_or_early_hall_edge_is_bounded);
    RUN_TEST(r14_seek_all_edges_captured_does_not_establish_home);
    RUN_TEST(r14_seek_stale_input_field_loss_and_timeout_fault);
    RUN_TEST(r14_seek_wrong_direction_and_unexpected_other_hall_fault);
    RUN_TEST(r14_seek_hall_edge_at_deadline_faults_all_axes);
}
