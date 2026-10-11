#include "test_framework.h"
#include "R14DriveAlarmLatch.h"

TEST(r14_any_of_six_normalized_alarms_latches_stop) {
    for (int axis = 0; axis < 6; ++axis) {
        r14::DriveAlarmLatch latch;
        ASSERT(!latch.faulted());
        latch.assert_alarm(axis);
        ASSERT(latch.faulted());
        ASSERT_EQ(latch.axis_mask(), static_cast<uint8_t>(1u << axis));
        ASSERT(!latch.invalid_axis_event());
        latch.assert_alarm((axis + 1) % 6);
        ASSERT(latch.faulted());
        ASSERT((latch.axis_mask() & static_cast<uint8_t>(1u << axis)) != 0);
    }
}

TEST(r14_invalid_alarm_axis_fails_closed) {
    r14::DriveAlarmLatch latch;
    latch.assert_alarm(-1);
    ASSERT(latch.faulted());
    ASSERT(latch.invalid_axis_event());
    ASSERT_EQ(latch.axis_mask(), 0);
}

void register_r14_drive_alarm_latch_tests(void) {
    RUN_TEST(r14_any_of_six_normalized_alarms_latches_stop);
    RUN_TEST(r14_invalid_alarm_axis_fails_closed);
}
