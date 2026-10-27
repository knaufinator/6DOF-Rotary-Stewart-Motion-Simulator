#include "test_framework.h"
#include "StepQuiescence.h"
#include "DriveScaleValidation.h"
#include <limits>

TEST(step_stop_requires_three_actually_low_operator_periods) {
    StepQuiescence q;
    ASSERT(q.allow_new_steps());
    q.request_stop();
    ASSERT(!q.allow_new_steps());
    // The task can request a stop after this period already released a gate.
    q.period_complete(true);
    ASSERT(!q.drained());
    q.period_complete(false);
    q.period_complete(false);
    ASSERT(!q.drained());
    q.period_complete(false);
    ASSERT(q.drained());
    for (int n = 0; n < 100; ++n) {
        ASSERT(!q.allow_new_steps());
        q.period_complete();
    }
}

TEST(step_resume_has_quiet_guard_and_can_be_stopped_again) {
    StepQuiescence q;
    for (int cycle = 0; cycle < 100; ++cycle) {
        q.request_stop();
        for (int n = 0; n < 3; ++n) q.period_complete();
        ASSERT(q.drained());
        q.prepare_resume();
        ASSERT(!q.drained());
        for (int n = 0; n < 3; ++n) {
            ASSERT(!q.allow_new_steps());
            q.period_complete();
        }
        ASSERT(q.allow_new_steps());
    }
}
TEST(drive_scale_rejects_nonfinite_overflow_and_invalid_persisted_values) {
    ASSERT(valid_drive_scale(2500,2.0f,50.0f));
    ASSERT(valid_drive_scale(2500,1.0f,50.0f));
    ASSERT(!valid_drive_scale(2500,0.0f,50.0f));
    ASSERT(!valid_drive_scale(2500,1e-20f,50.0f));
    ASSERT(!valid_drive_scale(0,2.0f,50.0f));
    ASSERT(!valid_drive_scale(2500,std::numeric_limits<float>::quiet_NaN(),50.0f));
    ASSERT(!valid_drive_scale(2500,2.0f,std::numeric_limits<float>::infinity()));
}

void register_step_quiescence_tests() {
    RUN_TEST(step_stop_requires_three_actually_low_operator_periods);
    RUN_TEST(step_resume_has_quiet_guard_and_can_be_stopped_again);
    RUN_TEST(drive_scale_rejects_nonfinite_overflow_and_invalid_persisted_values);
}
