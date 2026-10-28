#include "test_framework.h"
#include "EStopSafety.h"

TEST(estop_blocks_motion_for_every_unsafe_input_combination) {
    for (int paused = 0; paused <= 1; ++paused) {
        for (int live = 0; live <= 1; ++live) {
            for (int debounced = 0; debounced <= 1; ++debounced) {
                ASSERT_EQ(estop_motion_blocked(paused, live, debounced),
                          (paused || live || debounced) != 0);
            }
        }
    }
}

TEST(estop_reset_requires_both_input_states_released) {
    for (int live = 0; live <= 1; ++live) {
        for (int debounced = 0; debounced <= 1; ++debounced) {
            ASSERT_EQ(estop_reset_allowed(live, debounced),
                      (!live && !debounced));
        }
    }
}

void register_estop_safety_tests(void) {
    RUN_TEST(estop_blocks_motion_for_every_unsafe_input_combination);
    RUN_TEST(estop_reset_requires_both_input_states_released);
}
