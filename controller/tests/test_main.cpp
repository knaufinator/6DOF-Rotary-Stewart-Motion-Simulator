#include "test_framework.h"

/* Global counter definitions (declared extern in test_framework.h) */
int g_tests_run    = 0;
int g_tests_passed = 0;
int g_tests_failed = 0;
int g_current_test_failed = 0;
const char* g_current_test_name = "";

int main(int argc, char* argv[]) {
    (void)argc; (void)argv;

    printf("\n== Stewart Platform Unit Tests ==\n\n");

    printf("--- Inverse Kinematics ---\n");
    register_ik_tests();

    printf("\n--- Axis Scaling ---\n");
    register_axis_scaling_tests();

    printf("\n--- Motion Cueing ---\n");
    register_motion_cueing_tests();

    PRINT_RESULTS();
}
