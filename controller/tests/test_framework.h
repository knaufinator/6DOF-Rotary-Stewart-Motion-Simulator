#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

/*
 * Minimal C/C++ test framework — no external dependencies.
 * Usage:
 *   TEST(test_name) { ASSERT(...); ASSERT_NEAR(a, b, tol); }
 *   In main(): RUN_TEST(test_name);
 *              PRINT_RESULTS();
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

extern int g_tests_run;
extern int g_tests_passed;
extern int g_tests_failed;
extern int g_current_test_failed;
extern const char* g_current_test_name;

#define TEST(name) static void test_##name(void)

#define RUN_TEST(name) do { \
    g_tests_run++; \
    g_current_test_failed = 0; \
    g_current_test_name = #name; \
    printf("  [RUN ] %s\n", #name); \
    test_##name(); \
    if (g_current_test_failed) { \
        g_tests_failed++; \
        printf("  [FAIL] %s\n", #name); \
    } else { \
        g_tests_passed++; \
        printf("  [PASS] %s\n", #name); \
    } \
} while(0)

#define ASSERT(cond) do { \
    if (!(cond)) { \
        printf("    ASSERT FAILED: %s  (%s:%d)\n", #cond, __FILE__, __LINE__); \
        g_current_test_failed = 1; \
        return; \
    } \
} while(0)

#define ASSERT_MSG(cond, msg) do { \
    if (!(cond)) { \
        printf("    ASSERT FAILED: %s — %s  (%s:%d)\n", #cond, msg, __FILE__, __LINE__); \
        g_current_test_failed = 1; \
        return; \
    } \
} while(0)

#define ASSERT_NEAR(a, b, tol) do { \
    float _a = (float)(a), _b = (float)(b), _t = (float)(tol); \
    if (fabsf(_a - _b) > _t) { \
        printf("    ASSERT_NEAR FAILED: %s=%.6f vs %s=%.6f  (tol=%.6f)  (%s:%d)\n", \
               #a, _a, #b, _b, _t, __FILE__, __LINE__); \
        g_current_test_failed = 1; \
        return; \
    } \
} while(0)

#define ASSERT_GT(a, b) do { \
    float _a = (float)(a), _b = (float)(b); \
    if (!(_a > _b)) { \
        printf("    ASSERT_GT FAILED: %s=%.6f not > %s=%.6f  (%s:%d)\n", \
               #a, _a, #b, _b, __FILE__, __LINE__); \
        g_current_test_failed = 1; \
        return; \
    } \
} while(0)

#define ASSERT_LT(a, b) do { \
    float _a = (float)(a), _b = (float)(b); \
    if (!(_a < _b)) { \
        printf("    ASSERT_LT FAILED: %s=%.6f not < %s=%.6f  (%s:%d)\n", \
               #a, _a, #b, _b, __FILE__, __LINE__); \
        g_current_test_failed = 1; \
        return; \
    } \
} while(0)

#define ASSERT_EQ(a, b) do { \
    if ((a) != (b)) { \
        printf("    ASSERT_EQ FAILED: %s != %s  (%s:%d)\n", #a, #b, __FILE__, __LINE__); \
        g_current_test_failed = 1; \
        return; \
    } \
} while(0)

#define PRINT_RESULTS() do { \
    printf("\n========================================\n"); \
    printf("  %d tests run, %d passed, %d failed\n", g_tests_run, g_tests_passed, g_tests_failed); \
    printf("========================================\n"); \
    return (g_tests_failed > 0) ? 1 : 0; \
} while(0)

/* Declare test registration functions from each test file */
void register_ik_tests(void);
void register_axis_scaling_tests(void);
void register_motion_cueing_tests(void);

#endif /* TEST_FRAMEWORK_H */
