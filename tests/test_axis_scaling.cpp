#include "test_framework.h"
#include "AxisScaling.h"
#include "InverseKinematics.h"

/* ── Helpers ──────────────────────────────────────────────────────── */

static StewartConfig default_config(void) {
    StewartConfig cfg;
    initDefaultStewartConfig(&cfg);
    return cfg;
}

/* ── Tests ────────────────────────────────────────────────────────── */

TEST(as_center_maps_to_zero_12bit) {
    /* Raw center value (2047) on 12-bit (max_raw=4094) should map to 0 */
    StewartConfig cfg = default_config();
    AxisScaleConfig sc;
    computeAxisScalesFromGeometry(&sc, &cfg, 0.90f);

    float raw[6] = {2047, 2047, 2047, 2047, 2047, 2047};
    float pos[6];
    mapRawToPosition(raw, &sc, 4094.0f, pos);
    for (int i = 0; i < 6; i++) {
        ASSERT_NEAR(pos[i], 0.0f, 0.01f);
    }
}

TEST(as_center_maps_to_zero_8bit) {
    /* 8-bit: center=127, max_raw=255 */
    StewartConfig cfg = default_config();
    AxisScaleConfig sc;
    computeAxisScalesFromGeometry(&sc, &cfg, 0.90f);

    float raw[6] = {127, 127, 127, 127, 127, 127};
    float pos[6];
    mapRawToPosition(raw, &sc, 255.0f, pos);
    for (int i = 0; i < 6; i++) {
        ASSERT_NEAR(pos[i], 0.0f, 0.01f);
    }
}

TEST(as_center_maps_to_zero_16bit) {
    /* 16-bit: center=32767, max_raw=65535 */
    StewartConfig cfg = default_config();
    AxisScaleConfig sc;
    computeAxisScalesFromGeometry(&sc, &cfg, 0.90f);

    float raw[6] = {32767, 32767, 32767, 32767, 32767, 32767};
    float pos[6];
    mapRawToPosition(raw, &sc, 65535.0f, pos);
    for (int i = 0; i < 6; i++) {
        ASSERT_NEAR(pos[i], 0.0f, 0.01f);
    }
}

TEST(as_extremes_map_to_scale) {
    /* raw=0 → -scale, raw=max → +scale (approx) */
    StewartConfig cfg = default_config();
    AxisScaleConfig sc;
    computeAxisScalesFromGeometry(&sc, &cfg, 0.90f);

    float raw_min[6] = {0, 0, 0, 0, 0, 0};
    float raw_max[6] = {4094, 4094, 4094, 4094, 4094, 4094};
    float pos_min[6], pos_max[6];
    mapRawToPosition(raw_min, &sc, 4094.0f, pos_min);
    mapRawToPosition(raw_max, &sc, 4094.0f, pos_max);

    for (int i = 0; i < 3; i++) {
        /* Translation axes: output should be approximately ±scale in mm */
        ASSERT_LT(pos_min[i], 0.0f);
        ASSERT_GT(pos_max[i], 0.0f);
        ASSERT_NEAR(pos_min[i], -pos_max[i], 1.0f);
    }
    for (int i = 3; i < 6; i++) {
        /* Rotation axes: output is in radians, should be ±scale*pi/180 */
        ASSERT_LT(pos_min[i], 0.0f);
        ASSERT_GT(pos_max[i], 0.0f);
        ASSERT_NEAR(pos_min[i], -pos_max[i], 0.02f);
    }
}

TEST(as_angle_conversion) {
    /* Rotation axes should output radians, not degrees */
    StewartConfig cfg = default_config();
    AxisScaleConfig sc;
    computeAxisScalesFromGeometry(&sc, &cfg, 0.90f);

    /* is_angle flags should be set correctly */
    ASSERT_EQ(sc.is_angle[0], 0);
    ASSERT_EQ(sc.is_angle[1], 0);
    ASSERT_EQ(sc.is_angle[2], 0);
    ASSERT_EQ(sc.is_angle[3], 1);
    ASSERT_EQ(sc.is_angle[4], 1);
    ASSERT_EQ(sc.is_angle[5], 1);

    /* Max rotation output should be < 90° in radians (< 1.57 rad) */
    float raw_max[6] = {4094, 4094, 4094, 4094, 4094, 4094};
    float pos[6];
    mapRawToPosition(raw_max, &sc, 4094.0f, pos);
    for (int i = 3; i < 6; i++) {
        ASSERT_LT(pos[i], 1.57f);
        ASSERT_GT(pos[i], 0.0f);
    }
}

TEST(as_scales_positive) {
    /* All computed scales should be positive and reasonable */
    StewartConfig cfg = default_config();
    AxisScaleConfig sc;
    computeAxisScalesFromGeometry(&sc, &cfg, 0.90f);

    for (int i = 0; i < 6; i++) {
        ASSERT_GT(sc.scale[i], 0.0f);
    }
    /* Translation scales should be in a reasonable range (>10mm, <500mm) */
    for (int i = 0; i < 3; i++) {
        ASSERT_GT(sc.scale[i], 10.0f);
        ASSERT_LT(sc.scale[i], 500.0f);
    }
    /* Rotation scales should be in degrees (>1°, <90°) */
    for (int i = 3; i < 6; i++) {
        ASSERT_GT(sc.scale[i], 1.0f);
        ASSERT_LT(sc.scale[i], 90.0f);
    }
}

TEST(as_margin_effect) {
    /* Higher margin should produce larger (or equal) scales */
    StewartConfig cfg = default_config();
    AxisScaleConfig sc50, sc90;
    computeAxisScalesFromGeometry(&sc50, &cfg, 0.50f);
    computeAxisScalesFromGeometry(&sc90, &cfg, 0.90f);

    for (int i = 0; i < 6; i++) {
        /* 90% margin >= 50% margin */
        ASSERT_GT(sc90.scale[i], sc50.scale[i] - 1.0f);
    }
}

TEST(as_mapped_position_within_workspace) {
    /* Full-range mapped position should be within IK workspace */
    StewartConfig cfg = default_config();
    AxisScaleConfig sc;
    computeAxisScalesFromGeometry(&sc, &cfg, 0.90f);

    float raw_max[6] = {4094, 4094, 4094, 4094, 4094, 4094};
    float pos[6];
    mapRawToPosition(raw_max, &sc, 4094.0f, pos);

    /* Each individual axis at max should still be valid (margin protects us) */
    for (int axis = 0; axis < 6; axis++) {
        float test_pos[6] = {0, 0, 0, 0, 0, 0};
        test_pos[axis] = pos[axis];
        int result = validatePosition(test_pos, &cfg);
        ASSERT_EQ(result, 0);
    }
}

TEST(as_null_safety) {
    /* Should not crash with NULL pointers */
    computeAxisScalesFromGeometry(NULL, NULL, 0.9f);
    StewartConfig cfg = default_config();
    computeAxisScalesFromGeometry(NULL, &cfg, 0.9f);
    AxisScaleConfig sc;
    computeAxisScalesFromGeometry(&sc, NULL, 0.9f);
    /* All should return without crash */
}

/* ── Registration ─────────────────────────────────────────────────── */

void register_axis_scaling_tests(void) {
    RUN_TEST(as_center_maps_to_zero_12bit);
    RUN_TEST(as_center_maps_to_zero_8bit);
    RUN_TEST(as_center_maps_to_zero_16bit);
    RUN_TEST(as_extremes_map_to_scale);
    RUN_TEST(as_angle_conversion);
    RUN_TEST(as_scales_positive);
    RUN_TEST(as_margin_effect);
    RUN_TEST(as_mapped_position_within_workspace);
    RUN_TEST(as_null_safety);
}
