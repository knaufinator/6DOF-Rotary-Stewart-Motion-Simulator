#include "test_framework.h"
#include "InverseKinematics.h"

/* ── Helpers ──────────────────────────────────────────────────────── */

static StewartConfig default_config(void) {
    StewartConfig cfg;
    initDefaultStewartConfig(&cfg);
    return cfg;
}

static PlatformDef default_platform(void) {
    StewartConfig cfg = default_config();
    PlatformDef plat;
    buildPlatformFromConfig(&cfg, &plat);
    return plat;
}

static const float ZERO_POS[6] = {0, 0, 0, 0, 0, 0};

/* ── Tests ────────────────────────────────────────────────────────── */

TEST(ik_init_defaults) {
    StewartConfig cfg;
    initDefaultStewartConfig(&cfg);
    ASSERT_NEAR(cfg.RD, 400.05f, 0.01f);
    ASSERT_NEAR(cfg.PD, 406.4f, 0.01f);
    ASSERT_NEAR(cfg.ServoArmLengthL1, 184.15f, 0.01f);
    ASSERT_NEAR(cfg.ConnectingArmLengthL2, 723.9f, 0.01f);
    ASSERT_GT(cfg.platformHeight, 0.0f);
    ASSERT_GT(cfg.steps_per_degree, 0.0f);
}

TEST(ik_home_position_zero_angles) {
    /* At home (all inputs zero), all servo angles should be ~0 */
    StewartConfig cfg = default_config();
    float angles[6];
    calculateAllServoAngles(ZERO_POS, &cfg, angles);
    for (int i = 0; i < 6; i++) {
        ASSERT_NEAR(angles[i], 0.0f, 0.01f);
    }
}

TEST(ik_home_position_v2_zero_angles) {
    /* Generalized API should also give ~0 at home */
    PlatformDef plat = default_platform();
    float angles[6];
    calcAllActuatorAngles(ZERO_POS, &plat, angles);
    for (int i = 0; i < 6; i++) {
        ASSERT_NEAR(angles[i], 0.0f, 0.01f);
    }
}

TEST(ik_pure_heave_symmetry) {
    /* Pure heave: all 6 servos should produce identical angles */
    StewartConfig cfg = default_config();
    float pos[6] = {0, 0, 5.0f, 0, 0, 0};  /* 5mm heave */
    float angles[6];
    calculateAllServoAngles(pos, &cfg, angles);
    for (int i = 1; i < 6; i++) {
        ASSERT_NEAR(angles[i], angles[0], 0.001f);
    }
}

TEST(ik_pure_heave_symmetry_v2) {
    PlatformDef plat = default_platform();
    float pos[6] = {0, 0, 5.0f, 0, 0, 0};
    float angles[6];
    calcAllActuatorAngles(pos, &plat, angles);
    for (int i = 1; i < 6; i++) {
        ASSERT_NEAR(angles[i], angles[0], 0.001f);
    }
}

TEST(ik_heave_direction) {
    /* Positive heave should produce positive servo angles (platform goes up) */
    StewartConfig cfg = default_config();
    float pos[6] = {0, 0, 10.0f, 0, 0, 0};
    float angles[6];
    calculateAllServoAngles(pos, &cfg, angles);
    for (int i = 0; i < 6; i++) {
        ASSERT_GT(angles[i], 0.0f);
    }
}

TEST(ik_negative_heave) {
    /* Negative heave should produce negative servo angles */
    StewartConfig cfg = default_config();
    float pos[6] = {0, 0, -10.0f, 0, 0, 0};
    float angles[6];
    calculateAllServoAngles(pos, &cfg, angles);
    for (int i = 0; i < 6; i++) {
        ASSERT_LT(angles[i], 0.0f);
    }
}

TEST(ik_rod_length_preservation) {
    /*
     * For any valid pose, the distance from arm tip to platform joint
     * must equal L2. Verify using the generalized API.
     */
    StewartConfig cfg = default_config();
    PlatformDef plat;
    buildPlatformFromConfig(&cfg, &plat);

    float pos[6] = {5.0f, -3.0f, 8.0f, 0.02f, -0.01f, 0.03f};
    float angles[6];
    calcAllActuatorAngles(pos, &plat, angles);

    float roll = pos[3], pitch = pos[4], yaw = pos[5];
    float cr = cosf(roll),  sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw),   sy = sinf(yaw);

    for (int k = 0; k < 6; k++) {
        const ActuatorDef* a = &plat.actuators[k];
        float alpha = angles[k];

        /* Arm tip = base + L1 * [cos(beta)*cos(alpha), sin(beta)*cos(alpha), sin(alpha)] */
        float atx = a->base_pos[0] + a->L1 * cosf(a->beta) * cosf(alpha);
        float aty = a->base_pos[1] + a->L1 * sinf(a->beta) * cosf(alpha);
        float atz = a->base_pos[2] + a->L1 * sinf(alpha);

        /* Rotated platform joint: R * P_k + T + [0,0,home_h] */
        float px = a->plat_pos[0], py = a->plat_pos[1], pz = a->plat_pos[2];
        float rpx = (cy*cp)*px + (cy*sp*sr - sy*cr)*py + (cy*sp*cr + sy*sr)*pz + pos[0];
        float rpy = (sy*cp)*px + (sy*sp*sr + cy*cr)*py + (sy*sp*cr - cy*sr)*pz + pos[1];
        float rpz = (-sp)*px   + (cp*sr)*py             + (cp*cr)*pz + plat.home_height + pos[2];

        /* Rod length = distance from arm tip to rotated platform joint */
        float dx = rpx - atx, dy = rpy - aty, dz = rpz - atz;
        float rod_len = sqrtf(dx*dx + dy*dy + dz*dz);

        ASSERT_NEAR(rod_len, a->L2, 0.5f);
    }
}

TEST(ik_validate_home_position_valid) {
    /* Home position should be within workspace */
    StewartConfig cfg = default_config();
    ASSERT_EQ(validatePosition(ZERO_POS, &cfg), 0);
}

TEST(ik_validate_extreme_out_of_range) {
    /* Extreme position should be out of range */
    StewartConfig cfg = default_config();
    float pos[6] = {500.0f, 0, 0, 0, 0, 0};  /* 500mm surge */
    int result = validatePosition(pos, &cfg);
    ASSERT(result != 0);
}

TEST(ik_validate_v2_home_valid) {
    PlatformDef plat = default_platform();
    ASSERT_EQ(validatePositionV2(ZERO_POS, &plat), 0);
}

TEST(ik_validate_v2_extreme_out_of_range) {
    PlatformDef plat = default_platform();
    float pos[6] = {500.0f, 0, 0, 0, 0, 0};
    int result = validatePositionV2(pos, &plat);
    ASSERT(result != 0);
}

TEST(ik_servo_angle_clamping) {
    /* Very extreme input should be clamped to ±60° (±1.047 rad) */
    StewartConfig cfg = default_config();
    float pos[6] = {0, 0, 300.0f, 0, 0, 0};  /* way beyond workspace */
    float angles[6];
    calculateAllServoAngles(pos, &cfg, angles);
    for (int i = 0; i < 6; i++) {
        ASSERT_LT(angles[i], IK_SERVO_MAX_RAD + 0.001f);
        ASSERT_GT(angles[i], IK_SERVO_MIN_RAD - 0.001f);
    }
}

TEST(ik_old_vs_new_api_parity) {
    /*
     * Both APIs should produce the same angles at HOME (all zeros).
     * For non-zero rotations they diverge because the old API uses a
     * simplified rotation matrix while the new API uses proper ZYX Euler.
     * Pure translation (no rotation) should still agree closely.
     */
    StewartConfig cfg = default_config();
    PlatformDef plat;
    buildPlatformFromConfig(&cfg, &plat);

    /* Home position: must agree exactly */
    float home[6] = {0, 0, 0, 0, 0, 0};
    float a_old[6], a_new[6];
    calculateAllServoAngles(home, &cfg, a_old);
    calcAllActuatorAngles(home, &plat, a_new);
    for (int i = 0; i < 6; i++) {
        ASSERT_NEAR(a_old[i], a_new[i], 0.001f);
    }

    /* Pure translation (no rotation): should agree closely */
    float trans[6] = {3.0f, -2.0f, 5.0f, 0, 0, 0};
    calculateAllServoAngles(trans, &cfg, a_old);
    calcAllActuatorAngles(trans, &plat, a_new);
    for (int i = 0; i < 6; i++) {
        ASSERT_NEAR(a_old[i], a_new[i], 0.01f);
    }
}

TEST(ik_compute_home_height) {
    /* computeHomeHeight should match the config default */
    StewartConfig cfg = default_config();
    PlatformDef plat;
    buildPlatformFromConfig(&cfg, &plat);
    float h = computeHomeHeight(&plat);
    ASSERT_NEAR(h, cfg.platformHeight, 1.0f);
}

TEST(ik_build_platform_round_trip) {
    /* buildPlatformFromConfig should preserve arm lengths and limits */
    StewartConfig cfg = default_config();
    PlatformDef plat;
    buildPlatformFromConfig(&cfg, &plat);
    for (int k = 0; k < 6; k++) {
        ASSERT_NEAR(plat.actuators[k].L1, cfg.ServoArmLengthL1, 0.001f);
        ASSERT_NEAR(plat.actuators[k].L2, cfg.ConnectingArmLengthL2, 0.001f);
    }
    ASSERT_NEAR(plat.servo_min_rad, IK_SERVO_MIN_RAD, 0.0001f);
    ASSERT_NEAR(plat.servo_max_rad, IK_SERVO_MAX_RAD, 0.0001f);
    ASSERT_NEAR(plat.home_height, cfg.platformHeight, 0.001f);
}

TEST(ik_steps_per_degree) {
    /* Known calculation: (2500 * 4 / 80) * 50 / 360 = 17.361... */
    StewartConfig cfg;
    cfg.encoder_ppr = 2500;
    cfg.virtual_gear = 80.0f;
    cfg.planetary_ratio = 50.0f;
    cfg.steps_per_degree = 0.0f;
    computeStepsPerDegree(&cfg);
    float expected = (2500.0f * 4.0f / 80.0f) * 50.0f / 360.0f;
    ASSERT_NEAR(cfg.steps_per_degree, expected, 0.01f);
}

TEST(ik_steps_per_degree_gear1) {
    /* With gear=1: (2500 * 4 / 1) * 50 / 360 = 1388.889 */
    StewartConfig cfg;
    cfg.encoder_ppr = 2500;
    cfg.virtual_gear = 1.0f;
    cfg.planetary_ratio = 50.0f;
    cfg.steps_per_degree = 0.0f;
    computeStepsPerDegree(&cfg);
    float expected = (2500.0f * 4.0f) * 50.0f / 360.0f;
    ASSERT_NEAR(cfg.steps_per_degree, expected, 0.01f);
}

TEST(ik_null_safety) {
    /* Functions should handle NULL gracefully */
    float angles[6] = {0};
    calculateAllServoAngles(ZERO_POS, NULL, angles);
    /* Should not crash, angles should be unchanged */
    for (int i = 0; i < 6; i++) {
        ASSERT_NEAR(angles[i], 0.0f, 0.001f);
    }

    float a = calculateServoAngle(0, ZERO_POS, NULL);
    ASSERT_NEAR(a, 0.0f, 0.001f);

    int v = validatePosition(ZERO_POS, NULL);
    ASSERT_EQ(v, 0x3F);

    initDefaultStewartConfig(NULL);  /* should not crash */
}

TEST(ik_geometry_sensitivity) {
    /* Increasing L1 with fixed L2 should change servo angles for a given pose */
    StewartConfig cfg1 = default_config();
    StewartConfig cfg2 = default_config();
    cfg2.ServoArmLengthL1 *= 1.1f;  /* 10% longer arm */

    float pos[6] = {0, 0, 10.0f, 0, 0, 0};
    float a1[6], a2[6];
    calculateAllServoAngles(pos, &cfg1, a1);
    calculateAllServoAngles(pos, &cfg2, a2);

    /* Angles should differ — longer arm means smaller angle for same displacement */
    int differs = 0;
    for (int i = 0; i < 6; i++) {
        if (fabsf(a1[i] - a2[i]) > 0.001f) differs++;
    }
    ASSERT_GT(differs, 0);
}

/* ── Registration ─────────────────────────────────────────────────── */

void register_ik_tests(void) {
    RUN_TEST(ik_init_defaults);
    RUN_TEST(ik_home_position_zero_angles);
    RUN_TEST(ik_home_position_v2_zero_angles);
    RUN_TEST(ik_pure_heave_symmetry);
    RUN_TEST(ik_pure_heave_symmetry_v2);
    RUN_TEST(ik_heave_direction);
    RUN_TEST(ik_negative_heave);
    RUN_TEST(ik_rod_length_preservation);
    RUN_TEST(ik_validate_home_position_valid);
    RUN_TEST(ik_validate_extreme_out_of_range);
    RUN_TEST(ik_validate_v2_home_valid);
    RUN_TEST(ik_validate_v2_extreme_out_of_range);
    RUN_TEST(ik_servo_angle_clamping);
    RUN_TEST(ik_old_vs_new_api_parity);
    RUN_TEST(ik_compute_home_height);
    RUN_TEST(ik_build_platform_round_trip);
    RUN_TEST(ik_steps_per_degree);
    RUN_TEST(ik_steps_per_degree_gear1);
    RUN_TEST(ik_null_safety);
    RUN_TEST(ik_geometry_sensitivity);
}
