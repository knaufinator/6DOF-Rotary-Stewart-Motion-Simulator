#include "test_framework.h"
#include "MotionCueing.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Helpers ──────────────────────────────────────────────────────── */

static const float ZERO_IN[6] = {0, 0, 0, 0, 0, 0};

static MotionCueingConfig make_mca(float sr, int preset) {
    MotionCueingConfig cfg;
    initMotionCueing(&cfg, sr);
    if (preset != MCA_OFF)
        setMotionCueingPreset(&cfg, preset);
    return cfg;
}

/* ── Tests ────────────────────────────────────────────────────────── */

TEST(mca_init_defaults) {
    MotionCueingConfig cfg;
    initMotionCueing(&cfg, 100.0f);
    ASSERT_EQ(cfg.schema_version, (uint32_t)MCA_SCHEMA_VERSION);
    ASSERT_NEAR(cfg.sample_rate, 100.0f, 0.01f);
    ASSERT_EQ(cfg.enabled, 0);
    ASSERT_EQ(cfg.preset, MCA_OFF);
    for (int i = 0; i < 6; i++) {
        ASSERT_NEAR(cfg.channels[i].gain, 1.0f, 0.001f);
    }
}

TEST(mca_passthrough_when_disabled) {
    /* MCA_OFF: output should equal input */
    MotionCueingConfig cfg = make_mca(100.0f, MCA_OFF);
    float in[6] = {1.0f, -2.0f, 3.5f, 0.1f, -0.2f, 0.05f};
    float out[6];
    processMotionCueing(&cfg, in, out);
    for (int i = 0; i < 6; i++) {
        ASSERT_NEAR(out[i], in[i], 0.0001f);
    }
}

TEST(mca_dc_rejection) {
    /*
     * HP washout: constant (DC) input should decay to ~0 after many samples.
     * Use MCA_MODERATE (0.8Hz HP at 100Hz sample rate).
     */
    MotionCueingConfig cfg = make_mca(100.0f, MCA_MODERATE);
    float in[6] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    float out[6];

    /* Run 500 samples (5 seconds at 100Hz) — DC should wash out */
    for (int s = 0; s < 500; s++) {
        processMotionCueing(&cfg, in, out);
    }
    /* Surge (axis 0) should be close to 0 after washout */
    ASSERT_LT(fabsf(out[0]), 0.1f);
    ASSERT_LT(fabsf(out[1]), 0.1f);
    ASSERT_LT(fabsf(out[2]), 0.1f);
}

TEST(mca_ac_passthrough) {
    /*
     * A 5Hz sine wave through a 0.8Hz HP filter should pass with ~unity gain.
     * Use MCA_MODERATE at 100Hz sample rate.
     */
    MotionCueingConfig cfg = make_mca(100.0f, MCA_MODERATE);
    resetMotionCueing(&cfg);

    float freq = 5.0f;
    float sr = 100.0f;
    float peak_out = 0.0f;

    /* Let the filter settle first (100 samples) */
    for (int s = 0; s < 100; s++) {
        float t = s / sr;
        float val = sinf(2.0f * (float)M_PI * freq * t);
        float in[6] = {val, 0, 0, 0, 0, 0};
        float out[6];
        processMotionCueing(&cfg, in, out);
    }

    /* Now measure peak over 200 samples */
    for (int s = 100; s < 300; s++) {
        float t = s / sr;
        float val = sinf(2.0f * (float)M_PI * freq * t);
        float in[6] = {val, 0, 0, 0, 0, 0};
        float out[6];
        processMotionCueing(&cfg, in, out);
        if (fabsf(out[0]) > peak_out) peak_out = fabsf(out[0]);
    }

    /* Output amplitude should be close to gain * 1.0 (gain=1.0 for MODERATE surge) */
    ASSERT_GT(peak_out, 0.7f);
    ASSERT_LT(peak_out, 1.5f);
}

TEST(mca_gain_scaling) {
    /* Setting gain=2.0 on an axis should roughly double output amplitude */
    MotionCueingConfig cfg = make_mca(100.0f, MCA_OFF);
    cfg.enabled = 1;

    /* Setup: axis 0 with no filters, just gain */
    for (int i = 0; i < 6; i++) {
        cfg.channels[i].hp_enabled = 0;
        cfg.channels[i].lp_enabled = 0;
    }
    cfg.channels[0].gain = 2.0f;
    cfg.tilt.enabled = 0;

    float in[6] = {0.5f, 0, 0, 0, 0, 0};
    float out[6];
    processMotionCueing(&cfg, in, out);
    ASSERT_NEAR(out[0], 1.0f, 0.001f);
}

TEST(mca_rate_limiting) {
    /* Rate limiter should clamp the change per sample */
    MotionCueingConfig cfg = make_mca(100.0f, MCA_OFF);
    cfg.enabled = 1;

    for (int i = 0; i < 6; i++) {
        cfg.channels[i].hp_enabled = 0;
        cfg.channels[i].lp_enabled = 0;
    }
    cfg.channels[0].rate_limit = 0.1f;  /* max 0.1 per sample */
    cfg.channels[0].last_output = 0.0f;
    cfg.tilt.enabled = 0;

    /* Large step input */
    float in[6] = {10.0f, 0, 0, 0, 0, 0};
    float out[6];
    processMotionCueing(&cfg, in, out);
    /* First sample: should be clamped to 0.0 + 0.1 = 0.1 */
    ASSERT_NEAR(out[0], 0.1f, 0.001f);

    /* Second sample: should advance to 0.2 */
    processMotionCueing(&cfg, in, out);
    ASSERT_NEAR(out[0], 0.2f, 0.001f);
}

TEST(mca_tilt_coordination) {
    /*
     * Sustained surge input with tilt coordination enabled should
     * produce a pitch offset (axis 4).
     */
    MotionCueingConfig cfg = make_mca(100.0f, MCA_GENTLE);
    resetMotionCueing(&cfg);

    /* Feed sustained surge for 200 samples */
    float out[6];
    for (int s = 0; s < 200; s++) {
        float in[6] = {1.0f, 0, 0, 0, 0, 0};
        processMotionCueing(&cfg, in, out);
    }

    /* Pitch (index 4) should have some tilt offset */
    ASSERT_GT(fabsf(out[4]), 0.001f);
}

TEST(mca_preset_names) {
    ASSERT(strcmp(mcaPresetName(MCA_OFF), "off") == 0);
    ASSERT(strcmp(mcaPresetName(MCA_GENTLE), "gentle") == 0);
    ASSERT(strcmp(mcaPresetName(MCA_MODERATE), "moderate") == 0);
    ASSERT(strcmp(mcaPresetName(MCA_AGGRESSIVE), "aggressive") == 0);
    ASSERT(strcmp(mcaPresetName(MCA_RACE_PRO), "race_pro") == 0);
    ASSERT(strcmp(mcaPresetName(-1), "unknown") == 0);
    ASSERT(strcmp(mcaPresetName(99), "unknown") == 0);
}

TEST(mca_reset_clears_state) {
    MotionCueingConfig cfg = make_mca(100.0f, MCA_MODERATE);

    /* Feed some data to build up filter state */
    float in[6] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    float out[6];
    for (int s = 0; s < 50; s++)
        processMotionCueing(&cfg, in, out);

    resetMotionCueing(&cfg);

    /* After reset, all biquad states should be zero */
    for (int i = 0; i < 6; i++) {
        ASSERT_NEAR(cfg.channels[i].hp.z1, 0.0f, 0.0001f);
        ASSERT_NEAR(cfg.channels[i].hp.z2, 0.0f, 0.0001f);
        ASSERT_NEAR(cfg.channels[i].lp.z1, 0.0f, 0.0001f);
        ASSERT_NEAR(cfg.channels[i].lp.z2, 0.0f, 0.0001f);
        ASSERT_NEAR(cfg.channels[i].last_output, 0.0f, 0.0001f);
    }
    ASSERT_NEAR(cfg.tilt.surge_lp.z1, 0.0f, 0.0001f);
    ASSERT_NEAR(cfg.tilt.sway_lp.z1, 0.0f, 0.0001f);
}

TEST(mca_sample_rate_update) {
    /* Changing sample rate should recalculate coefficients without NaN */
    MotionCueingConfig cfg = make_mca(100.0f, MCA_MODERATE);
    mcaUpdateSampleRate(&cfg, 200.0f);
    ASSERT_NEAR(cfg.sample_rate, 200.0f, 0.01f);

    /* Process a sample — should not produce NaN */
    float in[6] = {1.0f, 0, 0, 0, 0, 0};
    float out[6];
    processMotionCueing(&cfg, in, out);
    for (int i = 0; i < 6; i++) {
        ASSERT(out[i] == out[i]);  /* NaN != NaN */
    }
}

TEST(mca_long_run_stability) {
    /* 100k samples of noise should not produce NaN or Inf */
    MotionCueingConfig cfg = make_mca(100.0f, MCA_AGGRESSIVE);

    /* Simple pseudo-random via LCG */
    unsigned int seed = 12345;
    for (int s = 0; s < 100000; s++) {
        float in[6];
        for (int i = 0; i < 6; i++) {
            seed = seed * 1103515245 + 12345;
            in[i] = ((float)(seed & 0xFFFF) / 32768.0f) - 1.0f;
        }
        float out[6];
        processMotionCueing(&cfg, in, out);
        for (int i = 0; i < 6; i++) {
            ASSERT(out[i] == out[i]);       /* not NaN */
            ASSERT(out[i] < 1e6f && out[i] > -1e6f);  /* bounded */
        }
    }
}

TEST(mca_validate_config) {
    MotionCueingConfig cfg = make_mca(100.0f, MCA_MODERATE);
    ASSERT_EQ(mcaValidateConfig(&cfg), 1);

    /* Wrong schema version → invalid */
    MotionCueingConfig bad = cfg;
    bad.schema_version = 0;
    ASSERT_EQ(mcaValidateConfig(&bad), 0);

    /* Bad sample rate → invalid */
    bad = cfg;
    bad.sample_rate = -1.0f;
    ASSERT_EQ(mcaValidateConfig(&bad), 0);

    /* Bad preset → invalid */
    bad = cfg;
    bad.preset = 99;
    ASSERT_EQ(mcaValidateConfig(&bad), 0);

    /* NULL → invalid */
    ASSERT_EQ(mcaValidateConfig(NULL), 0);
}

TEST(mca_preset_sets_enabled) {
    MotionCueingConfig cfg;
    initMotionCueing(&cfg, 100.0f);
    ASSERT_EQ(cfg.enabled, 0);

    setMotionCueingPreset(&cfg, MCA_MODERATE);
    ASSERT_EQ(cfg.enabled, 1);

    setMotionCueingPreset(&cfg, MCA_OFF);
    ASSERT_EQ(cfg.enabled, 0);
}

TEST(mca_all_presets_valid) {
    /* Every preset should produce a valid config */
    for (int p = 0; p < MCA_PRESET_COUNT; p++) {
        MotionCueingConfig cfg = make_mca(100.0f, p);
        ASSERT_EQ(mcaValidateConfig(&cfg), 1);
    }
}

TEST(mca_biquad_hp_dc_block) {
    /* Direct biquad HP test: DC input → 0 output after settling */
    BiquadFilter f;
    memset(&f, 0, sizeof(f));
    biquadSetHighpass(&f, 1.0f, 100.0f, 0.707f);

    float out = 0;
    for (int i = 0; i < 500; i++) {
        out = biquadProcess(&f, 1.0f);
    }
    ASSERT_LT(fabsf(out), 0.05f);
}

TEST(mca_biquad_lp_hf_reject) {
    /* Direct biquad LP test: 40Hz sine at 100Hz sample rate → heavily attenuated */
    BiquadFilter f;
    memset(&f, 0, sizeof(f));
    biquadSetLowpass(&f, 5.0f, 100.0f, 0.707f);

    float peak = 0;
    /* Settle */
    for (int i = 0; i < 100; i++) {
        float x = sinf(2.0f * (float)M_PI * 40.0f * i / 100.0f);
        biquadProcess(&f, x);
    }
    /* Measure */
    for (int i = 100; i < 300; i++) {
        float x = sinf(2.0f * (float)M_PI * 40.0f * i / 100.0f);
        float y = biquadProcess(&f, x);
        if (fabsf(y) > peak) peak = fabsf(y);
    }
    /* 40Hz through 5Hz LP should be heavily attenuated (<0.1) */
    ASSERT_LT(peak, 0.15f);
}

TEST(mca_biquad_notch_reject) {
    /* Notch at 10Hz should create a deep null at 10Hz */
    BiquadFilter f;
    memset(&f, 0, sizeof(f));
    biquadSetNotch(&f, 10.0f, 100.0f, 5.0f);

    float peak_at_notch = 0;
    /* Settle */
    for (int i = 0; i < 200; i++) {
        float x = sinf(2.0f * (float)M_PI * 10.0f * i / 100.0f);
        biquadProcess(&f, x);
    }
    /* Measure at notch freq */
    for (int i = 200; i < 400; i++) {
        float x = sinf(2.0f * (float)M_PI * 10.0f * i / 100.0f);
        float y = biquadProcess(&f, x);
        if (fabsf(y) > peak_at_notch) peak_at_notch = fabsf(y);
    }
    ASSERT_LT(peak_at_notch, 0.1f);

    /* Off-freq (2Hz) should pass through */
    biquadReset(&f);
    float peak_off = 0;
    for (int i = 0; i < 200; i++) {
        float x = sinf(2.0f * (float)M_PI * 2.0f * i / 100.0f);
        biquadProcess(&f, x);
    }
    for (int i = 200; i < 400; i++) {
        float x = sinf(2.0f * (float)M_PI * 2.0f * i / 100.0f);
        float y = biquadProcess(&f, x);
        if (fabsf(y) > peak_off) peak_off = fabsf(y);
    }
    ASSERT_GT(peak_off, 0.5f);
}

TEST(mca_input_filter_passthrough) {
    /* Input filter disabled → passthrough */
    InputFilterConfig ifcfg;
    initInputFilter(&ifcfg, 100.0f);
    ASSERT_EQ(ifcfg.enabled, 0);

    float in[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    float out[6];
    processInputFilter(&ifcfg, in, out);
    for (int i = 0; i < 6; i++) {
        ASSERT_NEAR(out[i], in[i], 0.0001f);
    }
}

/* ── Registration ─────────────────────────────────────────────────── */

void register_motion_cueing_tests(void) {
    RUN_TEST(mca_init_defaults);
    RUN_TEST(mca_passthrough_when_disabled);
    RUN_TEST(mca_dc_rejection);
    RUN_TEST(mca_ac_passthrough);
    RUN_TEST(mca_gain_scaling);
    RUN_TEST(mca_rate_limiting);
    RUN_TEST(mca_tilt_coordination);
    RUN_TEST(mca_preset_names);
    RUN_TEST(mca_reset_clears_state);
    RUN_TEST(mca_sample_rate_update);
    RUN_TEST(mca_long_run_stability);
    RUN_TEST(mca_validate_config);
    RUN_TEST(mca_preset_sets_enabled);
    RUN_TEST(mca_all_presets_valid);
    RUN_TEST(mca_biquad_hp_dc_block);
    RUN_TEST(mca_biquad_lp_hf_reject);
    RUN_TEST(mca_biquad_notch_reject);
    RUN_TEST(mca_input_filter_passthrough);
}
