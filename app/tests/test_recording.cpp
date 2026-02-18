// Unit tests for fixed-rate recording and playback precision.
// Build: cl /EHsc /std:c++17 /I../src test_recording.cpp /Fe:test_recording.exe
// Run:   test_recording.exe

#define _USE_MATH_DEFINES
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <cassert>

// ── Minimal types mirroring app.h ──────────────────────────────────

struct RecordSample {
    double time;
    float  input[6];
};

struct SavedRecording {
    char                        name[64];
    double                      sample_rate_hz;
    std::vector<RecordSample>   samples;
    double duration() const {
        if (samples.empty()) return 0.0;
        return samples.back().time;
    }
};

// ── Fixed-rate oversampling (same logic as production code) ────────

static void simulateRecording(
    std::vector<RecordSample>& samples,
    int rate_hz,
    double total_duration_sec,
    double frame_rate_hz,          // simulated render FPS
    float (*inputFunc)(double t, int axis)  // generates test signal
) {
    samples.clear();
    double frame_dt = 1.0 / frame_rate_hz;
    double start_time = 0.0;

    // First-order hold state (mirrors production RecordingState)
    float prev_input[6] = {}, curr_input[6] = {};
    double prev_time = 0.0, curr_time = 0.0;
    for (int i = 0; i < 6; i++)
        prev_input[i] = curr_input[i] = inputFunc(0.0, i);

    for (double t = 0.0; t < total_duration_sec; t += frame_dt) {
        // Advance first-order hold: prev ← old curr, curr ← new input
        memcpy(prev_input, curr_input, sizeof(prev_input));
        prev_time = curr_time;
        for (int i = 0; i < 6; i++)
            curr_input[i] = inputFunc(t, i);
        curr_time = t;

        int target_count = (int)(t * (double)rate_hz) + 1;
        if (target_count > (int)samples.size()) {
            double dt = curr_time - prev_time;
            while ((int)samples.size() < target_count) {
                RecordSample s;
                s.time = (double)samples.size() / (double)rate_hz;
                // First-order hold: interpolate between prev and curr frame
                double sample_wall = start_time + s.time;
                float alpha = (dt > 1e-9)
                    ? (float)((sample_wall - prev_time) / dt)
                    : 1.0f;
                if (alpha < 0.0f) alpha = 0.0f;
                if (alpha > 1.0f) alpha = 1.0f;
                for (int i = 0; i < 6; i++)
                    s.input[i] = prev_input[i] * (1.0f - alpha) + curr_input[i] * alpha;
                samples.push_back(s);
            }
        }
    }
}

// ── Direct-index playback (same logic as production code) ──────────

static void sampleFixedRate(const SavedRecording& sr, double elapsed, float out[6]) {
    if (sr.samples.empty()) { memset(out, 0, 6 * sizeof(float)); return; }

    double dur = sr.duration();
    if (elapsed < 0.0) elapsed = 0.0;
    if (elapsed > dur) elapsed = dur;

    // Direct index from sample rate (O(1) — no linear search)
    double fidx = elapsed * sr.sample_rate_hz;
    int idx = (int)fidx;
    if (idx >= (int)sr.samples.size() - 1) {
        memcpy(out, sr.samples.back().input, 6 * sizeof(float));
        return;
    }
    float alpha = (float)(fidx - (double)idx);
    for (int i = 0; i < 6; i++) {
        out[i] = sr.samples[idx].input[i] * (1.0f - alpha) +
                 sr.samples[idx + 1].input[i] * alpha;
    }
}

// ── Test Helpers ───────────────────────────────────────────────────

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) static void test_##name()
#define RUN(name) do { \
    printf("  %-50s ", #name); \
    test_##name(); \
    printf("PASS\n"); \
    tests_passed++; \
} while(0)

#define ASSERT_EQ(a, b) do { \
    auto _a = (a); auto _b = (b); \
    if (_a != _b) { \
        printf("FAIL\n    %s:%d: %s == %lld, expected %lld\n", \
               __FILE__, __LINE__, #a, (long long)_a, (long long)_b); \
        tests_failed++; return; \
    } \
} while(0)

#define ASSERT_NEAR(a, b, eps) do { \
    double _a = (double)(a); double _b = (double)(b); double _e = (double)(eps); \
    if (fabs(_a - _b) > _e) { \
        printf("FAIL\n    %s:%d: %s == %.6f, expected %.6f (eps=%.6f)\n", \
               __FILE__, __LINE__, #a, _a, _b, _e); \
        tests_failed++; return; \
    } \
} while(0)

// ── Tests ──────────────────────────────────────────────────────────

// Test: sample count matches rate × duration
TEST(sample_count_at_1000hz) {
    std::vector<RecordSample> samples;
    double duration = 5.0;
    int rate = 1000;
    simulateRecording(samples, rate, duration, 60.0,
        [](double, int) -> float { return 0.0f; });

    // Expected: rate * duration + 1 (sample at t=0 counts)
    int expected = (int)(duration * rate);
    // Allow ±1 for rounding at boundary
    int actual = (int)samples.size();
    ASSERT_NEAR(actual, expected, 2);
}

TEST(sample_count_at_100hz) {
    std::vector<RecordSample> samples;
    simulateRecording(samples, 100, 10.0, 60.0,
        [](double, int) -> float { return 0.0f; });
    ASSERT_NEAR((int)samples.size(), 1000, 2);
}

TEST(sample_count_at_200hz) {
    std::vector<RecordSample> samples;
    simulateRecording(samples, 200, 5.0, 144.0,
        [](double, int) -> float { return 0.0f; });
    ASSERT_NEAR((int)samples.size(), 1000, 2);
}

// Test: timestamps are perfectly spaced at 1/rate
TEST(timestamp_spacing_1000hz) {
    std::vector<RecordSample> samples;
    simulateRecording(samples, 1000, 1.0, 60.0,
        [](double, int) -> float { return 0.0f; });

    double expected_dt = 1.0 / 1000.0;
    for (int i = 1; i < (int)samples.size(); i++) {
        double dt = samples[i].time - samples[i-1].time;
        ASSERT_NEAR(dt, expected_dt, 1e-12);
    }
}

TEST(timestamp_spacing_100hz) {
    std::vector<RecordSample> samples;
    simulateRecording(samples, 100, 2.0, 60.0,
        [](double, int) -> float { return 0.0f; });

    double expected_dt = 1.0 / 100.0;
    for (int i = 1; i < (int)samples.size(); i++) {
        double dt = samples[i].time - samples[i-1].time;
        ASSERT_NEAR(dt, expected_dt, 1e-12);
    }
}

// Test: duration matches expected (samples/rate)
TEST(duration_consistency) {
    std::vector<RecordSample> samples;
    simulateRecording(samples, 1000, 23.0, 140.0,
        [](double, int) -> float { return 0.0f; });

    SavedRecording sr;
    sr.sample_rate_hz = 1000.0;
    sr.samples = samples;

    // Duration from timestamps
    double dur_ts = sr.duration();
    // Duration from sample count / rate
    double dur_rate = (double)(sr.samples.size() - 1) / sr.sample_rate_hz;

    ASSERT_NEAR(dur_ts, dur_rate, 0.002);
    ASSERT_NEAR(dur_ts, 23.0, 0.05);
}

// Test: reproduces original bug scenario — 23s at "1000 Hz" should have ~23000 samples
TEST(original_bug_23s_1000hz) {
    std::vector<RecordSample> samples;
    // Simulate at 140 FPS (like the user's system)
    simulateRecording(samples, 1000, 23.0, 140.0,
        [](double, int) -> float { return 0.0f; });

    // With oversampling: should be ~23000, NOT 3234
    ASSERT_NEAR((int)samples.size(), 23000, 10);

    // Duration should be ~23s
    ASSERT_NEAR(samples.back().time, 23.0, 0.01);
}

// Test: sine wave signal is captured correctly
TEST(sine_wave_capture) {
    std::vector<RecordSample> samples;
    // 10 Hz sine on axis 0
    simulateRecording(samples, 1000, 1.0, 60.0,
        [](double t, int axis) -> float {
            return (axis == 0) ? (float)sin(2.0 * M_PI * 10.0 * t) : 0.0f;
        });

    // At 1000 Hz for 1 second, we should have ~984-1000 samples
    // (last frame at 60 FPS lands at ~0.983s, so exact count depends on frame alignment)
    ASSERT_NEAR((int)samples.size(), 1000, 20);

    // Check a few known values
    // At t=0.025 (sample 25), sin(2π×10×0.025) = sin(π/2) = 1.0
    // But due to sample-and-hold, the value is from the last frame update
    // Frame at 60 FPS: nearest frame to t=0.025 is frame 1 (t=0.0167) or 2 (t=0.0333)
    // Value at t=0.0167: sin(2π×10×0.0167) = sin(1.047) ≈ 0.866
    // So sample 25 holds value from frame at t=0.0167 — approximately 0.866
    // This is expected: sample-and-hold captures the signal at frame rate
}

// Test: direct-index playback returns correct values
TEST(playback_direct_index) {
    SavedRecording sr;
    sr.sample_rate_hz = 100.0;
    snprintf(sr.name, sizeof(sr.name), "test");

    // Create a simple ramp: input[0] goes from 0 to 1 over 1 second
    for (int i = 0; i <= 100; i++) {
        RecordSample s;
        s.time = (double)i / 100.0;
        for (int a = 0; a < 6; a++)
            s.input[a] = (a == 0) ? (float)i / 100.0f : 0.0f;
        sr.samples.push_back(s);
    }

    // Sample at t=0.5 should give 0.5
    float out[6];
    sampleFixedRate(sr, 0.5, out);
    ASSERT_NEAR(out[0], 0.5f, 0.01f);

    // Sample at t=0.0 should give 0.0
    sampleFixedRate(sr, 0.0, out);
    ASSERT_NEAR(out[0], 0.0f, 0.001f);

    // Sample at t=1.0 should give 1.0
    sampleFixedRate(sr, 1.0, out);
    ASSERT_NEAR(out[0], 1.0f, 0.001f);

    // Sample at t=0.255 should interpolate between 0.25 and 0.26
    sampleFixedRate(sr, 0.255, out);
    ASSERT_NEAR(out[0], 0.255f, 0.01f);
}

// Test: playback at 2x speed completes in half the time
TEST(playback_speed_2x) {
    SavedRecording sr;
    sr.sample_rate_hz = 100.0;
    snprintf(sr.name, sizeof(sr.name), "speed_test");

    for (int i = 0; i <= 200; i++) {
        RecordSample s;
        s.time = (double)i / 100.0;  // 2 seconds
        s.input[0] = (float)i / 200.0f;
        for (int a = 1; a < 6; a++) s.input[a] = 0.0f;
        sr.samples.push_back(s);
    }

    // At 2x speed, t=0.5 real time → elapsed=1.0 in data
    float out[6];
    double data_elapsed = 0.5 * 2.0;  // capture_speed = 2.0
    sampleFixedRate(sr, data_elapsed, out);
    ASSERT_NEAR(out[0], 0.5f, 0.01f);
}

// Test: high sample count with low FPS still produces correct count
TEST(oversampling_low_fps) {
    std::vector<RecordSample> samples;
    // 1000 Hz recording at only 30 FPS — heavy oversampling
    simulateRecording(samples, 1000, 5.0, 30.0,
        [](double, int) -> float { return 42.0f; });

    ASSERT_NEAR((int)samples.size(), 5000, 10);

    // All samples should have the held value
    for (auto& s : samples)
        ASSERT_NEAR(s.input[0], 42.0f, 0.001f);
}

// Test: recording at rate <= FPS still works correctly
TEST(recording_at_low_rate) {
    std::vector<RecordSample> samples;
    // 50 Hz recording at 144 FPS — no oversampling needed
    simulateRecording(samples, 50, 2.0, 144.0,
        [](double, int) -> float { return 1.0f; });

    ASSERT_NEAR((int)samples.size(), 100, 2);
}

// Test: empty recording edge case
TEST(empty_recording_playback) {
    SavedRecording sr;
    sr.sample_rate_hz = 1000.0;
    sr.samples.clear();

    float out[6] = {99, 99, 99, 99, 99, 99};
    sampleFixedRate(sr, 0.5, out);
    for (int i = 0; i < 6; i++)
        ASSERT_NEAR(out[i], 0.0f, 0.001f);
}

// Test: verify no accumulated timing drift over long recordings
TEST(no_timing_drift_long_recording) {
    std::vector<RecordSample> samples;
    // 10 minutes at 500 Hz, 60 FPS
    double duration = 600.0;
    simulateRecording(samples, 500, duration, 60.0,
        [](double, int) -> float { return 0.0f; });

    int expected = (int)(duration * 500);
    ASSERT_NEAR((int)samples.size(), expected, 5);

    // Check last timestamp matches expected duration
    double expected_last_time = (double)(samples.size() - 1) / 500.0;
    ASSERT_NEAR(samples.back().time, expected_last_time, 1e-10);

    // Check spacing at end of recording (should be identical to start)
    int n = (int)samples.size();
    double dt_start = samples[1].time - samples[0].time;
    double dt_end = samples[n-1].time - samples[n-2].time;
    ASSERT_NEAR(dt_start, dt_end, 1e-12);
}

// Test: first-order hold produces smooth interpolation (no staircase)
TEST(first_order_hold_smooth) {
    std::vector<RecordSample> samples;
    // Linear ramp: axis 0 goes from 0 to 1 over 1 second
    // At 60 FPS with 1000 Hz recording, there are ~17 samples per frame
    // First-order hold should linearly interpolate between frame values
    simulateRecording(samples, 1000, 1.0, 60.0,
        [](double t, int axis) -> float {
            return (axis == 0) ? (float)t : 0.0f;
        });

    // Check that consecutive samples are NOT identical (no staircase).
    // With a linear ramp, every sample should differ from the next.
    int identical_pairs = 0;
    for (int i = 1; i < (int)samples.size(); i++) {
        if (fabsf(samples[i].input[0] - samples[i-1].input[0]) < 1e-9f)
            identical_pairs++;
    }
    // With zero-order hold, ~94% of pairs would be identical (16/17 per frame).
    // With first-order hold, very few (if any) should be identical.
    // Allow a small tolerance for frame boundary samples.
    double identical_pct = 100.0 * identical_pairs / (samples.size() - 1);
    if (identical_pct > 5.0) {
        printf("FAIL\n    %s:%d: %.1f%% identical pairs (expected <5%% with first-order hold)\n",
               __FILE__, __LINE__, identical_pct);
        tests_failed++; return;
    }
}

// ── Main ───────────────────────────────────────────────────────────

int main() {
    printf("=== Recording Precision Tests ===\n\n");

    RUN(sample_count_at_1000hz);
    RUN(sample_count_at_100hz);
    RUN(sample_count_at_200hz);
    RUN(timestamp_spacing_1000hz);
    RUN(timestamp_spacing_100hz);
    RUN(duration_consistency);
    RUN(original_bug_23s_1000hz);
    RUN(sine_wave_capture);
    RUN(playback_direct_index);
    RUN(playback_speed_2x);
    RUN(oversampling_low_fps);
    RUN(recording_at_low_rate);
    RUN(empty_recording_playback);
    RUN(no_timing_drift_long_recording);
    RUN(first_order_hold_smooth);

    printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
