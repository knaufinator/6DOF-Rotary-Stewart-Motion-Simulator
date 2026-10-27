// R13BenchPulse.h — bounded constant-rate pulse schedule for R13 bench mode (step 12).
//
// Pure and allocation-free for host tests. The bench timer calls due(now_us) and
// advances the axis-0 target by the returned count; the production step engine
// emits each step. Limits (R13_BENCH_MAX_*) were raised 2026-10-20 for the camera-calibrated
// bench; the step-12 envelope is now enforced by the host scripts and the firmware runaway guard.
#ifndef R13_BENCH_PULSE_H
#define R13_BENCH_PULSE_H

#include <stdint.h>

#define R13_BENCH_MAX_HZ     250000   // raised 2026-10-20 for the camera-calibrated bench (was 5000)
#define R13_BENCH_MAX_COUNT  1000000  // raised 2026-10-20 (was 10000); envelope is enforced by the host scripts

class R13BenchPulse {
public:
    // Validate and start: |n| pulses at hz, sign of n = direction. Returns false if out of range.
    bool start(int32_t hz, int32_t n, int64_t now_us) {
        if (hz < 1 || hz > R13_BENCH_MAX_HZ) return false;
        if (n == 0 || n > R13_BENCH_MAX_COUNT || n < -R13_BENCH_MAX_COUNT) return false;
        _hz = hz;
        _total = n < 0 ? -n : n;
        _dir = n < 0 ? -1 : 1;
        _emitted = 0;
        _t0 = now_us;
        _active = true;
        return true;
    }

    void stop() { _active = false; }

    // Signed number of new steps due since the last call (never exceeds the remaining count).
    int32_t due(int64_t now_us) {
        if (!_active) return 0;
        int64_t elapsed = now_us - _t0;
        if (elapsed < 0) elapsed = 0;
        int64_t should = elapsed * _hz / 1000000 + 1;  // first pulse immediately
        if (should > _total) should = _total;
        int32_t n = (int32_t)(should - _emitted);
        _emitted = should;
        if (_emitted >= _total) _active = false;
        return n * _dir;
    }

    bool    active()  const { return _active; }
    int32_t emitted() const { return (int32_t)_emitted * _dir; }
    int32_t total()   const { return (int32_t)_total * _dir; }

private:
    int64_t _t0 = 0, _total = 0, _emitted = 0;
    int32_t _hz = 0;
    int     _dir = 1;
    bool    _active = false;
};

#endif // R13_BENCH_PULSE_H
