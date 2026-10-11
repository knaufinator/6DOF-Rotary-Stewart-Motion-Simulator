#pragma once

// R14 offline trajectory schedule. This class never drives STEP/DIR and is not
// connected to the production command path. A safety supervisor must authorize
// every move and every pulse independently of this mathematical schedule.
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace r14 {

class SixAxisTrajectory {
public:
    static const int kAxes = 6;

    struct AxisLimits {
        int32_t min_count;
        int32_t max_count;
        double max_speed_steps_s;
        double max_accel_steps_s2;
        double max_jerk_steps_s3;
    };

    enum State { DISARMED, RUNNING, COMPLETE, FAULT };
    enum Error { NONE, BAD_CONFIG, OUT_OF_RANGE, DURATION_EXCEEDED,
                 CLOCK_REVERSED, UPDATE_MISSED, ABORTED };

    SixAxisTrajectory() : state_(DISARMED), error_(NONE), start_us_(0),
                          last_us_(0), duration_us_(0), max_gap_us_(0) {
        for (int i = 0; i < kAxes; ++i) start_[i] = target_[i] = last_count_[i] = 0;
    }

    // One common time law for all six count deltas. No replan/restart is
    // accepted on a running, completed, or faulted instance.
    bool begin(const int32_t start[kAxes], const int32_t target[kAxes],
               const AxisLimits limits[kAxes], uint64_t now_us,
               uint64_t max_gap_us, uint64_t max_duration_us) {
        if (state_ != DISARMED) return false;
        if (!start || !target || !limits || max_gap_us == 0 ||
            max_duration_us == 0) return fail(BAD_CONFIG);

        double required_s = 0.0;
        bool any_move = false;
        for (int i = 0; i < kAxes; ++i) {
            const AxisLimits& c = limits[i];
            if (c.min_count >= c.max_count ||
                !positive_finite(c.max_speed_steps_s) ||
                !positive_finite(c.max_accel_steps_s2) ||
                !positive_finite(c.max_jerk_steps_s3)) return fail(BAD_CONFIG);
            if (start[i] < c.min_count || start[i] > c.max_count ||
                target[i] < c.min_count || target[i] > c.max_count)
                return fail(OUT_OF_RANGE);

            const double distance = std::fabs(static_cast<double>(target[i]) -
                                              static_cast<double>(start[i]));
            if (distance > 0.0) {
                any_move = true;
                const double speed_s = kPeakSpeed * distance / c.max_speed_steps_s;
                const double accel_s = std::sqrt(kPeakAccel * distance /
                                                 c.max_accel_steps_s2);
                const double jerk_s = std::cbrt(kPeakJerk * distance /
                                                c.max_jerk_steps_s3);
                const double axis_s = std::fmax(speed_s, std::fmax(accel_s, jerk_s));
                if (!finite_bits(axis_s)) return fail(BAD_CONFIG);
                required_s = std::fmax(required_s, axis_s);
            }
            start_[i] = start[i];
            target_[i] = target[i];
            last_count_[i] = start[i];
        }

        if (any_move) {
            const double required_us = std::ceil(required_s * 1000000.0);
            if (!finite_bits(required_us) || required_us < 1.0 ||
                required_us > static_cast<double>(max_duration_us) ||
                required_us >= static_cast<double>(std::numeric_limits<uint64_t>::max()))
                return fail(DURATION_EXCEEDED);
            duration_us_ = static_cast<uint64_t>(required_us);
        } else {
            duration_us_ = 0;
        }
        start_us_ = last_us_ = now_us;
        max_gap_us_ = max_gap_us;
        error_ = NONE;
        state_ = any_move ? RUNNING : COMPLETE;
        return true;
    }

    // Returns false and leaves out[] untouched after a timing fault. The
    // caller must never interpret an old out[] value as motion permission.
    bool sample(uint64_t now_us, int32_t out[kAxes]) {
        if (!out || (state_ != RUNNING && state_ != COMPLETE)) return false;
        if (state_ == COMPLETE) {
            for (int i = 0; i < kAxes; ++i) out[i] = target_[i];
            return true;
        }
        if (now_us < last_us_ || now_us < start_us_) return fail(CLOCK_REVERSED);
        if (now_us - last_us_ > max_gap_us_) return fail(UPDATE_MISSED);
        const uint64_t elapsed_us = now_us - start_us_;
        if (elapsed_us >= duration_us_) {
            for (int i = 0; i < kAxes; ++i) {
                last_count_[i] = target_[i];
                out[i] = target_[i];
            }
            last_us_ = now_us;
            state_ = COMPLETE;
            return true;
        }

        const double u = static_cast<double>(elapsed_us) /
                         static_cast<double>(duration_us_);
        // Septic smoothstep: s, speed, acceleration and jerk are all zero at
        // each end. Exact peak speed = 2.1875, acceleration < 8, jerk = 52.5
        // in normalized time. Integer targets inherit <= 0.5 count rounding.
        const double u2 = u * u;
        const double u4 = u2 * u2;
        const double s = u4 * (35.0 + u * (-84.0 + u * (70.0 - 20.0 * u)));
        int32_t next[kAxes];
        for (int i = 0; i < kAxes; ++i) {
            const double continuous = static_cast<double>(start_[i]) +
                (static_cast<double>(target_[i]) - static_cast<double>(start_[i])) * s;
            const int64_t rounded = static_cast<int64_t>(std::llround(continuous));
            const int32_t low = start_[i] < target_[i] ? start_[i] : target_[i];
            const int32_t high = start_[i] > target_[i] ? start_[i] : target_[i];
            if (rounded < low || rounded > high) return fail(BAD_CONFIG);
            next[i] = static_cast<int32_t>(rounded);
        }
        for (int i = 0; i < kAxes; ++i) {
            last_count_[i] = next[i];
            out[i] = next[i];
        }
        last_us_ = now_us;
        return true;
    }

    void abort() { fail(ABORTED); }
    State state() const { return state_; }
    Error error() const { return error_; }
    uint64_t duration_us() const { return duration_us_; }
    int32_t last_count(int axis) const {
        return axis >= 0 && axis < kAxes ? last_count_[axis] : 0;
    }

private:
    static constexpr double kPeakSpeed = 2.1875;
    static constexpr double kPeakAccel = 8.0; // conservative; exact 7.5132...
    static constexpr double kPeakJerk = 52.5;

    static bool finite_bits(double value) {
        // main/CMakeLists.txt currently uses -ffast-math, under which
        // std::isfinite(NaN) can be optimized to true. Inspect IEEE-754 bits.
        static_assert(sizeof(double) == sizeof(uint64_t), "64-bit double required");
        static_assert(std::numeric_limits<double>::is_iec559, "IEEE-754 required");
        uint64_t bits;
        std::memcpy(&bits, &value, sizeof(bits));
        return (bits & UINT64_C(0x7ff0000000000000)) !=
               UINT64_C(0x7ff0000000000000);
    }
    static bool positive_finite(double value) {
        return finite_bits(value) && value > 0.0;
    }
    bool fail(Error reason) {
        state_ = FAULT;
        error_ = reason;
        return false;
    }

    State state_;
    Error error_;
    uint64_t start_us_;
    uint64_t last_us_;
    uint64_t duration_us_;
    uint64_t max_gap_us_;
    int32_t start_[kAxes];
    int32_t target_[kAxes];
    int32_t last_count_[kAxes];
};

} // namespace r14
