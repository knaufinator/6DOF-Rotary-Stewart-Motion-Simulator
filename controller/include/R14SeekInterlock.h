#pragma once

// Hardware-independent R14 seek admission guard. It emits no STEP/DIR signal,
// does not establish home, and has no live IO mapping. The hardware pulse
// inhibit still has to respond independently between input samples.
#include <cstdint>

namespace r14 {

class SeekInterlock {
public:
    static const int kAxes = 6;

    // Every polarity is normalized by a future, qualified IO adapter. In
    // particular hall_closed means NC away from the 12 o'clock magnet, and
    // drive_alarm is true for the measured active alarm state.
    struct Inputs {
        uint64_t sampled_us;
        bool estop_closed;
        bool field_good;
        bool local_hold; // reserved legacy telemetry; never a motion requirement
        bool bottom_verified;
        bool calibration_valid;
        bool hall_closed[kAxes];
        bool drive_ready[kAxes];
        bool drive_alarm[kAxes];
    };

    struct SeekLimits {
        int8_t approach_direction; // -1 or +1 in commanded count space
        uint32_t min_edge_pulses;
        uint32_t max_seek_pulses;
        uint64_t max_seek_us;
    };

    enum State { DISARMED, ARMED_FOR_SEEK, ALL_EDGES_CAPTURED, GLOBAL_FAULT };
    enum Fault { NONE, BAD_CONFIG, CLOCK_OR_INPUT_STALE, INPUT_LOST,
                 DRIVE_FAULT, HALL_UNEXPECTED, HALL_TOO_EARLY,
                 SEEK_PULSE_LIMIT, SEEK_TIME_LIMIT, WRONG_DIRECTION };
    enum AxisState { NOT_STARTED, SEEKING, EDGE_FOUND };

    explicit SeekInterlock(uint64_t max_input_age_us,
                           uint64_t arm_request_window_us)
        : state_(DISARMED), fault_(NONE), fault_axis_(-1),
          max_age_us_(max_input_age_us), arm_window_us_(arm_request_window_us),
          have_sample_(false), armed_at_us_(0), last_now_us_(0) {
        for (int i = 0; i < kAxes; ++i) {
            configured_[i] = false;
            axis_[i] = NOT_STARTED;
            reserved_[i] = 0;
            seek_start_us_[i] = 0;
        }
        if (max_age_us_ == 0 || arm_window_us_ == 0) trip(BAD_CONFIG, -1);
    }

    bool configure_axis(int axis, SeekLimits limits) {
        if (state_ != DISARMED || axis < 0 || axis >= kAxes) return false;
        if ((limits.approach_direction != -1 && limits.approach_direction != 1) ||
            limits.max_seek_pulses == 0 || limits.max_seek_us == 0 ||
            limits.min_edge_pulses == 0 ||
            limits.min_edge_pulses > limits.max_seek_pulses)
            return trip(BAD_CONFIG, axis);
        limits_[axis] = limits;
        configured_[axis] = true;
        return true;
    }

    // Observe fresh normalized inputs periodically, including while idle.
    // Sampling alone never arms: an explicit operator request is required.
    bool observe(const Inputs& in, uint64_t now_us) {
        if (state_ == GLOBAL_FAULT) return false;
        if (in.sampled_us > now_us || now_us - in.sampled_us > max_age_us_ ||
            (have_sample_ &&
             (now_us < last_now_us_ || in.sampled_us < last_input_.sampled_us)))
            return trip(CLOCK_OR_INPUT_STALE, -1);

        last_input_ = in;
        last_now_us_ = now_us;
        have_sample_ = true;

        for (int i = 0; i < kAxes; ++i)
            if (in.drive_alarm[i] && state_ != DISARMED) return trip(DRIVE_FAULT, i);

        if (state_ == DISARMED) {
            return true; // unhealthy boot stays disarmed and cannot arm
        }
        if (!base_healthy(in, false)) return trip(INPUT_LOST, -1);

        for (int i = 0; i < kAxes; ++i) {
            if (axis_[i] == SEEKING) {
                // A Hall transition observed at or after the deadline is
                // late even if it is the first open sample. Check time
                // before accepting the edge so it cannot bypass the bound.
                if (now_us < seek_start_us_[i] ||
                    now_us - seek_start_us_[i] >= limits_[i].max_seek_us)
                    return trip(SEEK_TIME_LIMIT, i);
                if (!in.hall_closed[i]) {
                    if (reserved_[i] < limits_[i].min_edge_pulses)
                        return trip(HALL_TOO_EARLY, i);
                    axis_[i] = EDGE_FOUND;
                } else if (reserved_[i] >= limits_[i].max_seek_pulses)
                    return trip(SEEK_PULSE_LIMIT, i);
            } else if (axis_[i] == NOT_STARTED && !in.hall_closed[i]) {
                return trip(HALL_UNEXPECTED, i);
            } else if (axis_[i] == EDGE_FOUND && in.hall_closed[i]) {
                return trip(HALL_UNEXPECTED, i);
            }
        }
        bool all_found = true;
        for (int i = 0; i < kAxes; ++i) all_found &= axis_[i] == EDGE_FOUND;
        if (all_found) state_ = ALL_EDGES_CAPTURED;
        return true;
    }

    // Deliberate operator request; no physical hold-to-run input is required.
    // This admits only a bounded seek; no normal platform move is permitted.
    bool request_arm(uint64_t now_us) {
        if (state_ != DISARMED || !fresh(now_us) ||
            !base_healthy(last_input_, true) || !all_configured()) return false;
        armed_at_us_ = now_us;
        state_ = ARMED_FOR_SEEK;
        return true;
    }

    bool ready_for_seek(uint64_t now_us) const {
        return state_ == ARMED_FOR_SEEK && fresh(now_us) &&
               now_us >= armed_at_us_ && now_us - armed_at_us_ <= arm_window_us_ &&
               base_healthy(last_input_, false);
    }

    bool begin_seek(int axis, uint64_t now_us) {
        if (!ready_for_seek(now_us) || axis < 0 || axis >= kAxes ||
            axis_[axis] != NOT_STARTED ||
            !last_input_.hall_closed[axis]) return false;
        axis_[axis] = SEEKING;
        seek_start_us_[axis] = now_us;
        reserved_[axis] = 0;
        return true;
    }

    // Call before scheduling *each* prospective STEP edge. Reservation is
    // conservative if an edge is later suppressed. It is never proof that a
    // drive moved. A production pulse adapter must own actual edge accounting.
    bool reserve_seek_pulse(int axis, int direction, uint64_t now_us) {
        if (state_ != ARMED_FOR_SEEK || axis < 0 || axis >= kAxes ||
            axis_[axis] != SEEKING) return false;
        if (!fresh(now_us)) return trip(CLOCK_OR_INPUT_STALE, axis);
        if (!base_healthy(last_input_, false)) return trip(INPUT_LOST, axis);
        if (!last_input_.hall_closed[axis]) return trip(HALL_UNEXPECTED, axis);
        if (direction != limits_[axis].approach_direction)
            return trip(WRONG_DIRECTION, axis);
        if (now_us - seek_start_us_[axis] >= limits_[axis].max_seek_us)
            return trip(SEEK_TIME_LIMIT, axis);
        if (reserved_[axis] >= limits_[axis].max_seek_pulses)
            return trip(SEEK_PULSE_LIMIT, axis);
        ++reserved_[axis];
        return true;
    }

    void emergency_fault(Fault reason, int axis = -1) {
        trip(reason == NONE ? INPUT_LOST : reason, axis);
    }
    State state() const { return state_; }
    Fault fault() const { return fault_; }
    int fault_axis() const { return fault_axis_; }
    AxisState axis_state(int axis) const {
        return axis >= 0 && axis < kAxes ? axis_[axis] : NOT_STARTED;
    }
    uint32_t reserved_pulses(int axis) const {
        return axis >= 0 && axis < kAxes ? reserved_[axis] : 0;
    }
    bool normal_motion_permitted() const { return false; }

private:
    bool base_healthy(const Inputs& in, bool require_away_halls) const {
        if (!in.estop_closed || !in.field_good ||
            !in.bottom_verified || !in.calibration_valid) return false;
        for (int i = 0; i < kAxes; ++i) {
            if (!in.drive_ready[i] || in.drive_alarm[i]) return false;
            if (require_away_halls && !in.hall_closed[i]) return false;
        }
        return true;
    }
    bool all_configured() const {
        for (int i = 0; i < kAxes; ++i) if (!configured_[i]) return false;
        return true;
    }
    bool fresh(uint64_t now_us) const {
        return have_sample_ && now_us >= last_now_us_ &&
               now_us >= last_input_.sampled_us &&
               now_us - last_input_.sampled_us <= max_age_us_;
    }
    bool trip(Fault reason, int axis) {
        if (state_ != GLOBAL_FAULT) {
            state_ = GLOBAL_FAULT;
            fault_ = reason;
            fault_axis_ = axis;
        }
        return false;
    }

    State state_;
    Fault fault_;
    int fault_axis_;
    uint64_t max_age_us_;
    uint64_t arm_window_us_;
    Inputs last_input_;
    bool have_sample_;
    uint64_t armed_at_us_;
    uint64_t last_now_us_;
    SeekLimits limits_[kAxes];
    bool configured_[kAxes];
    AxisState axis_[kAxes];
    uint32_t reserved_[kAxes];
    uint64_t seek_start_us_[kAxes];
};

} // namespace r14
