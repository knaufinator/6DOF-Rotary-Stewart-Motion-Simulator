#pragma once

// Composition of the host-tested first-seek guard and six-drive alarm latch.
// HardwareQualified=false is the production R14 staging type until physical
// Hall/local/drive inputs and the independent pulse gate are proven.
#include "R14SeekInterlock.h"
#include "R14DriveAlarmLatch.h"

namespace r14 {

template <bool HardwareQualified>
class MotionAdmission {
public:
    typedef SeekInterlock::Inputs Inputs;
    typedef SeekInterlock::SeekLimits SeekLimits;

    MotionAdmission(uint64_t input_age_us, uint64_t arm_window_us,
                    uint64_t home_request_window_us)
        : seek_(input_age_us, arm_window_us),
          home_window_us_(home_request_window_us),
          home_requested_(false), home_request_us_(0) {}

    bool configure_axis(int axis, SeekLimits limits) {
        return HardwareQualified && seek_.configure_axis(axis, limits);
    }

    bool submit_inputs(const Inputs& in, uint64_t now_us) {
        if (!HardwareQualified) return false;
        // A normalized drive alarm latches even before an operator arms.
        for (int i = 0; i < SeekInterlock::kAxes; ++i) {
            if (in.drive_alarm[i]) {
                report_alarm(i);
                return false;
            }
        }
        if (!seek_.observe(in, now_us)) {
            home_requested_ = false;
            return false;
        }
        return true;
    }

    bool request_operator_arm(uint64_t now_us) {
        return HardwareQualified && !alarms_.faulted() &&
               seek_.request_arm(now_us);
    }

    bool request_operator_home(uint64_t now_us) {
        if (!HardwareQualified || alarms_.faulted() || home_window_us_ == 0 ||
            !seek_.ready_for_seek(now_us)) return false;
        home_requested_ = true;
        home_request_us_ = now_us;
        return true;
    }

    bool begin_seek(int axis, uint64_t now_us) {
        if (!HardwareQualified || alarms_.faulted() || !home_requested_ ||
            now_us < home_request_us_ ||
            now_us - home_request_us_ > home_window_us_) return false;
        return seek_.begin_seek(axis, now_us);
    }

    bool reserve_seek_pulse(int axis, int direction, uint64_t now_us) {
        if (!HardwareQualified || alarms_.faulted() || !home_requested_) return false;
        if (!seek_.reserve_seek_pulse(axis, direction, now_us)) {
            home_requested_ = false;
            return false;
        }
        return true;
    }

    void report_alarm(int axis) {
        alarms_.assert_alarm(axis);
        seek_.emergency_fault(SeekInterlock::DRIVE_FAULT, axis);
        home_requested_ = false;
    }

    void report_reconnect_or_reset() {
        seek_.emergency_fault(SeekInterlock::INPUT_LOST);
        home_requested_ = false;
    }

    bool faulted() const {
        return alarms_.faulted() || seek_.state() == SeekInterlock::GLOBAL_FAULT;
    }
    uint8_t alarm_mask() const { return alarms_.axis_mask(); }
    SeekInterlock::State seek_state() const { return seek_.state(); }
    uint32_t reserved_pulses(int axis) const { return seek_.reserved_pulses(axis); }
    bool home_request_pending() const { return home_requested_; }
    bool normal_motion_permitted() const { return false; }
    bool hardware_qualified() const { return HardwareQualified; }

private:
    SeekInterlock seek_;
    DriveAlarmLatch alarms_;
    uint64_t home_window_us_;
    bool home_requested_;
    uint64_t home_request_us_;
};

} // namespace r14
