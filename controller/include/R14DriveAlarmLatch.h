#pragma once

#include <atomic>
#include <cstdint>

namespace r14 {

// Normalized alarm events only. The electrical polarity and source pin are
// deliberately outside this class until the installed drive is measured.
// No clear method: recovery requires a new supervisor/home cycle.
class DriveAlarmLatch {
public:
    DriveAlarmLatch() : bits_(0) {}

    void assert_alarm(int axis) {
        const uint8_t bit = axis >= 0 && axis < 6
            ? static_cast<uint8_t>(1u << axis) : static_cast<uint8_t>(0x80u);
        bits_.fetch_or(bit, std::memory_order_acq_rel);
    }

    bool faulted() const { return bits_.load(std::memory_order_acquire) != 0; }
    uint8_t axis_mask() const {
        return bits_.load(std::memory_order_acquire) & static_cast<uint8_t>(0x3fu);
    }
    bool invalid_axis_event() const {
        return (bits_.load(std::memory_order_acquire) & 0x80u) != 0;
    }

private:
    std::atomic<uint8_t> bits_;
};

} // namespace r14
