#pragma once

#include <atomic>
#include <cstdint>
#if defined(__GNUC__)
#define STEP_ALWAYS_INLINE __attribute__((always_inline)) inline
#else
#define STEP_ALWAYS_INLINE inline
#endif

// Stop requests cross from task context to the period ISR. Only the ISR advances
// the drain/guard counters. Three LOW-gate periods cover the two-period pipeline.
// This protocol is host-tested; the real pad timing still needs bench capture.
class StepQuiescence {
public:
    static_assert(ATOMIC_BOOL_LOCK_FREE == 2 && ATOMIC_INT_LOCK_FREE == 2,
                  "ISR stop protocol requires lock-free atomics");
    STEP_ALWAYS_INLINE void request_stop() { requested_.store(true, std::memory_order_release); }
    STEP_ALWAYS_INLINE bool stopping() const { return requested_.load(std::memory_order_acquire); }
    STEP_ALWAYS_INLINE bool drained() const { return quiet_.load(std::memory_order_acquire) >= 3; }
    void prepare_resume() {
        resume_guard_ = 3;
        quiet_.store(0, std::memory_order_relaxed);
        requested_.store(false, std::memory_order_release);
    }
    STEP_ALWAYS_INLINE bool allow_new_steps() const { return !stopping() && resume_guard_ == 0; }
    // Called AFTER writing all three operator gates and booking prior pulses.
    STEP_ALWAYS_INLINE void period_complete(bool gate_released = false) {
        if (stopping()) {
            if (gate_released) { quiet_.store(0, std::memory_order_release); return; }
            const uint32_t n = quiet_.load(std::memory_order_relaxed);
            if (n < 3) quiet_.store(n + 1, std::memory_order_release);
        } else {
            if (resume_guard_ != 0) --resume_guard_;
        }
    }
private:
    std::atomic<bool> requested_{false};
    std::atomic<uint32_t> quiet_{0};
    uint8_t resume_guard_ = 0;
};
#undef STEP_ALWAYS_INLINE
