#pragma once

// The live and debounced normally-closed inputs must both be released.
static inline bool estop_motion_blocked(bool paused, bool live_active, bool debounced_active) {
    return paused || live_active || debounced_active;
}

static inline bool estop_reset_allowed(bool live_active, bool debounced_active) {
    return !live_active && !debounced_active;
}
