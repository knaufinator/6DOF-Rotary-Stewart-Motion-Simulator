#pragma once
#include <cstdint>

namespace controller {
// DATA18 wire format. Preserve the existing low-18-bit policy and minimum
// length; framing and source selection belong to the caller.
inline bool decode_motion_payload(const uint8_t* payload, int len, float raw[6]) {
    if (!payload || len < 18) return false;
    for (int i = 0; i < 6; ++i) {
        const uint32_t value = uint32_t(payload[3*i]) |
            (uint32_t(payload[3*i+1]) << 8) | (uint32_t(payload[3*i+2]) << 16);
        raw[i] = float(value & 0x3ffffu);
    }
    return true;
}

template<class Write>
bool guarded_motor_target(bool blocked, Write write) {
    return !blocked && write();
}
} // namespace controller
