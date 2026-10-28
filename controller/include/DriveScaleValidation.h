#pragma once
#include <cstdint>
#include <cstring>
#include <limits>

inline bool drive_finite(float v) {
    uint32_t bits; std::memcpy(&bits,&v,sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u; // works under -ffast-math
}
inline bool valid_drive_scale(int encoder, float gear, float planetary) {
    if (!drive_finite(gear) || !drive_finite(planetary) ||
        encoder < 1 || encoder > 100000 || gear <= 0 || gear > 1000 ||
        planetary <= 0 || planetary > 1000) return false;
    const double limit = 72.0 * (double(encoder)*4.0/double(gear))*double(planetary)/360.0;
    // Keep a full negative-to-positive delta inside signed 32-bit count space.
    return limit >= 1.0 && limit <= double(std::numeric_limits<int32_t>::max()/2);
}
