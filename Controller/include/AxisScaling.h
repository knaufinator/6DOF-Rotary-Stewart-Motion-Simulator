#ifndef AXIS_SCALING_H
#define AXIS_SCALING_H

#include <math.h>
#include "InverseKinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define AXIS_COUNT 6

/**
 * Per-axis scaling configuration.
 * Scales are derived from platform geometry via computeAxisScalesFromGeometry()
 * and represent the maximum safe displacement per axis (±scale).
 * Raw input (0 to max_raw) is mapped through mapRawToPosition() to physical position.
 *
 * scale[]    — max displacement per axis: mm for translation, degrees for rotation
 * is_angle[] — 1 if the axis output should be converted from degrees to radians
 */
typedef struct {
    float scale[AXIS_COUNT];
    int   is_angle[AXIS_COUNT];
} AxisScaleConfig;

/**
 * Initialize axis scales by probing the IK workspace.
 * Binary-searches each axis to find the max displacement where all servos
 * remain within their physical limits (±60°), then applies a safety margin.
 * Result: scales are derived entirely from platform geometry — no magic numbers.
 *
 * @param config      Output scale config to populate
 * @param stewart     Platform geometry (must be initialized first)
 * @param margin      Safety factor (0.0–1.0), e.g. 0.90 = use 90% of max range
 */
void computeAxisScalesFromGeometry(AxisScaleConfig* config, const StewartConfig* stewart, float margin);

/**
 * Map 6 raw values (0 to max_raw range) to physical position using per-axis scales.
 * Output: [surge, sway, heave, roll_rad, pitch_rad, yaw_rad]
 *
 * Equivalent to:
 *   position[i] = mapfloat(raw[i], 0, max_raw, -scale, +scale)
 *   if (is_angle[i]) position[i] *= (PI / 180)
 *
 * Firmware: max_raw = 4094 (binary protocol)
 * SIL:      max_raw = (1 << bit_range) - 1  (e.g. 4095 for 12-bit, 65535 for 16-bit)
 */
void mapRawToPosition(const float raw[6], const AxisScaleConfig* config, float max_raw, float position[6]);

#ifdef __cplusplus
}
#endif

#endif // AXIS_SCALING_H
