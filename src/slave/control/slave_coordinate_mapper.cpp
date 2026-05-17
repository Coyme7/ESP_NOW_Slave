#include "slave/control/slave_coordinate_mapper.h"

#include <math.h>

#include "common/math/clamp.h"
#include "common/protocol/protocol_units.h"
#include "slave/config/slave_config.h"

namespace {

const SlaveAxisConfig &axisConfig(AxisId axis) {
    return (axis == AXIS_Y) ? kSlaveYAxis : kSlaveXAxis;
}

}  // namespace

float slaveAxisHalfRangeMm(AxisId axis) {
    return axisConfig(axis).half_range_mm;
}

float slaveAxisNormToPaperMm(AxisId axis, int16_t norm) {
    return normToUnit(norm) * slaveAxisHalfRangeMm(axis);
}

float slaveClampAxisPaperMm(AxisId axis, float mm, bool *clamped) {
    const float half_range_mm = slaveAxisHalfRangeMm(axis);
    const float min_mm = fmaxf(-half_range_mm, kSlaveAxisLimit.min_mm);
    const float max_mm = fminf(half_range_mm, kSlaveAxisLimit.max_mm);
    const float limited = clampFloat(mm, min_mm, max_mm);
    if (clamped != nullptr) {
        *clamped = limited != mm;
    }
    return limited;
}

float slaveAxisPaperMmToGimbalAngleRad(AxisId axis, float mm) {
    const SlaveAxisConfig &config = axisConfig(axis);
    const float limited_mm = slaveClampAxisPaperMm(axis, mm, nullptr);
    return config.center_angle_rad +
           (config.direction * atanf(limited_mm / config.throw_distance_mm));
}

int16_t slaveAxisGimbalAngleRadToNorm(AxisId axis, float angle_rad) {
    const SlaveAxisConfig &config = axisConfig(axis);
    const float axis_angle_rad = (angle_rad - config.center_angle_rad) * config.direction;
    const float mm = tanf(axis_angle_rad) * config.throw_distance_mm;
    return unitToNorm(mm / slaveAxisHalfRangeMm(axis));
}

float slaveXNormToPaperMm(int16_t x_norm) {
    return slaveAxisNormToPaperMm(AXIS_X, x_norm);
}

float slavePaperMmToGimbalAngleRad(float x_mm) {
    return slaveAxisPaperMmToGimbalAngleRad(AXIS_X, x_mm);
}

int16_t slaveGimbalAngleRadToXNorm(float angle_rad) {
    return slaveAxisGimbalAngleRadToNorm(AXIS_X, angle_rad);
}
