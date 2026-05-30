#include "slave/control/slave_coordinate_mapper.h"

#include <math.h>

#include "common/math/clamp.h"
#include "common/protocol/protocol_units.h"
#include "slave/config/slave_config.h"

namespace {

const SlaveAxisConfig &axisConfig(AxisId axis) {
    return (axis == AXIS_Y) ? kSlaveYAxis : kSlaveXAxis;
}

const SlaveAxisLimitConfig &axisLimitConfig(AxisId axis) {
    return (axis == AXIS_Y) ? kSlaveYAxisLimit : kSlaveXAxisLimit;
}

}  // namespace

PaperGeometry slaveCurrentPaperGeometry() {
    PaperGeometry geometry = kSlavePaperGeometry;
    geometry.x_center_angle_rad = kSlaveXAxis.geometry.center_angle_rad;
    geometry.y_center_angle_rad = kSlaveYAxis.geometry.center_angle_rad;
    geometry.x_sign = kSlaveXAxis.geometry.direction_sign;
    geometry.y_sign = kSlaveYAxis.geometry.direction_sign;
    return geometry;
}

float slaveAxisHalfRangeMm(AxisId axis) {
    const float paper_half = (axis == AXIS_Y) ? (kSlavePaperGeometry.height_mm * 0.5f)
                                             : (kSlavePaperGeometry.width_mm * 0.5f);
    const float configured_half = axisConfig(axis).geometry.half_range_mm;
    return (configured_half > 0.0f) ? fminf(configured_half, paper_half) : paper_half;
}

float slaveAxisLimitMinMm(AxisId axis) {
    const float half_range_mm = slaveAxisHalfRangeMm(axis);
    return fmaxf(-half_range_mm, axisLimitConfig(axis).min_mm);
}

float slaveAxisLimitMaxMm(AxisId axis) {
    const float half_range_mm = slaveAxisHalfRangeMm(axis);
    return fminf(half_range_mm, axisLimitConfig(axis).max_mm);
}

PaperPointMm slaveNormToPaperPointMm(int16_t x_norm, int16_t y_norm) {
    return normToPaperPointMm(x_norm, y_norm, slaveCurrentPaperGeometry());
}

GimbalAnglesRad slavePaperPointToGimbalAnglesRad(const PaperPointMm &point) {
    const PaperGeometry geometry = slaveCurrentPaperGeometry();
    const PaperPointMm limited = clampPaperPointMm(point, geometry, nullptr);
    return paperPointToGimbalAnglesRad(limited, geometry);
}

float slaveAxisNormToPaperMm(AxisId axis, int16_t norm) {
    return normToUnit(norm) * slaveAxisHalfRangeMm(axis);
}

float slaveClampAxisPaperMm(AxisId axis, float mm, bool *clamped) {
    const float min_mm = slaveAxisLimitMinMm(axis);
    const float max_mm = slaveAxisLimitMaxMm(axis);
    const float limited = clampFloat(mm, min_mm, max_mm);
    if (clamped != nullptr) {
        *clamped = limited != mm;
    }
    return limited;
}

float slaveAxisPaperMmToGimbalAngleRad(AxisId axis, float mm) {
    const float limited = slaveClampAxisPaperMm(axis, mm, nullptr);
    return slaveLimitedAxisPaperMmToGimbalAngleRad(axis, limited);
}

float slaveLimitedAxisPaperMmToGimbalAngleRad(AxisId axis, float limited_mm) {
    const PaperGeometry geometry = slaveCurrentPaperGeometry();
    if (axis == AXIS_Y) {
        return geometry.y_center_angle_rad +
               geometry.y_sign * atanf((limited_mm + geometry.center_y_mm) / geometry.distance_mm);
    }
    return geometry.x_center_angle_rad +
           geometry.x_sign * atanf((limited_mm + geometry.center_x_mm) / geometry.distance_mm);
}

int16_t slaveAxisGimbalAngleRadToNorm(AxisId axis, float angle_rad) {
    const PaperGeometry geometry = slaveCurrentPaperGeometry();
    const bool is_y = axis == AXIS_Y;
    const float center_angle_rad =
        is_y ? geometry.y_center_angle_rad : geometry.x_center_angle_rad;
    const float direction = is_y ? geometry.y_sign : geometry.x_sign;
    const float center_mm = is_y ? geometry.center_y_mm : geometry.center_x_mm;
    const float axis_angle_rad = (angle_rad - center_angle_rad) * direction;
    const float mm = tanf(axis_angle_rad) * geometry.distance_mm - center_mm;
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
