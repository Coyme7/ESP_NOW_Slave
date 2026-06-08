#include "slave/control/slave_coordinate_mapper.h"

#include <math.h>

#include "common/math/clamp.h"
#include "common/protocol/protocol_units.h"
#include "slave/config/slave_config.h"

namespace {

struct SlaveAxisDerivedGeometry {
    float half_range_mm;
    float limit_min_mm;
    float limit_max_mm;
    float center_angle_rad;
    float center_mm;
    float direction_sign;
    float distance_mm;
    float inv_distance;
};

constexpr float constexprMin(float a, float b) {
    return (a < b) ? a : b;
}

constexpr float constexprMax(float a, float b) {
    return (a > b) ? a : b;
}

constexpr float configuredHalfRange(float configured_half_mm, float paper_half_mm) {
    return (configured_half_mm > 0.0f)
               ? constexprMin(configured_half_mm, paper_half_mm)
               : paper_half_mm;
}

constexpr SlaveAxisDerivedGeometry makeAxisDerivedGeometry(const SlaveAxisConfig &axis,
                                                           const SlaveAxisLimitConfig &limit,
                                                           float paper_half_mm,
                                                           float center_mm,
                                                           float distance_mm) {
    return {
        configuredHalfRange(axis.geometry.half_range_mm, paper_half_mm),
        constexprMax(-configuredHalfRange(axis.geometry.half_range_mm, paper_half_mm), limit.min_mm),
        constexprMin(configuredHalfRange(axis.geometry.half_range_mm, paper_half_mm), limit.max_mm),
        axis.geometry.center_angle_rad,
        center_mm,
        static_cast<float>(axis.geometry.direction_sign),
        distance_mm,
        1.0f / distance_mm,
    };
}

static constexpr SlaveAxisDerivedGeometry kSlaveXAxisDerived =
    makeAxisDerivedGeometry(kSlaveXAxis,
                            kSlaveXAxisLimit,
                            kSlavePaperGeometry.width_mm * 0.5f,
                            kSlavePaperGeometry.center_x_mm,
                            kSlavePaperGeometry.distance_mm);

static constexpr SlaveAxisDerivedGeometry kSlaveYAxisDerived =
    makeAxisDerivedGeometry(kSlaveYAxis,
                            kSlaveYAxisLimit,
                            kSlavePaperGeometry.height_mm * 0.5f,
                            kSlavePaperGeometry.center_y_mm,
                            kSlavePaperGeometry.distance_mm);

const SlaveAxisDerivedGeometry &axisDerived(AxisId axis) {
    return (axis == AXIS_Y) ? kSlaveYAxisDerived : kSlaveXAxisDerived;
}

}  // namespace

PaperGeometry slaveCurrentPaperGeometry() {
    PaperGeometry geometry = kSlavePaperGeometry;
    geometry.x_center_angle_rad = kSlaveXAxisDerived.center_angle_rad;
    geometry.y_center_angle_rad = kSlaveYAxisDerived.center_angle_rad;
    geometry.x_sign = kSlaveXAxisDerived.direction_sign;
    geometry.y_sign = kSlaveYAxisDerived.direction_sign;
    return geometry;
}

float slaveAxisHalfRangeMm(AxisId axis) {
    return axisDerived(axis).half_range_mm;
}

float slaveAxisLimitMinMm(AxisId axis) {
    return axisDerived(axis).limit_min_mm;
}

float slaveAxisLimitMaxMm(AxisId axis) {
    return axisDerived(axis).limit_max_mm;
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
    return normToUnit(norm) * axisDerived(axis).half_range_mm;
}

float slaveClampAxisPaperMm(AxisId axis, float mm, bool *clamped) {
    const SlaveAxisDerivedGeometry &geometry = axisDerived(axis);
    const float limited = clampFloat(mm, geometry.limit_min_mm, geometry.limit_max_mm);
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
    const SlaveAxisDerivedGeometry &geometry = axisDerived(axis);
    return geometry.center_angle_rad +
           geometry.direction_sign * atanf((limited_mm + geometry.center_mm) * geometry.inv_distance);
}

int16_t slaveAxisGimbalAngleRadToNorm(AxisId axis, float angle_rad) {
    const SlaveAxisDerivedGeometry &geometry = axisDerived(axis);
    const float axis_angle_rad =
        (angle_rad - geometry.center_angle_rad) * geometry.direction_sign;
    const float mm = tanf(axis_angle_rad) * geometry.distance_mm - geometry.center_mm;
    return unitToNorm(mm / geometry.half_range_mm);
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
