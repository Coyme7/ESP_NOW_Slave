#pragma once

#include <math.h>
#include <stdint.h>

#include "common/math/clamp.h"
#include "common/protocol/protocol_units.h"

enum PaperProfile : uint8_t {
    PAPER_LEGACY_250 = 0,
    PAPER_A4_LANDSCAPE = 1,
    PAPER_A4_PORTRAIT = 2,
    PAPER_CUSTOM = 3,
};

struct PaperGeometry {
    float width_mm;
    float height_mm;
    float distance_mm;
    float center_x_mm;
    float center_y_mm;
    float x_center_angle_rad;
    float y_center_angle_rad;
    float x_sign;
    float y_sign;
};

struct PaperPointMm {
    float x_mm;
    float y_mm;
};

struct GimbalAnglesRad {
    float x_angle_rad;
    float y_angle_rad;
};

inline PaperGeometry makePaperGeometry(PaperProfile profile,
                                       float custom_width_mm,
                                       float custom_height_mm,
                                       float distance_mm) {
    PaperGeometry geometry = {};
    geometry.distance_mm = distance_mm;
    geometry.x_sign = 1.0f;
    geometry.y_sign = 1.0f;
    switch (profile) {
        case PAPER_A4_LANDSCAPE:
            geometry.width_mm = A4_LANDSCAPE_WIDTH_MM;
            geometry.height_mm = A4_LANDSCAPE_HEIGHT_MM;
            break;
        case PAPER_A4_PORTRAIT:
            geometry.width_mm = A4_PORTRAIT_WIDTH_MM;
            geometry.height_mm = A4_PORTRAIT_HEIGHT_MM;
            break;
        case PAPER_CUSTOM:
            geometry.width_mm = custom_width_mm;
            geometry.height_mm = custom_height_mm;
            break;
        case PAPER_LEGACY_250:
        default:
            geometry.width_mm = 250.0f;
            geometry.height_mm = 250.0f;
            break;
    }
    return geometry;
}

inline PaperPointMm normToPaperPointMm(int16_t x_norm,
                                       int16_t y_norm,
                                       const PaperGeometry &geometry) {
    PaperPointMm point = {};
    point.x_mm = normToUnit(x_norm) * geometry.width_mm * 0.5f;
    point.y_mm = normToUnit(y_norm) * geometry.height_mm * 0.5f;
    return point;
}

inline PaperPointMm clampPaperPointMm(const PaperPointMm &point,
                                      const PaperGeometry &geometry,
                                      bool *clamped) {
    PaperPointMm limited = {};
    limited.x_mm = clampFloat(point.x_mm, -geometry.width_mm * 0.5f, geometry.width_mm * 0.5f);
    limited.y_mm = clampFloat(point.y_mm, -geometry.height_mm * 0.5f, geometry.height_mm * 0.5f);
    if (clamped != nullptr) {
        *clamped = (limited.x_mm != point.x_mm) || (limited.y_mm != point.y_mm);
    }
    return limited;
}

inline GimbalAnglesRad paperPointToGimbalAnglesRad(const PaperPointMm &point,
                                                   const PaperGeometry &geometry) {
    GimbalAnglesRad angles = {};
    angles.x_angle_rad = geometry.x_center_angle_rad +
                         geometry.x_sign * atanf((point.x_mm + geometry.center_x_mm) /
                                                 geometry.distance_mm);
    angles.y_angle_rad = geometry.y_center_angle_rad +
                         geometry.y_sign * atanf((point.y_mm + geometry.center_y_mm) /
                                                 geometry.distance_mm);
    return angles;
}
