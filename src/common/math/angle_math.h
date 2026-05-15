#pragma once

#include "common/math/clamp.h"

static constexpr float PI_F = 3.14159265358979323846f;

inline float degToRad(float deg) {
    return deg * PI_F / 180.0f;
}

inline float radToDeg(float rad) {
    return rad * 180.0f / PI_F;
}

inline float wrapAngleDegToSigned180(float angle_deg) {
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

inline float absoluteAngleDegToSignedAngleDeg(float absolute_angle_deg, float center_deg) {
    return wrapAngleDegToSigned180(absolute_angle_deg - center_deg);
}

inline float signedAngleDeltaDeg(float current_deg, float previous_deg) {
    return wrapAngleDegToSigned180(current_deg - previous_deg);
}

