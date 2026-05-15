#pragma once

#include <stdint.h>

#include "common/math/clamp.h"

static constexpr int16_t NORM_MIN = static_cast<int16_t>(-32767 - 1);
static constexpr int16_t NORM_MAX = 32767;

// 物理测试几何。协议包只传归一化坐标，不传电角度或编码器原始值。
static constexpr float MASTER_KNOB_HALF_RANGE_DEG = 90.0f;
static constexpr float A4_WIDTH_MM = 210.0f;
static constexpr float A4_HEIGHT_MM = 297.0f;
static constexpr float PAPER_EDGE_MARGIN_MM = 10.0f;
static constexpr float PLOT_X_HALF_RANGE_MM = (A4_WIDTH_MM * 0.5f) - PAPER_EDGE_MARGIN_MM;
static constexpr float PLOT_Y_HALF_RANGE_MM = (A4_HEIGHT_MM * 0.5f) - PAPER_EDGE_MARGIN_MM;
static constexpr float DEFAULT_THROW_DISTANCE_MM = 700.0f;

inline int16_t unitToNorm(float unit) {
    unit = clampFloat(unit, -1.0f, 1.0f);
    if (unit <= -1.0f) {
        return NORM_MIN;
    }
    return static_cast<int16_t>(unit * static_cast<float>(NORM_MAX));
}

inline float normToUnit(int16_t norm) {
    if (norm == NORM_MIN) {
        return -1.0f;
    }
    return clampFloat(static_cast<float>(norm) / static_cast<float>(NORM_MAX), -1.0f, 1.0f);
}

inline int16_t signedAngleDegToNorm(float angle_deg, float half_range_deg) {
    const float limited = clampFloat(angle_deg, -half_range_deg, half_range_deg);
    return unitToNorm(limited / half_range_deg);
}

inline float normToSignedAngleDeg(int16_t norm, float half_range_deg) {
    return normToUnit(norm) * half_range_deg;
}

inline int16_t angleDegToNorm(float angle_deg) {
    return signedAngleDegToNorm(angle_deg, MASTER_KNOB_HALF_RANGE_DEG);
}

inline float normToAngleDeg(int16_t norm) {
    return normToSignedAngleDeg(norm, MASTER_KNOB_HALF_RANGE_DEG);
}

inline int16_t percentToNorm(float percent) {
    if (percent < -100.0f) {
        percent = -100.0f;
    } else if (percent > 100.0f) {
        percent = 100.0f;
    }
    return static_cast<int16_t>((percent / 100.0f) * static_cast<float>(NORM_MAX));
}

inline float normToPercent(int16_t norm) {
    return (static_cast<float>(norm) / static_cast<float>(NORM_MAX)) * 100.0f;
}

