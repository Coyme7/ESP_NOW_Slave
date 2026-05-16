#pragma once

#include <stdint.h>

#include "common/math/clamp.h"

// 协议中的坐标使用 int16_t 归一化范围，减少包大小并保证主从一致。
static constexpr int16_t NORM_MIN = static_cast<int16_t>(-32767 - 1);
static constexpr int16_t NORM_MAX = 32767;

// 物理测试几何。协议包只传归一化坐标，不传电角度或编码器原始值。
static constexpr float MASTER_KNOB_HALF_RANGE_DEG = 165.0f;
static constexpr float A4_WIDTH_MM = 210.0f;
static constexpr float A4_HEIGHT_MM = 297.0f;
static constexpr float PAPER_EDGE_MARGIN_MM = 10.0f;
// 当前 X 单轴联调使用 250mm 画幅，中心两侧各 125mm。
static constexpr float PLOT_X_HALF_RANGE_MM = 125.0f;
static constexpr float PLOT_Y_HALF_RANGE_MM = (A4_HEIGHT_MM * 0.5f) - PAPER_EDGE_MARGIN_MM;
static constexpr float DEFAULT_THROW_DISTANCE_MM = 300.0f;

// 把 [-1, 1] 的单位量映射到 int16_t；超界输入会被夹紧。
inline int16_t unitToNorm(float unit) {
    unit = clampFloat(unit, -1.0f, 1.0f);
    if (unit <= -1.0f) {
        return NORM_MIN;
    }
    return static_cast<int16_t>(unit * static_cast<float>(NORM_MAX));
}

// 把 int16_t 归一化值还原成 [-1, 1] 浮点量。
inline float normToUnit(int16_t norm) {
    if (norm == NORM_MIN) {
        return -1.0f;
    }
    return clampFloat(static_cast<float>(norm) / static_cast<float>(NORM_MAX), -1.0f, 1.0f);
}

// 把以 0 为中心的角度映射为协议归一化坐标。
inline int16_t signedAngleDegToNorm(float angle_deg, float half_range_deg) {
    const float limited = clampFloat(angle_deg, -half_range_deg, half_range_deg);
    return unitToNorm(limited / half_range_deg);
}

// 从机可用它把 target_x_norm 还原成目标角度。
inline float normToSignedAngleDeg(int16_t norm, float half_range_deg) {
    return normToUnit(norm) * half_range_deg;
}

// 默认按主机旋钮半程范围转换角度到 norm。
inline int16_t angleDegToNorm(float angle_deg) {
    return signedAngleDegToNorm(angle_deg, MASTER_KNOB_HALF_RANGE_DEG);
}

// 默认按主机旋钮半程范围把 norm 转回角度。
inline float normToAngleDeg(int16_t norm) {
    return normToSignedAngleDeg(norm, MASTER_KNOB_HALF_RANGE_DEG);
}

// 把 0..100% 映射到 -1..1 的协议坐标，便于状态显示与调试。
inline int16_t percentToNorm(float percent) {
    if (percent < -100.0f) {
        percent = -100.0f;
    } else if (percent > 100.0f) {
        percent = 100.0f;
    }
    return static_cast<int16_t>((percent / 100.0f) * static_cast<float>(NORM_MAX));
}

// 把协议坐标转换成 0..100%，用于串口状态行。
inline float normToPercent(int16_t norm) {
    return (static_cast<float>(norm) / static_cast<float>(NORM_MAX)) * 100.0f;
}
