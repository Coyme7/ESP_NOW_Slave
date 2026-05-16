#pragma once

#include "common/math/clamp.h"

// 圆周率常量使用 float，避免在 ESP32 热路径中引入 double 运算。
static constexpr float PI_F = 3.14159265358979323846f;

// 角度转弧度：SimpleFOC 内部常以弧度表示机械角。
inline float degToRad(float deg) {
    return deg * PI_F / 180.0f;
}

// 弧度转角度：串口状态和虚拟边界更适合使用度。
inline float radToDeg(float rad) {
    return rad * 180.0f / PI_F;
}

// 把任意角度折叠到 [-180, 180]，用于处理编码器跨 0/360 度边界。
inline float wrapAngleDegToSigned180(float angle_deg) {
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

// 绝对角减去机械中心后再 wrap，得到以旋钮中位为 0 的控制角。
inline float absoluteAngleDegToSignedAngleDeg(float absolute_angle_deg, float center_deg) {
    return wrapAngleDegToSigned180(absolute_angle_deg - center_deg);
}

// 两次采样角度的最短差值，用于速度估算，避免跨 180 度时出现大跳变。
inline float signedAngleDeltaDeg(float current_deg, float previous_deg) {
    return wrapAngleDegToSigned180(current_deg - previous_deg);
}

