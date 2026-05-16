#pragma once

#include "common/math/clamp.h"

// 一阶低通滤波：tf_s 越大越平滑，dt_s 越大单步跟随越快。
// 当 dt_s 或 tf_s 非法时直接返回输入，避免滤波状态卡死。
inline float lowPassFilter(float input, float previous, float dt_s, float tf_s) {
    if (tf_s <= 0.0f) {
        return input;
    }
    if (dt_s <= 0.0f) {
        return previous;
    }

    const float alpha = tf_s / (tf_s + dt_s);
    return alpha * previous + (1.0f - alpha) * input;
}

