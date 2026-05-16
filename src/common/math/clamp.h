#pragma once

// 通用浮点限幅：先比较下限，再比较上限，避免目标电流或归一化值越界。
inline float clampFloat(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

