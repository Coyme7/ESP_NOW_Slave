#pragma once

#include <stdint.h>

#include "common/math/axis_math.h"

// 从机单轴运行态。X 和 Y 共用该结构，默认 Y 只走软件仿真，不访问硬件。
struct SlaveAxisRuntime {
    AxisId axis;
    int16_t command_norm;
    float command_mm;
    float smooth_mm;
    float target_angle_rad;
    float actual_angle_rad;
    float track_err_mrad;
    float total_err_mm;
    bool limit_hit;
    bool clamped;
};
