#pragma once

#include <stdint.h>

struct SlaveAxisGeometryConfig {
    float center_angle_rad;  // 纸面中心云台角，单位 rad。
    int8_t direction_sign;   // 机械方向符号。
    float throw_distance_mm; // 投影距离，单位 mm。
    float half_range_mm;     // 纸面半幅，单位 mm。
};

struct SlaveAxisTrackingConfig {
    float settle_error_rad;         // 到位阈值，单位 rad。
    float simulated_response_alpha; // 仿真跟随系数。
};

struct SlaveAxisConfig {
    SlaveAxisGeometryConfig geometry; // 纸面到云台几何映射。
    SlaveAxisTrackingConfig tracking; // 跟随和到位判定。
};

struct SlaveAxisLimitConfig {
    float min_mm; // 低端软限位，单位 mm。
    float max_mm; // 高端软限位，单位 mm。
};
