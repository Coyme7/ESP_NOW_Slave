#pragma once

struct SlaveTrajectoryConfig {
    float draw_speed_mm_s;      // 落笔速度，单位 mm/s。
    float lift_speed_mm_s;      // 抬笔速度，单位 mm/s。
    float accel_mm_s2;          // 加速度，单位 mm/s^2。
    float command_deadband_mm;  // 命令死区，单位 mm。
};

struct SlaveAxisTrajectoryConfig {
    float settle_error_mm;      // 到位阈值，单位 mm。
};
