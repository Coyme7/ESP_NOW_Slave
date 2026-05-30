#pragma once

struct SlaveMotorVoltageConfig {
    float supply_v;          // 驱动供电电压，单位 V。
    float driver_limit_v;    // 驱动限压，单位 V。
    float motor_limit_v;     // 闭环电机限压，单位 V。
    float open_loop_limit_v; // 开环限压，单位 V。
    float align_v;           // FOC 对齐电压，单位 V。
};

struct SlaveMotorLimitConfig {
    float velocity_rad_s; // 速度上限，单位 rad/s。
};

struct SlavePositionLoopConfig {
    float p; // 位置环 P。
};

struct SlaveVelocityLoopConfig {
    float p;           // 速度环 P。
    float i;           // 速度环 I。
    float d;           // 速度环 D。
    float output_ramp; // 速度环输出斜率。
};

struct SlaveMotorFilterConfig {
    float velocity_tf; // 速度滤波 Tf。
    float angle_tf;    // 角度滤波 Tf。
};

struct SlaveMotorFocConfig {
    SlaveMotorVoltageConfig voltage;  // 电压和对齐参数。
    SlaveMotorLimitConfig limit;      // 运动限幅。
    SlavePositionLoopConfig position; // 位置环参数。
    SlaveVelocityLoopConfig velocity; // 速度环参数。
    SlaveMotorFilterConfig filter;    // SimpleFOC 滤波参数。
};
