#pragma once

#include <Arduino.h>

#include "shared_types.h"

// 运行模式、硬件边界和跨分类静态检查。
#include "slave/config/slave_build_options.h"

// 日志、状态输出和 timing 诊断。
// 需要放在 task/control 之前，保证派生的 SLAVE_TIMING_* 宏可被热路径编译期裁剪使用。
#include "slave/config/slave_log_config.h"

// 任务周期、绑核、优先级和栈。
#include "slave/config/slave_task_config.h"

// 通信、控制周期和真实硬件开关。
#include "slave/config/slave_comm_config.h"
#include "slave/config/slave_control_config.h"
#include "slave/config/slave_hardware_config.h"

// slave_config
// 职责：集中从机侧任务调度、X 轴纸面几何和临时电机测试参数。
//
// 这里是从机的“调参入口”。运动控制只读取这些常量，不把投影距离、
// 中心角、方向、稳定误差或任务优先级硬编码在算法中。
//
// 运行约束：这里只放编译期或只读配置，不访问 GPIO、ESP-NOW 或电机驱动。
// 后续阶段：正式 2208 云台接回时，通过 SLAVE_X_MOTOR_POLE_PAIRS 和本配置重新标定。

// 从机单轴纸面到云台角度的配置。
// center_angle_rad 是纸面中心对应的云台角；direction 用来适配机械方向。
// half_range_mm 是该轴允许工作半幅；默认 X/Y 都按 -125..+125mm 安全范围限制。
// simulated_response_alpha 只在真实电机关闭时用于仿真跟随，方便先验证通信和安全逻辑。
struct SlaveAxisConfig {
    float center_angle_rad;
    float direction;
    float throw_distance_mm;
    float half_range_mm;
    float settle_error_rad;
    float simulated_response_alpha;
};

using SlaveXAxisConfig = SlaveAxisConfig;

extern const SlaveAxisConfig kSlaveXAxis;
extern const SlaveAxisConfig kSlaveYAxis;

// 从机 X 轴真实位置闭环调参入口。
// 当前只有 X 轴，参数先偏慢偏软，确保能看清方向、收敛和温升。
struct SlaveMotorFocConfig {
    float supply_voltage_v;
    float driver_voltage_limit_v;
    float motor_voltage_limit_v;
    float align_voltage_v;
    float velocity_limit_rad_s;
    float angle_p;
    float velocity_pid_p;
    float velocity_pid_i;
    float velocity_pid_d;
    float velocity_pid_ramp;
    float velocity_lpf_tf;
    float angle_lpf_tf;
};

extern const SlaveMotorFocConfig kSlaveMotorFoc;

// 从机 X 轴纸面轨迹平滑参数。
// 速度单位 mm/s，加速度单位 mm/s^2，死区和到位阈值单位 mm。
struct SlaveTrajectoryConfig {
    float draw_speed_mm_s;
    float lift_speed_mm_s;
    float accel_mm_s2;
    float command_deadband_mm;
    float settle_error_mm;
};

extern const SlaveTrajectoryConfig kSlaveTrajectory;

// X/Y 软件限位。默认限制到 -125mm..+125mm，目标超限时 clamp 并在状态日志中显示。
struct SlaveAxisLimitConfig {
    float min_mm;
    float max_mm;
};

extern const SlaveAxisLimitConfig kSlaveAxisLimit;

// 当前单轴联调使用固定主机 MAC。加入配对流程前，从机不会自动扫描主机。
extern const uint8_t kSlavePeerMasterAddress[6];
