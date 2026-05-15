#pragma once

#include <Arduino.h>

#include "shared_types.h"

// 当前从机仍使用 100us 本地控制周期。该值不属于 common 协议契约。
static constexpr uint32_t SLAVE_CONTROL_LOOP_PERIOD_US = 100UL;
static constexpr uint32_t CONTROL_LOOP_PERIOD_US = SLAVE_CONTROL_LOOP_PERIOD_US;

// slave_config
// 职责：集中从机侧任务调度、X 轴纸面几何和临时电机测试参数。
//
// 这里是从机的“调参入口”。运动控制只读取这些常量，不把投影距离、
// 中心角、方向、稳定误差或任务优先级硬编码在算法中。
//
// 运行约束：这里只放编译期或只读配置，不访问 GPIO、ESP-NOW 或电机驱动。
// 后续阶段：正式 2208 云台接回时，通过 SLAVE_X_MOTOR_POLE_PAIRS 和本配置重新标定。

// 默认关闭真实 X 轴电机输出。第一阶段先跑通信闭环、仿真跟随和 UV 安全联锁。
#ifndef SLAVE_X_MOTOR_HW_ENABLED
#define SLAVE_X_MOTOR_HW_ENABLED 0
#endif

// 从机 ESP-NOW 通信开关：实际主从联调默认开启；只在隔离运动/UV 安全排查时临时关闭。
#ifndef SLAVE_ESPNOW_ENABLED
#define SLAVE_ESPNOW_ENABLED 1
#endif

// DengFoc/BLDCDriver3PWM 使能脚默认按高电平有效处理。
// 如果实测 EN 低电平有效，必须在 build_flags 中设为 0 后再开真实 X 轴输出。
#ifndef SLAVE_DRIVER_ENABLE_ACTIVE_HIGH
#define SLAVE_DRIVER_ENABLE_ACTIVE_HIGH 1
#endif

// Core 1 专门跑 X 轴控制；Core 0 负责 ESP-NOW、UV 安全和串口打印。
static constexpr BaseType_t SLAVE_CONTROL_CORE = 1;
static constexpr BaseType_t SLAVE_IO_CORE = 0;

// 控制任务优先级最高；UV 安全高于普通通信/状态，便于及时关闭紫光灯。
static constexpr UBaseType_t SLAVE_CONTROL_TASK_PRIORITY = configMAX_PRIORITIES - 1;
static constexpr UBaseType_t SLAVE_COMM_TASK_PRIORITY = 3;
static constexpr UBaseType_t SLAVE_SAFETY_TASK_PRIORITY = 2;
static constexpr UBaseType_t SLAVE_STATUS_TASK_PRIORITY = 1;

// 栈大小按当前 Arduino + ESP-NOW + SimpleFOC 组合预留；新增复杂逻辑后要复查水位。
static constexpr uint32_t SLAVE_CONTROL_TASK_STACK_BYTES = 8192;
static constexpr uint32_t SLAVE_COMM_TASK_STACK_BYTES = 4096;
static constexpr uint32_t SLAVE_SAFETY_TASK_STACK_BYTES = 4096;
static constexpr uint32_t SLAVE_STATUS_TASK_STACK_BYTES = 4096;

// FreeRTOS tick 固定为 1 ms；当前从机本地控制周期为 100us。
static constexpr uint32_t SLAVE_CONTROL_STEPS_PER_TICK = 1000UL / CONTROL_LOOP_PERIOD_US;
// 从机控制定时器保持和主机一致的 100 us esp_timer 节拍，避免高优先级 busy-wait 饿死 Idle。
static constexpr uint32_t SLAVE_CONTROL_TIMER_PERIOD_US = CONTROL_LOOP_PERIOD_US;
static constexpr uint32_t SLAVE_CONTROL_TIMER_TIMEOUT_MS = 10UL;

// 从机 X 轴纸面到云台角度的配置。
// center_angle_rad 是 A4 中心对应的云台角；direction 用来适配机械方向；
// throw_distance_mm 是紫光灯到纸面的投影距离；settle_error_rad 决定允许开 UV 的误差窗口；
// simulated_response_alpha 只在真实电机关闭时用于仿真跟随，方便先验证通信和安全逻辑。
struct SlaveXAxisConfig {
    float center_angle_rad;
    float direction;
    float throw_distance_mm;
    float settle_error_rad;
    float simulated_response_alpha;
};

extern const SlaveXAxisConfig kSlaveXAxis;

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

// 当前单轴联调使用固定主机 MAC。加入配对流程前，从机不会自动扫描主机。
extern const uint8_t kSlavePeerMasterAddress[6];
