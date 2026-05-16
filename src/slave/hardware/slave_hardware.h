#pragma once

#include <stdint.h>

struct SlaveXMotorStepTiming {
    uint32_t loop_foc_us;
    uint32_t move_us;
    uint32_t read_us;
    uint32_t loop_foc_ran;
};

float applySlaveXMotorTarget(float target_angle_rad,
                             float fallback_actual_angle_rad,
                             SlaveXMotorStepTiming *timing);

// slave_hardware
// 职责：从机紫光 MOS、安全输出、MT6701/SimpleFOC X 轴硬件初始化和目标输出。
// 运行约束：setUvPen() 只由安全任务调用；applySlaveXMotorTarget() 可在本地控制
// 控制步中调用，内部不得打印、分配内存或等待无线事件。

// 设置紫光笔 MOS 输出。只有安全任务应该调用它，其他模块只表达 pen 命令或联锁状态。
void setUvPen(bool enabled);

// 上电第一步调用：关闭 UV 和两路电机使能，建立失效保护默认状态。
void configureSlaveSafeOutputs();

// 初始化真实 X 轴电机硬件。默认编译配置下不启用硬件，只锁存禁用故障并返回 false。
bool setupSlaveXMotorHardware();

// 向真实电机写入目标角，或在硬件关闭时返回仿真角度。
float applySlaveXMotorTarget(float target_angle_rad, float fallback_actual_angle_rad);
