#pragma once

#include <stdint.h>

struct SlaveXMotorStepTiming {
    uint32_t loop_foc_us;
    uint32_t move_us;
    uint32_t sensor_us;
    uint32_t loop_foc_ran;
};

// 设置紫光 MOS 输出；只允许安全任务调用。
void setUvOutput(bool enabled);

// 上电第一步调用：关闭 UV 和电机使能，建立失效保护默认状态。
void configureSlaveSafeOutputs();

// 传感器与电机初始化由 SLAVE_RUN_MODE 选择调用路径。
bool setupSlaveXSensorHardware();
bool setupSlaveXMotorHardware();
bool setupSlaveYSensorHardware();
bool setupSlaveYMotorOpenLoopHardware();
bool setupSlaveYMotorClosedLoopHardware();

bool sampleSlaveYSensorForStatus(float *angle_rad, uint16_t *raw_angle);

float applySlaveXMotorTarget(float target_angle_rad,
                             float fallback_actual_angle_rad,
                             SlaveXMotorStepTiming *timing);

float runSlaveXMotorPerfIsolationStep(float target_angle_rad,
                                      float fallback_actual_angle_rad,
                                      SlaveXMotorStepTiming *timing);

float applySlaveYMotorTarget(float target_angle_rad,
                             float fallback_actual_angle_rad,
                             SlaveXMotorStepTiming *timing);

float applySlaveXMotorTarget(float target_angle_rad, float fallback_actual_angle_rad);
float runSlaveXMotorPerfIsolationStep(float target_angle_rad, float fallback_actual_angle_rad);
float applySlaveYMotorTarget(float target_angle_rad, float fallback_actual_angle_rad);
