#pragma once

#include <SimpleFOC.h>

#include "slave/config/types/slave_motor_types.h"
#include "slave/hardware/slave_current_sense_adc1.h"
#include "slave/hardware/slave_mt6701_sensor.h"

// 从机诊断模块共享的硬件上下文。
// 诊断代码只在启动/显式诊断路径运行，不进入控制热路径。
struct SlaveMotorDiagnosticsContext {
    const char *axis_name;
    BLDCMotor &motor;
    BLDCDriver3PWM &driver;
    SlaveAdc1CurrentSense &current_sense;
    SlaveMt6701Sensor &sensor;
    const SlaveMotorFocConfig &motor_config;
};

// 在驱动 EN/PWM 已进入运行态偏置后校准 ADC offset，保持原始电流采样公式不变。
bool calibrateSlaveCurrentSenseOffsets(SlaveMotorDiagnosticsContext &context);

// U/V/W 单相注入诊断，只在 SLAVE_ENABLE_CURRENT_SENSE_DIAG_TEST 下由启动流程调用。
void runSlaveCurrentSenseProbe(SlaveMotorDiagnosticsContext &context);
