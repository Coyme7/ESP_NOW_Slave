#pragma once

#include "slave/config/types/slave_current_sense_types.h"

// 电流采样硬件换算配置。
// shunt/gain 必须与 DengV3 驱动板实物一致，ADC 满量程按 DB_12 级 3.10V 计算。
static constexpr SlaveCurrentSenseHardwareConfig kSlaveCurrentSenseHardware = {
    0.01f,           // 分流电阻，单位 ohm。
    50.0f,           // 电流采样放大倍数。
    3.10f,           // ADC 满量程电压，单位 V。
    4095.0f,         // ADC raw 最大值。
    3.10f / 4095.0f, // ADC raw 到电压换算系数，单位 V/count。
    true,            // 跳过 SimpleFOC driverAlign，采样方向由低电流实测参数决定。
};

// MCPWM 峰值定相采样：总 ADC 转换约 8kHz，A/B 交替后每相和完整采样对约 4kHz。
static constexpr uint32_t kSlaveCurrentSenseTargetConversionHz = 8000UL;
static constexpr uint32_t kSlaveCurrentSenseSyncMeasureMs = 25UL;
static constexpr uint32_t kSlaveCurrentSensePairStaleUs = 500UL;

// 近轨样本会放大非线性和削顶误差，不能进入电流环。
static constexpr int kSlaveCurrentSenseRailMarginRaw = 32;

// 连续四次采样拒绝或控制读取陈旧后禁用对应轴。
static constexpr uint16_t kSlaveCurrentSenseAdcConsecutiveErrorLimit = 4U;

// 零电流同步校准验收阈值。
static constexpr float kSlaveCurrentSenseCalibrationMeanMinRatio = 0.05f;
static constexpr float kSlaveCurrentSenseCalibrationMeanMaxRatio = 0.95f;
static constexpr float kSlaveCurrentSenseCalibrationStddevMaxRaw = 40.0f;
static constexpr int kSlaveCurrentSenseCalibrationPeakToPeakMaxRaw = 200;
static constexpr int kSlaveCurrentSenseRuntimeBaselineDeltaMaxRaw = 32;

// X/Y 轴电流采样方向符号。
// 各轴符号保留当前硬件配置，必须在同步采样稳定后用低电流实测复核。
static constexpr SlaveCurrentSenseAxisConfig kSlaveXCurrentSenseAxis = {
    1,  // X A 相采样方向符号。
    -1, // X B 相采样方向符号。
};

static constexpr SlaveCurrentSenseAxisConfig kSlaveYCurrentSenseAxis = {
    -1, // Y A 相采样方向符号。
    -1, // Y B 相采样方向符号。
};

// 电流采样诊断和 offset 校准参数。
// 这些延时只在启动诊断/校准路径使用，不进入控制热路径。
static constexpr SlaveCurrentSenseDiagConfig kSlaveCurrentSenseDiag = {
    3.0f, // 诊断注入电压，单位 V。
    5U,    // 早期采样等待时间，单位 ms。
    80U,   // 稳定采样等待时间，单位 ms。
    8U,    // offset 校准前 ADC 预读次数。
    1000U, // offset 校准平均次数。
    80U,   // offset 校准前等待时间，单位 ms。
};
