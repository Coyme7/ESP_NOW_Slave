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
    true,            // 跳过 SimpleFOC driverAlign，采样方向由 bring-up 参数决定。
};

// 连续四个完整 A/B 采样周期失败后禁用对应轴；从机 4kHz 下约为 1ms。
static constexpr uint16_t kSlaveCurrentSenseAdcConsecutiveErrorLimit = 4U;

// X/Y 轴电流采样方向符号。
// Y 默认复用已验证 X 符号；实机 bring-up 后可单独改 kSlaveYCurrentSenseAxis。
static constexpr SlaveCurrentSenseAxisConfig kSlaveXCurrentSenseAxis = {
    1,  // X A 相采样方向符号。
    -1, // X B 相采样方向符号。
};

static constexpr SlaveCurrentSenseAxisConfig kSlaveYCurrentSenseAxis = {
    1, // Y A 相采样方向符号。
    -1, // Y B 相采样方向符号。
};

// 电流采样诊断和 offset 校准参数。
// 这些延时只在启动诊断/校准路径使用，不进入控制热路径。
static constexpr SlaveCurrentSenseDiagConfig kSlaveCurrentSenseDiag = {
    0.30f, // 诊断注入电压，单位 V。
    5U,    // 早期采样等待时间，单位 ms。
    80U,   // 稳定采样等待时间，单位 ms。
    8U,    // offset 校准前 ADC 预读次数。
    256U,  // offset 校准平均次数。
    80U,   // offset 校准前等待时间，单位 ms。
};
