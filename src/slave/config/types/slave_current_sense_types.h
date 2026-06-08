#pragma once

#include <stdint.h>

struct SlaveCurrentSenseHardwareConfig {
    float shunt_ohm;            // 分流电阻，单位 ohm。
    float gain;                 // 电流采样放大倍数。
    float adc_full_scale_v;     // ADC 满量程电压，单位 V。
    float adc_raw_max;          // ADC raw 最大值。
    float adc_raw_to_voltage_v; // ADC raw 到电压换算系数，单位 V/count。
    bool skip_align;            // 是否跳过 SimpleFOC driverAlign。
};

struct SlaveCurrentSenseAxisConfig {
    int8_t gain_sign_a; // A 相采样方向符号。
    int8_t gain_sign_b; // B 相采样方向符号。
};

struct SlaveCurrentSenseDiagConfig {
    float voltage_v;           // 诊断注入电压，单位 V。
    uint32_t early_ms;         // 早期采样等待时间，单位 ms。
    uint32_t settle_ms;        // 稳定采样等待时间，单位 ms。
    uint16_t adc_prime_reads;  // offset 校准前 ADC 预读次数。
    uint16_t offset_reads;     // offset 校准平均次数。
    uint32_t offset_settle_ms; // offset 校准前等待时间，单位 ms。
};
