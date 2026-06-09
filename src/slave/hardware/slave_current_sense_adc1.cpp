#include "slave/hardware/slave_current_sense_adc1.h"

#include "driver/adc.h"
#include "slave/config/slave_config.h"

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
extern "C" void recordSlaveTimingCurrentSenseUs(uint32_t duration_us) __attribute__((weak));
#endif

// 构造函数只保存引脚和换算系数，不做硬件初始化，避免全局对象阶段访问外设。
SlaveAdc1CurrentSense::SlaveAdc1CurrentSense(float shunt_resistor,
                                             float amp_gain,
                                             int pinA,
                                             int pinB,
                                             int pinC)
    : InlineCurrentSense(shunt_resistor, amp_gain, pinA, pinB, pinC),
      pin_a_(pinA),
      pin_b_(pinB),
      has_phase_c_(pinC != NOT_SET) {
    offset_ia = 0.0f;
    offset_ib = 0.0f;
    offset_ic = 0.0f;
    refreshFastScale();
}

// ESP32-S3 ADC1 GPIO 到通道号的映射检查；不支持的 GPIO 直接初始化失败。
bool SlaveAdc1CurrentSense::gpioToAdc1Channel(int pin, uint8_t &channel) {
    switch (pin) {
    case 1:
        channel = 0;
        return true;
    case 2:
        channel = 1;
        return true;
    case 3:
        channel = 2;
        return true;
    case 4:
        channel = 3;
        return true;
    case 5:
        channel = 4;
        return true;
    case 6:
        channel = 5;
        return true;
    case 7:
        channel = 6;
        return true;
    case 8:
        channel = 7;
        return true;
    case 9:
        channel = 8;
        return true;
    case 10:
        channel = 9;
        return true;
    default:
        return false;
    }
}

namespace {

constexpr int kSlaveAdcPrimeReads = 8;
constexpr adc_atten_t kSlaveAdcAttenuation = ADC_ATTEN_DB_12;

int clampSlaveAdcRaw(int raw) {
    if (raw < 0) {
        return 0;
    }
    const int max_raw = static_cast<int>(kSlaveCurrentSenseHardware.adc_raw_max);
    return (raw > max_raw) ? max_raw : raw;
}

}  // namespace

// IDF 单次读取会在 Wi-Fi 初始化后重新取得 ADC1 控制权，避免 RTC 快路径返回冻结值。
int SlaveAdc1CurrentSense::readFastRaw(uint8_t channel) {
    const int raw = adc1_get_raw(static_cast<adc1_channel_t>(channel));
    return clampSlaveAdcRaw(raw);
}

int SlaveAdc1CurrentSense::readFastRawUnmasked(uint8_t channel) {
    return adc1_get_raw(static_cast<adc1_channel_t>(channel));
}

void SlaveAdc1CurrentSense::publishLastSample(int raw_a,
                                              int raw_b,
                                              const PhaseCurrent_s &current) {
    telemetry_divider_++;
    if (telemetry_divider_ < SLAVE_CURRENT_SENSE_TELEMETRY_EVERY_N_STEPS) {
        return;
    }
    telemetry_divider_ = 0;

    sample_seq_++;
    last_raw_a_ = raw_a;
    last_raw_b_ = raw_b;
    last_current_a_ = current.a;
    last_current_b_ = current.b;
    last_current_c_ = current.c;
    sample_seq_++;
}

// 初始化 ADC 采样对象：当前只支持 A/B 两相高侧采样。
int SlaveAdc1CurrentSense::init() {
    if (has_phase_c_) {
        initialized = false;
        return 0;
    }

    if (!gpioToAdc1Channel(pin_a_, chan_a_) || !gpioToAdc1Channel(pin_b_, chan_b_)) {
        initialized = false;
        return 0;
    }

    if (adc1_config_width(ADC_WIDTH_BIT_12) != ESP_OK ||
        adc1_config_channel_atten(static_cast<adc1_channel_t>(chan_a_),
                                  kSlaveAdcAttenuation) != ESP_OK ||
        adc1_config_channel_atten(static_cast<adc1_channel_t>(chan_b_),
                                  kSlaveAdcAttenuation) != ESP_OK) {
        initialized = false;
        return 0;
    }

    for (int i = 0; i < kSlaveAdcPrimeReads; ++i) {
        (void)readFastRaw(chan_a_);
        (void)readFastRaw(chan_b_);
    }

    refreshFastScale();
    initialized = true;
    return 1;
}

// 跳过 SimpleFOC 自动相位对齐，采样方向由手动诊断参数决定。
int SlaveAdc1CurrentSense::driverAlign(float align_voltage) {
    (void)align_voltage;
    return 1;
}

void SlaveAdc1CurrentSense::refreshFastScale() {
    raw_to_current_a_ = kSlaveCurrentSenseHardware.adc_raw_to_voltage_v * gain_a;
    raw_to_current_b_ = kSlaveCurrentSenseHardware.adc_raw_to_voltage_v * gain_b;
    offset_current_a_ = offset_ia * gain_a;
    offset_current_b_ = offset_ib * gain_b;
}

// SimpleFOC 每次 loopFOC 会调用这里读取相电流，是控制热路径的一部分。
PhaseCurrent_s SlaveAdc1CurrentSense::getPhaseCurrents() {
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t read_start_us = micros();
#endif
    // 热路径只做两次 ADC 原始采样和预计算线性换算，不做 offset 重算或诊断打印。
    const int raw_a = readFastRaw(chan_a_);
    const int raw_b = readFastRaw(chan_b_);

    PhaseCurrent_s current;
    current.a = static_cast<float>(raw_a) * raw_to_current_a_ - offset_current_a_;
    current.b = static_cast<float>(raw_b) * raw_to_current_b_ - offset_current_b_;
    current.c = 0.0f;
    publishLastSample(raw_a, raw_b, current);
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    if (recordSlaveTimingCurrentSenseUs) {
        recordSlaveTimingCurrentSenseUs(micros() - read_start_us);
    }
#endif
    return current;
}

// 诊断读取 A 相原始 ADC 值。
int SlaveAdc1CurrentSense::readRawA() const {
    if (!initialized) {
        return 0;
    }
    return readFastRaw(chan_a_);
}

// 诊断读取 B 相原始 ADC 值。
int SlaveAdc1CurrentSense::readRawB() const {
    if (!initialized) {
        return 0;
    }
    return readFastRaw(chan_b_);
}

// 兼容现有 raw16 诊断字段；IDF 后端原生输出为 12bit，因此与 raw_adc 同量纲。
int SlaveAdc1CurrentSense::readRawUnmaskedA() const {
    if (!initialized) {
        return 0;
    }
    return readFastRawUnmasked(chan_a_);
}

// 兼容现有 raw16 诊断字段；IDF 后端原生输出为 12bit，因此与 raw_adc 同量纲。
int SlaveAdc1CurrentSense::readRawUnmaskedB() const {
    if (!initialized) {
        return 0;
    }
    return readFastRawUnmasked(chan_b_);
}

int SlaveAdc1CurrentSense::lastRawA() const {
    return last_raw_a_;
}

int SlaveAdc1CurrentSense::lastRawB() const {
    return last_raw_b_;
}

PhaseCurrent_s SlaveAdc1CurrentSense::lastPhaseCurrents() const {
    PhaseCurrent_s current = {};
    uint32_t before = 0;
    uint32_t after = 0;
    do {
        before = sample_seq_;
        current.a = last_current_a_;
        current.b = last_current_b_;
        current.c = last_current_c_;
        after = sample_seq_;
    } while (before != after || (after & 1U) != 0U);
    return current;
}
