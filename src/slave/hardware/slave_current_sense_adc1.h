#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>

// 从机电流采样适配类：用 ESP32-S3 ADC1 两相采样对接 SimpleFOC InlineCurrentSense。
class SlaveAdc1CurrentSense : public InlineCurrentSense {
public:
    SlaveAdc1CurrentSense(float shunt_resistor,
                          float amp_gain,
                          int pinA,
                          int pinB,
                          int pinC = NOT_SET);

    int init() override;
    int driverAlign(float align_voltage, bool modulation_centered = false) override;
    PhaseCurrent_s getPhaseCurrents() override;

    int readRawA() const;
    int readRawB() const;
    int lastRawA() const;
    int lastRawB() const;
    PhaseCurrent_s lastPhaseCurrents() const;
    uint32_t readErrorCount() const;
    uint16_t consecutiveReadErrors() const;
    bool readFaulted() const;

private:
    static bool gpioToAdc1Channel(int pin, uint8_t &channel);
    bool readChannel(uint8_t channel, int &raw) const;
    void publishLastSample(int raw_a, int raw_b, const PhaseCurrent_s &current);

    int pin_a_ = NOT_SET;
    int pin_b_ = NOT_SET;
    bool has_phase_c_ = false;
    uint8_t chan_a_ = 0;
    uint8_t chan_b_ = 0;
    float raw_to_voltage_v_ = 0.0f;
    volatile uint32_t sample_seq_ = 0;
    volatile int last_raw_a_ = 0;
    volatile int last_raw_b_ = 0;
    volatile float last_current_a_ = 0.0f;
    volatile float last_current_b_ = 0.0f;
    volatile float last_current_c_ = 0.0f;
    mutable volatile uint32_t read_error_count_ = 0;
    volatile uint16_t consecutive_read_errors_ = 0;
};
