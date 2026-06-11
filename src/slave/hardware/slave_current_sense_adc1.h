#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>

#include "slave/hardware/slave_adc1_dma_sampler.h"

// 从机电流采样适配器：SimpleFOC 热路径只读取 ADC1 DMA 锁存快照。
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
    bool waitNextRawPair(uint32_t &last_sequence,
                         uint32_t timeout_ms,
                         int &raw_a,
                         int &raw_b) const;
    int lastRawA() const;
    int lastRawB() const;
    PhaseCurrent_s lastPhaseCurrents() const;
    uint32_t readErrorCount() const;
    uint16_t consecutiveReadErrors() const;
    bool readFaulted() const;

private:
    void publishLastSample(int raw_a, int raw_b, const PhaseCurrent_s &current);

    int pin_a_ = NOT_SET;
    int pin_b_ = NOT_SET;
    bool has_phase_c_ = false;
    SlaveAdc1DmaSlot slot_a_ = SLAVE_ADC1_DMA_SLOT_X_A;
    SlaveAdc1DmaSlot slot_b_ = SLAVE_ADC1_DMA_SLOT_X_B;
    float raw_to_voltage_v_ = 0.0f;
    volatile uint32_t sample_seq_ = 0;
    volatile int last_raw_a_ = 0;
    volatile int last_raw_b_ = 0;
    volatile float last_current_a_ = 0.0f;
    volatile float last_current_b_ = 0.0f;
    volatile float last_current_c_ = 0.0f;
};
