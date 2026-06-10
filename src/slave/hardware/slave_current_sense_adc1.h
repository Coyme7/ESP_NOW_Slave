#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>
#include <driver/mcpwm_prelude.h>

struct SlaveCurrentSenseRawPair {
    bool valid;
    uint32_t sequence;
    int raw_a;
    int raw_b;
    uint32_t age_us;
    uint32_t skew_us;
};

struct SlaveCurrentSenseCalibrationStats {
    bool valid;
    uint32_t samples;
    float mean_a_raw;
    float mean_b_raw;
    float stddev_a_raw;
    float stddev_b_raw;
    int min_a_raw;
    int max_a_raw;
    int min_b_raw;
    int max_b_raw;
};

struct SlaveCurrentSenseSyncStats {
    bool sync_ready;
    uint32_t pwm_event_hz;
    uint32_t adc_conversion_hz;
    uint32_t phase_sample_hz;
    uint32_t pair_sequence;
    uint32_t pair_age_us;
    uint32_t pair_skew_us;
    uint32_t stale_count;
    uint32_t rail_count;
    uint32_t reject_count;
    uint32_t adc_read_errors;
    uint16_t consecutive_errors;
    SlaveCurrentSenseCalibrationStats calibration;
};

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
    uint8_t channelA() const;
    uint8_t channelB() const;
    const char *adcBackendName() const;
    PhaseCurrent_s lastPhaseCurrents() const;
    uint32_t readErrorCount() const;
    uint16_t consecutiveReadErrors() const;
    bool readFaulted() const;
    bool snapshotRawPair(SlaveCurrentSenseRawPair &pair) const;
    SlaveCurrentSenseSyncStats syncStats() const;
    void setCalibrationStats(const SlaveCurrentSenseCalibrationStats &stats);
    void armRuntimeValidation();

private:
    static bool gpioToAdc1Channel(int pin, uint8_t &channel);
    static bool initAdc1Pin(int pin, uint8_t &channel);
    static bool IRAM_ATTR pwmFullCallback(mcpwm_timer_handle_t timer,
                                          const mcpwm_timer_event_data_t *event_data,
                                          void *user_data);
    bool configurePwmSynchronizedSampling();
    bool IRAM_ATTR handlePwmFullEvent();
    bool readPinDirect(int pin, int &raw) const;
    bool IRAM_ATTR rawNearRail(int raw) const;
    uint32_t cyclesToUs(uint32_t cycles) const;
    void IRAM_ATTR publishRawPair(int raw_a,
                                  int raw_b,
                                  uint32_t publish_cycles,
                                  uint32_t skew_cycles);
    void publishLastSample(const PhaseCurrent_s &current);

    int pin_a_ = NOT_SET;
    int pin_b_ = NOT_SET;
    bool has_phase_c_ = false;
    uint8_t chan_a_ = 0;
    uint8_t chan_b_ = 0;
    float raw_to_voltage_v_ = 0.0f;
    uint32_t cpu_mhz_ = 1;
    volatile bool sync_ready_ = false;
    volatile bool sampling_enabled_ = false;
    volatile bool sample_phase_a_ = true;
    volatile bool pending_a_valid_ = false;
    volatile int pending_raw_a_ = 0;
    volatile uint32_t pending_a_cycles_ = 0;
    volatile uint32_t phase_accumulator_ = 0;
    volatile uint32_t pwm_event_hz_ = 0;
    volatile uint32_t measured_conversion_hz_ = 0;
    volatile uint32_t measured_phase_sample_hz_ = 0;
    volatile uint32_t pwm_event_count_ = 0;
    volatile uint32_t adc_conversion_count_ = 0;
    volatile uint32_t raw_pair_lock_ = 0;
    volatile uint32_t raw_pair_sequence_ = 0;
    volatile uint32_t raw_pair_publish_cycles_ = 0;
    volatile uint32_t raw_pair_skew_cycles_ = 0;
    volatile uint32_t sample_seq_ = 0;
    volatile int last_raw_a_ = 0;
    volatile int last_raw_b_ = 0;
    volatile float last_current_a_ = 0.0f;
    volatile float last_current_b_ = 0.0f;
    volatile float last_current_c_ = 0.0f;
    mutable volatile uint32_t read_error_count_ = 0;
    volatile uint32_t stale_count_ = 0;
    volatile uint32_t rail_count_ = 0;
    volatile uint32_t reject_count_ = 0;
    volatile uint16_t consecutive_rejects_ = 0;
    volatile uint16_t consecutive_stale_reads_ = 0;
    volatile bool runtime_validation_armed_ = false;
    volatile bool sampling_fault_latched_ = false;
    SlaveCurrentSenseCalibrationStats calibration_stats_ = {};
};
