#include "slave/hardware/slave_current_sense_adc1.h"

#include <esp_cpu.h>
#include <esp_intr_alloc.h>
#include <soc/sens_reg.h>

#include "current_sense/hardware_specific/esp32/esp32_adc_driver.h"
#include "drivers/hardware_specific/esp32/esp32_driver_mcpwm.h"
#include "drivers/hardware_specific/esp32/mcpwm_private.h"

#include "slave/config/slave_config.h"

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
extern "C" void recordSlaveTimingCurrentSenseUs(uint32_t duration_us) __attribute__((weak));
#endif

SlaveAdc1CurrentSense::SlaveAdc1CurrentSense(float shunt_resistor,
                                             float amp_gain,
                                             int pinA,
                                             int pinB,
                                             int pinC)
    : InlineCurrentSense(shunt_resistor, amp_gain, pinA, pinB, pinC),
      pin_a_(pinA),
      pin_b_(pinB),
      has_phase_c_(pinC != NOT_SET),
      raw_to_voltage_v_(kSlaveCurrentSenseHardware.adc_raw_to_voltage_v) {
    offset_ia = 0.0f;
    offset_ib = 0.0f;
    offset_ic = 0.0f;
}

namespace {

bool IRAM_ATTR slaveAdcRawInRange(int raw) {
    const int max_raw = static_cast<int>(kSlaveCurrentSenseHardware.adc_raw_max);
    return raw >= 0 && raw <= max_raw;
}

uint16_t IRAM_ATTR readSlaveAdc1ChannelFast(uint8_t channel) {
    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_SAR_M);
    SET_PERI_REG_BITS(SENS_SAR_MEAS1_CTRL2_REG,
                      SENS_SAR1_EN_PAD,
                      1UL << channel,
                      SENS_SAR1_EN_PAD_S);
    SET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_SAR_M);
    while (GET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG,
                             SENS_MEAS1_DONE_SAR) == 0U) {
    }
    return static_cast<uint16_t>(
        GET_PERI_REG_BITS2(SENS_SAR_MEAS1_CTRL2_REG,
                           SENS_MEAS1_DATA_SAR,
                           SENS_MEAS1_DATA_SAR_S));
}

uint32_t measuredRateHz(uint32_t samples, uint32_t elapsed_us) {
    if (samples == 0U || elapsed_us == 0U) {
        return 0U;
    }
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(samples) * 1000000ULL + elapsed_us / 2U) /
        elapsed_us);
}

}  // namespace

bool SlaveAdc1CurrentSense::gpioToAdc1Channel(int pin, uint8_t &channel) {
    const int8_t mapped_channel = digitalPinToAnalogChannel(pin);
    if (mapped_channel < 0 ||
        mapped_channel >= static_cast<int8_t>(SOC_ADC_MAX_CHANNEL_NUM)) {
        return false;
    }
    channel = static_cast<uint8_t>(mapped_channel);
    return true;
}

bool SlaveAdc1CurrentSense::initAdc1Pin(int pin, uint8_t &channel) {
    if (!gpioToAdc1Channel(pin, channel)) {
        return false;
    }
    return adcInit(static_cast<uint8_t>(pin));
}

bool SlaveAdc1CurrentSense::readPinDirect(int pin, int &raw) const {
    const int sample = static_cast<int>(adcRead(static_cast<uint8_t>(pin)));
    if (!slaveAdcRawInRange(sample)) {
        if (read_error_count_ != UINT32_MAX) {
            read_error_count_ = read_error_count_ + 1U;
        }
        return false;
    }
    raw = sample;
    return true;
}

bool SlaveAdc1CurrentSense::rawNearRail(int raw) const {
    const int max_raw = static_cast<int>(kSlaveCurrentSenseHardware.adc_raw_max);
    return raw <= kSlaveCurrentSenseRailMarginRaw ||
           raw >= (max_raw - kSlaveCurrentSenseRailMarginRaw);
}

uint32_t SlaveAdc1CurrentSense::cyclesToUs(uint32_t cycles) const {
    return cpu_mhz_ > 0U ? cycles / cpu_mhz_ : cycles;
}

void SlaveAdc1CurrentSense::publishRawPair(int raw_a,
                                           int raw_b,
                                           uint32_t publish_cycles,
                                           uint32_t skew_cycles) {
    raw_pair_lock_ = raw_pair_lock_ + 1U;
    __sync_synchronize();
    last_raw_a_ = raw_a;
    last_raw_b_ = raw_b;
    raw_pair_publish_cycles_ = publish_cycles;
    raw_pair_skew_cycles_ = skew_cycles;
    raw_pair_sequence_ = raw_pair_sequence_ + 1U;
    __sync_synchronize();
    raw_pair_lock_ = raw_pair_lock_ + 1U;
}

void SlaveAdc1CurrentSense::publishLastSample(
    const PhaseCurrent_s &current) {
    sample_seq_ = sample_seq_ + 1U;
    last_current_a_ = current.a;
    last_current_b_ = current.b;
    last_current_c_ = current.c;
    sample_seq_ = sample_seq_ + 1U;
}

bool SlaveAdc1CurrentSense::pwmFullCallback(
    mcpwm_timer_handle_t timer,
    const mcpwm_timer_event_data_t *event_data,
    void *user_data) {
    (void)timer;
    (void)event_data;
    if (user_data == nullptr) {
        return false;
    }
    return static_cast<SlaveAdc1CurrentSense *>(user_data)->handlePwmFullEvent();
}

bool SlaveAdc1CurrentSense::handlePwmFullEvent() {
    pwm_event_count_ = pwm_event_count_ + 1U;
    const uint32_t event_hz = pwm_event_hz_;
    if (!sampling_enabled_ || event_hz == 0U) {
        return false;
    }

    const uint32_t accumulated =
        phase_accumulator_ + kSlaveCurrentSenseTargetConversionHz;
    if (accumulated < event_hz) {
        phase_accumulator_ = accumulated;
        return false;
    }
    phase_accumulator_ = accumulated - event_hz;

    const bool sample_a = sample_phase_a_;
    const uint8_t channel = sample_a ? chan_a_ : chan_b_;
    const int raw = static_cast<int>(readSlaveAdc1ChannelFast(channel));
    const uint32_t sample_cycles = esp_cpu_get_cycle_count();
    adc_conversion_count_ = adc_conversion_count_ + 1U;

    const bool raw_in_range = slaveAdcRawInRange(raw);
    const bool near_rail = raw_in_range && rawNearRail(raw);
    if (!raw_in_range) {
        if (read_error_count_ != UINT32_MAX) {
            read_error_count_ = read_error_count_ + 1U;
        }
    }
    if (!raw_in_range || near_rail) {
        if (near_rail && rail_count_ != UINT32_MAX) {
            rail_count_ = rail_count_ + 1U;
        }
        if (reject_count_ != UINT32_MAX) {
            reject_count_ = reject_count_ + 1U;
        }
        if (consecutive_rejects_ != UINT16_MAX) {
            consecutive_rejects_ =
                static_cast<uint16_t>(consecutive_rejects_ + 1U);
        }
        if (consecutive_rejects_ >=
                kSlaveCurrentSenseAdcConsecutiveErrorLimit &&
            runtime_validation_armed_) {
            sampling_fault_latched_ = true;
        }
        pending_a_valid_ = false;
        sample_phase_a_ = true;
        return false;
    }

    if (sample_a) {
        pending_raw_a_ = raw;
        pending_a_cycles_ = sample_cycles;
        pending_a_valid_ = true;
        sample_phase_a_ = false;
        return false;
    }

    sample_phase_a_ = true;
    if (!pending_a_valid_) {
        if (reject_count_ != UINT32_MAX) {
            reject_count_ = reject_count_ + 1U;
        }
        if (consecutive_rejects_ != UINT16_MAX) {
            consecutive_rejects_ =
                static_cast<uint16_t>(consecutive_rejects_ + 1U);
        }
        if (consecutive_rejects_ >=
                kSlaveCurrentSenseAdcConsecutiveErrorLimit &&
            runtime_validation_armed_) {
            sampling_fault_latched_ = true;
        }
        return false;
    }

    const int raw_a = pending_raw_a_;
    const uint32_t skew_cycles = sample_cycles - pending_a_cycles_;
    pending_a_valid_ = false;
    consecutive_rejects_ = 0;
    publishRawPair(raw_a, raw, sample_cycles, skew_cycles);
    return false;
}

bool SlaveAdc1CurrentSense::configurePwmSynchronizedSampling() {
    if (driver == nullptr || driver->params == nullptr ||
        driver->type() != DriverType::BLDC) {
        return false;
    }

    auto *driver_params =
        static_cast<ESP32MCPWMDriverParams *>(driver->params);
    if (driver_params->timers[0] == nullptr) {
        return false;
    }

    auto *timer = reinterpret_cast<mcpwm_timer_t *>(driver_params->timers[0]);
    if (timer->on_full != nullptr) {
        return false;
    }

    const mcpwm_timer_event_callbacks_t callbacks = {
        .on_full = pwmFullCallback,
    };
    const mcpwm_timer_fsm_t saved_fsm = timer->fsm;
    timer->fsm = MCPWM_TIMER_FSM_INIT;
    const esp_err_t register_result =
        mcpwm_timer_register_event_callbacks(driver_params->timers[0],
                                             &callbacks,
                                             this);
    timer->fsm = saved_fsm;
    if (register_result != ESP_OK || esp_intr_enable(timer->intr) != ESP_OK) {
        return false;
    }

    const uint32_t event_count_before = pwm_event_count_;
    const uint32_t event_measure_start_us = micros();
    delay(kSlaveCurrentSenseSyncMeasureMs);
    const uint32_t event_measure_elapsed_us = micros() - event_measure_start_us;
    const uint32_t event_count = pwm_event_count_ - event_count_before;
    const uint32_t event_hz = measuredRateHz(event_count,
                                             event_measure_elapsed_us);
    if (event_hz < kSlaveCurrentSenseTargetConversionHz) {
        return false;
    }

    pwm_event_hz_ = event_hz;
    phase_accumulator_ = 0U;
    sample_phase_a_ = true;
    pending_a_valid_ = false;
    sampling_enabled_ = true;

    const uint32_t conversion_count_before = adc_conversion_count_;
    const uint32_t pair_count_before = raw_pair_sequence_;
    const uint32_t sample_measure_start_us = micros();
    delay(kSlaveCurrentSenseSyncMeasureMs);
    const uint32_t sample_measure_elapsed_us = micros() - sample_measure_start_us;
    const uint32_t conversions =
        adc_conversion_count_ - conversion_count_before;
    const uint32_t pairs = raw_pair_sequence_ - pair_count_before;
    measured_conversion_hz_ = measuredRateHz(conversions,
                                             sample_measure_elapsed_us);
    measured_phase_sample_hz_ = measuredRateHz(pairs,
                                               sample_measure_elapsed_us);
    if (pairs == 0U) {
        sampling_enabled_ = false;
        return false;
    }

    sync_ready_ = true;
    return true;
}

int SlaveAdc1CurrentSense::init() {
    if (has_phase_c_) {
        initialized = false;
        return 0;
    }

    if (!initAdc1Pin(pin_a_, chan_a_) || !initAdc1Pin(pin_b_, chan_b_)) {
        initialized = false;
        return 0;
    }

    int raw_a = 0;
    int raw_b = 0;
    if (!readPinDirect(pin_a_, raw_a) || !readPinDirect(pin_b_, raw_b)) {
        initialized = false;
        return 0;
    }
    cpu_mhz_ = static_cast<uint32_t>(getCpuFrequencyMhz());
    if (cpu_mhz_ == 0U) {
        cpu_mhz_ = 1U;
    }
    last_raw_a_ = raw_a;
    last_raw_b_ = raw_b;
    consecutive_rejects_ = 0;
    consecutive_stale_reads_ = 0;

    if (!configurePwmSynchronizedSampling()) {
        initialized = false;
        return 0;
    }
    initialized = true;
    return 1;
}

int SlaveAdc1CurrentSense::driverAlign(float align_voltage, bool modulation_centered) {
    return CurrentSense::driverAlign(align_voltage, modulation_centered);
}

PhaseCurrent_s SlaveAdc1CurrentSense::getPhaseCurrents() {
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t read_start_us = micros();
#endif
    SlaveCurrentSenseRawPair pair = {};
    const bool pair_valid = snapshotRawPair(pair) &&
                            pair.age_us <= kSlaveCurrentSensePairStaleUs;
    if (!pair_valid) {
        if (stale_count_ != UINT32_MAX) {
            stale_count_ = stale_count_ + 1U;
        }
        if (consecutive_stale_reads_ != UINT16_MAX) {
            consecutive_stale_reads_ =
                static_cast<uint16_t>(consecutive_stale_reads_ + 1U);
        }
        if (consecutive_stale_reads_ >=
                kSlaveCurrentSenseAdcConsecutiveErrorLimit &&
            runtime_validation_armed_) {
            sampling_fault_latched_ = true;
        }
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
        if (recordSlaveTimingCurrentSenseUs) {
            recordSlaveTimingCurrentSenseUs(micros() - read_start_us);
        }
#endif
        return lastPhaseCurrents();
    }
    consecutive_stale_reads_ = 0;
    const float voltage_a =
        static_cast<float>(pair.raw_a) * raw_to_voltage_v_;
    const float voltage_b =
        static_cast<float>(pair.raw_b) * raw_to_voltage_v_;

    PhaseCurrent_s current;
    current.a = (voltage_a - offset_ia) * gain_a;
    current.b = (voltage_b - offset_ib) * gain_b;
    current.c = 0.0f;
    publishLastSample(current);
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    if (recordSlaveTimingCurrentSenseUs) {
        recordSlaveTimingCurrentSenseUs(micros() - read_start_us);
    }
#endif
    return current;
}

int SlaveAdc1CurrentSense::readRawA() const {
    if (!initialized) {
        return 0;
    }
    SlaveCurrentSenseRawPair pair = {};
    return snapshotRawPair(pair) ? pair.raw_a : last_raw_a_;
}

int SlaveAdc1CurrentSense::readRawB() const {
    if (!initialized) {
        return 0;
    }
    SlaveCurrentSenseRawPair pair = {};
    return snapshotRawPair(pair) ? pair.raw_b : last_raw_b_;
}

int SlaveAdc1CurrentSense::lastRawA() const {
    return last_raw_a_;
}

int SlaveAdc1CurrentSense::lastRawB() const {
    return last_raw_b_;
}

uint8_t SlaveAdc1CurrentSense::channelA() const {
    return chan_a_;
}

uint8_t SlaveAdc1CurrentSense::channelB() const {
    return chan_b_;
}

const char *SlaveAdc1CurrentSense::adcBackendName() const {
    return "mcpwm_sync_fast";
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

uint32_t SlaveAdc1CurrentSense::readErrorCount() const {
    return read_error_count_;
}

uint16_t SlaveAdc1CurrentSense::consecutiveReadErrors() const {
    const uint16_t current =
        consecutive_rejects_ > consecutive_stale_reads_
            ? consecutive_rejects_
            : consecutive_stale_reads_;
    return sampling_fault_latched_ &&
                   current < kSlaveCurrentSenseAdcConsecutiveErrorLimit
               ? kSlaveCurrentSenseAdcConsecutiveErrorLimit
               : current;
}

bool SlaveAdc1CurrentSense::readFaulted() const {
    return !sync_ready_ ||
           (runtime_validation_armed_ && sampling_fault_latched_);
}

bool SlaveAdc1CurrentSense::snapshotRawPair(
    SlaveCurrentSenseRawPair &pair) const {
    uint32_t before = 0U;
    uint32_t after = 0U;
    uint32_t sequence = 0U;
    uint32_t publish_cycles = 0U;
    uint32_t skew_cycles = 0U;
    int raw_a = 0;
    int raw_b = 0;
    do {
        before = raw_pair_lock_;
        if ((before & 1U) != 0U) {
            continue;
        }
        __sync_synchronize();
        sequence = raw_pair_sequence_;
        raw_a = last_raw_a_;
        raw_b = last_raw_b_;
        publish_cycles = raw_pair_publish_cycles_;
        skew_cycles = raw_pair_skew_cycles_;
        __sync_synchronize();
        after = raw_pair_lock_;
    } while (before != after || (after & 1U) != 0U);

    pair.valid = sequence != 0U;
    pair.sequence = sequence;
    pair.raw_a = raw_a;
    pair.raw_b = raw_b;
    pair.age_us = pair.valid
                      ? cyclesToUs(esp_cpu_get_cycle_count() - publish_cycles)
                      : UINT32_MAX;
    pair.skew_us = pair.valid ? cyclesToUs(skew_cycles) : 0U;
    return pair.valid;
}

SlaveCurrentSenseSyncStats SlaveAdc1CurrentSense::syncStats() const {
    SlaveCurrentSenseSyncStats stats = {};
    SlaveCurrentSenseRawPair pair = {};
    (void)snapshotRawPair(pair);
    stats.sync_ready = sync_ready_;
    stats.pwm_event_hz = pwm_event_hz_;
    stats.adc_conversion_hz = measured_conversion_hz_;
    stats.phase_sample_hz = measured_phase_sample_hz_;
    stats.pair_sequence = pair.sequence;
    stats.pair_age_us = pair.age_us;
    stats.pair_skew_us = pair.skew_us;
    stats.stale_count = stale_count_;
    stats.rail_count = rail_count_;
    stats.reject_count = reject_count_;
    stats.adc_read_errors = read_error_count_;
    stats.consecutive_errors = consecutiveReadErrors();
    stats.calibration = calibration_stats_;
    return stats;
}

void SlaveAdc1CurrentSense::setCalibrationStats(
    const SlaveCurrentSenseCalibrationStats &stats) {
    calibration_stats_ = stats;
}

void SlaveAdc1CurrentSense::armRuntimeValidation() {
    runtime_validation_armed_ = false;
    sampling_enabled_ = false;
    delayMicroseconds(100);

    stale_count_ = 0U;
    rail_count_ = 0U;
    reject_count_ = 0U;
    read_error_count_ = 0U;
    consecutive_rejects_ = 0U;
    consecutive_stale_reads_ = 0U;
    sampling_fault_latched_ = false;
    phase_accumulator_ = 0U;
    sample_phase_a_ = true;
    pending_a_valid_ = false;

    sampling_enabled_ = true;
    runtime_validation_armed_ = true;
}
