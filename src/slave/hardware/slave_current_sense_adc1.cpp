#include "slave/hardware/slave_current_sense_adc1.h"

#include "slave/config/slave_config.h"

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

void SlaveAdc1CurrentSense::publishLastSample(int raw_a,
                                              int raw_b,
                                              const PhaseCurrent_s &current) {
    sample_seq_ = sample_seq_ + 1U;
    last_raw_a_ = raw_a;
    last_raw_b_ = raw_b;
    last_current_a_ = current.a;
    last_current_b_ = current.b;
    last_current_c_ = current.c;
    sample_seq_ = sample_seq_ + 1U;
}

int SlaveAdc1CurrentSense::init() {
    if (has_phase_c_) {
        initialized = false;
        return 0;
    }
    if (!slaveAdc1DmaSlotForPin(pin_a_, slot_a_) ||
        !slaveAdc1DmaSlotForPin(pin_b_, slot_b_) ||
        !slaveAdc1DmaSamplerRequired()) {
        initialized = false;
        return 0;
    }

    int raw_a = 0;
    int raw_b = 0;
    if (!slaveAdc1DmaReadLatestRaw(slot_a_, raw_a) ||
        !slaveAdc1DmaReadLatestRaw(slot_b_, raw_b)) {
        initialized = false;
        return 0;
    }

    last_raw_a_ = raw_a;
    last_raw_b_ = raw_b;
    initialized = true;
    return 1;
}

int SlaveAdc1CurrentSense::driverAlign(float align_voltage, bool modulation_centered) {
    (void)align_voltage;
    (void)modulation_centered;
    return 1;
}

PhaseCurrent_s SlaveAdc1CurrentSense::getPhaseCurrents() {
    int raw_a = last_raw_a_;
    int raw_b = last_raw_b_;
    if (!slaveAdc1DmaReadControlRaw(slot_a_, raw_a)) {
        (void)slaveAdc1DmaReadLatestRaw(slot_a_, raw_a);
    }
    if (!slaveAdc1DmaReadControlRaw(slot_b_, raw_b)) {
        (void)slaveAdc1DmaReadLatestRaw(slot_b_, raw_b);
    }

    const float voltage_a = static_cast<float>(raw_a) * raw_to_voltage_v_;
    const float voltage_b = static_cast<float>(raw_b) * raw_to_voltage_v_;

    PhaseCurrent_s current;
    current.a = (voltage_a - offset_ia) * gain_a;
    current.b = (voltage_b - offset_ib) * gain_b;
    current.c = 0.0f;
    publishLastSample(raw_a, raw_b, current);
    return current;
}

int SlaveAdc1CurrentSense::readRawA() const {
    if (!initialized) {
        return 0;
    }
    int raw = last_raw_a_;
    (void)slaveAdc1DmaReadLatestRaw(slot_a_, raw);
    return raw;
}

int SlaveAdc1CurrentSense::readRawB() const {
    if (!initialized) {
        return 0;
    }
    int raw = last_raw_b_;
    (void)slaveAdc1DmaReadLatestRaw(slot_b_, raw);
    return raw;
}

bool SlaveAdc1CurrentSense::waitNextRawPair(uint32_t &last_sequence,
                                            uint32_t timeout_ms,
                                            int &raw_a,
                                            int &raw_b) const {
    if (!initialized) {
        return false;
    }
    return waitForSlaveAdc1DmaRawPair(slot_a_,
                                      slot_b_,
                                      last_sequence,
                                      timeout_ms,
                                      raw_a,
                                      raw_b);
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

uint32_t SlaveAdc1CurrentSense::readErrorCount() const {
    const SlaveAdc1DmaHealthSnapshot health = snapshotSlaveAdc1DmaHealth();
    return health.invalid_frames +
           health.read_errors +
           health.pool_overflows;
}

uint16_t SlaveAdc1CurrentSense::consecutiveReadErrors() const {
    const uint32_t stale = snapshotSlaveAdc1DmaHealth().stale_control_cycles;
    return stale > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(stale);
}

bool SlaveAdc1CurrentSense::readFaulted() const {
    return slaveAdc1DmaFaultLatched();
}
