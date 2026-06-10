#include "slave/diagnostics/current_probe.h"

#include <Arduino.h>
#include <math.h>

#include "common/math/angle_math.h"
#include "common/math/clamp.h"
#include "slave/config/slave_config.h"

#if (SLAVE_X_MOTOR_HW_ENABLED || SLAVE_Y_MOTOR_HW_ENABLED) && SLAVE_ENABLE_CURRENT_SENSE
namespace {

#if SLAVE_ENABLE_CURRENT_SENSE_DIAG_TEST
PhaseCurrent_s sampleSlaveCurrentSenseVoltagesOnce(SlaveMotorDiagnosticsContext &context) {
    const float saved_offset_ia = context.current_sense.offset_ia;
    const float saved_offset_ib = context.current_sense.offset_ib;
    const float saved_offset_ic = context.current_sense.offset_ic;
    const float saved_gain_a = context.current_sense.gain_a;
    const float saved_gain_b = context.current_sense.gain_b;
    const float saved_gain_c = context.current_sense.gain_c;

    context.current_sense.offset_ia = 0.0f;
    context.current_sense.offset_ib = 0.0f;
    context.current_sense.offset_ic = 0.0f;
    context.current_sense.gain_a = 1.0f;
    context.current_sense.gain_b = 1.0f;
    context.current_sense.gain_c = 1.0f;

    const PhaseCurrent_s sample = context.current_sense.getPhaseCurrents();

    context.current_sense.offset_ia = saved_offset_ia;
    context.current_sense.offset_ib = saved_offset_ib;
    context.current_sense.offset_ic = saved_offset_ic;
    context.current_sense.gain_a = saved_gain_a;
    context.current_sense.gain_b = saved_gain_b;
    context.current_sense.gain_c = saved_gain_c;

    return sample;
}
#endif

struct SlaveRawStatsAccumulator {
    uint32_t count;
    double mean;
    double m2;
    int min_raw;
    int max_raw;
};

void addSlaveRawSample(SlaveRawStatsAccumulator &stats, int raw) {
    stats.count++;
    const double delta = static_cast<double>(raw) - stats.mean;
    stats.mean += delta / static_cast<double>(stats.count);
    const double delta2 = static_cast<double>(raw) - stats.mean;
    stats.m2 += delta * delta2;
    if (raw < stats.min_raw) {
        stats.min_raw = raw;
    }
    if (raw > stats.max_raw) {
        stats.max_raw = raw;
    }
}

float slaveRawStddev(const SlaveRawStatsAccumulator &stats) {
    if (stats.count < 2U) {
        return 0.0f;
    }
    return static_cast<float>(
        sqrt(stats.m2 / static_cast<double>(stats.count - 1U)));
}

bool waitForNextSlaveCurrentSensePair(SlaveAdc1CurrentSense &current_sense,
                                      uint32_t previous_sequence,
                                      SlaveCurrentSenseRawPair &pair) {
    const uint32_t start_us = micros();
    const uint32_t timeout_us = kSlaveCurrentSensePairStaleUs * 4UL;
    do {
        if (current_sense.snapshotRawPair(pair) &&
            pair.sequence != previous_sequence &&
            pair.age_us <= kSlaveCurrentSensePairStaleUs) {
            return true;
        }
        delayMicroseconds(20);
    } while (static_cast<uint32_t>(micros() - start_us) < timeout_us);
    return false;
}

bool primeSlaveCurrentSenseAdc(SlaveMotorDiagnosticsContext &context) {
    SlaveCurrentSenseRawPair pair = {};
    (void)context.current_sense.snapshotRawPair(pair);
    uint32_t sequence = pair.sequence;
    for (uint16_t i = 0; i < kSlaveCurrentSenseDiag.adc_prime_reads; ++i) {
        if (!waitForNextSlaveCurrentSensePair(context.current_sense,
                                              sequence,
                                              pair)) {
            return false;
        }
        sequence = pair.sequence;
    }
    return true;
}

#if SLAVE_ENABLE_CURRENT_SENSE_DIAG_TEST
void logSlaveCurrentProbeSample(SlaveMotorDiagnosticsContext &context,
                                const char *label,
                                const char *stage,
                                float phase_u_v,
                                float phase_v_v,
                                float phase_w_v) {
    const PhaseCurrent_s current = context.current_sense.getPhaseCurrents();
    const PhaseCurrent_s voltage = sampleSlaveCurrentSenseVoltagesOnce(context);
    SlaveCurrentSenseRawPair raw_pair = {};
    (void)context.current_sense.snapshotRawPair(raw_pair);
    const float delta_mv_a = (voltage.a - context.current_sense.offset_ia) * 1000.0f;
    const float delta_mv_b = (voltage.b - context.current_sense.offset_ib) * 1000.0f;
    context.sensor.update();
    Serial.printf("[Slave] current_probe axis=%s %s %s pwm=%.2f,%.2f,%.2f va=%.4fV vb=%.4fV dmv_a=%.1f dmv_b=%.1f ia=%.4fA ib=%.4fA raw_adc=%d,%d adc_errors=%lu raw=%u angle=%.2fdeg\n",
                  context.axis_name,
                  label,
                  stage,
                  phase_u_v,
                  phase_v_v,
                  phase_w_v,
                  voltage.a,
                  voltage.b,
                  delta_mv_a,
                  delta_mv_b,
                  current.a,
                  current.b,
                  raw_pair.raw_a,
                  raw_pair.raw_b,
                  static_cast<unsigned long>(context.current_sense.readErrorCount()),
                  static_cast<unsigned int>(context.sensor.rawAngle()),
                  radToDeg(context.sensor.getMechanicalAngle()));
}

void runSlaveCurrentProbePoint(SlaveMotorDiagnosticsContext &context,
                               const char *label,
                               float phase_u_v,
                               float phase_v_v,
                               float phase_w_v) {
    context.driver.setPwm(phase_u_v, phase_v_v, phase_w_v);
    delay(kSlaveCurrentSenseDiag.early_ms);
    logSlaveCurrentProbeSample(context, label, "early", phase_u_v, phase_v_v, phase_w_v);
    const uint32_t remaining_settle_ms =
        (kSlaveCurrentSenseDiag.settle_ms > kSlaveCurrentSenseDiag.early_ms)
            ? (kSlaveCurrentSenseDiag.settle_ms - kSlaveCurrentSenseDiag.early_ms)
            : 0;
    if (remaining_settle_ms > 0) {
        delay(remaining_settle_ms);
    }
    logSlaveCurrentProbeSample(context, label, "settled", phase_u_v, phase_v_v, phase_w_v);
}
#endif

}  // namespace
#endif

bool calibrateSlaveCurrentSenseOffsets(SlaveMotorDiagnosticsContext &context) {
#if (SLAVE_X_MOTOR_HW_ENABLED || SLAVE_Y_MOTOR_HW_ENABLED) && SLAVE_ENABLE_CURRENT_SENSE
    const float saved_offset_ia = context.current_sense.offset_ia;
    const float saved_offset_ib = context.current_sense.offset_ib;
    const float saved_gain_a = context.current_sense.gain_a;
    const float saved_gain_b = context.current_sense.gain_b;
    const float saved_gain_c = context.current_sense.gain_c;
    const float saved_offset_ic = context.current_sense.offset_ic;
    const float saved_driver_voltage_limit = context.driver.voltage_limit;
    const uint32_t calibration_reads =
        (kSlaveCurrentSenseDiag.offset_reads > 0)
            ? kSlaveCurrentSenseDiag.offset_reads
            : 1UL;
    const float runtime_voltage_limit =
        context.motor_config.voltage.motor_limit_v;
    const float centered_zero_voltage = runtime_voltage_limit * 0.5f;
    const SlaveCurrentSenseSyncStats sync_before =
        context.current_sense.syncStats();

    context.driver.voltage_limit = runtime_voltage_limit;
    context.driver.enable();
    context.driver.setPwm(centered_zero_voltage,
                          centered_zero_voltage,
                          centered_zero_voltage);
    delay(kSlaveCurrentSenseDiag.offset_settle_ms);
    const bool prime_ok = primeSlaveCurrentSenseAdc(context);

    SlaveRawStatsAccumulator stats_a = {
        0U,
        0.0,
        0.0,
        static_cast<int>(kSlaveCurrentSenseHardware.adc_raw_max),
        0,
    };
    SlaveRawStatsAccumulator stats_b = stats_a;
    SlaveCurrentSenseRawPair pair = {};
    (void)context.current_sense.snapshotRawPair(pair);
    uint32_t pair_sequence = pair.sequence;
    uint32_t valid_reads = 0;
    while (prime_ok && valid_reads < calibration_reads) {
        if (!waitForNextSlaveCurrentSensePair(context.current_sense,
                                              pair_sequence,
                                              pair)) {
            break;
        }
        pair_sequence = pair.sequence;
        addSlaveRawSample(stats_a, pair.raw_a);
        addSlaveRawSample(stats_b, pair.raw_b);
        valid_reads++;
    }

    context.driver.setPwm(0.0f, 0.0f, 0.0f);
    context.driver.disable();
    context.driver.voltage_limit = saved_driver_voltage_limit;

    SlaveCurrentSenseCalibrationStats calibration = {};
    calibration.samples = valid_reads;
    calibration.mean_a_raw = static_cast<float>(stats_a.mean);
    calibration.mean_b_raw = static_cast<float>(stats_b.mean);
    calibration.stddev_a_raw = slaveRawStddev(stats_a);
    calibration.stddev_b_raw = slaveRawStddev(stats_b);
    calibration.min_a_raw = valid_reads > 0U ? stats_a.min_raw : 0;
    calibration.max_a_raw = valid_reads > 0U ? stats_a.max_raw : 0;
    calibration.min_b_raw = valid_reads > 0U ? stats_b.min_raw : 0;
    calibration.max_b_raw = valid_reads > 0U ? stats_b.max_raw : 0;
    const int peak_to_peak_a =
        calibration.max_a_raw - calibration.min_a_raw;
    const int peak_to_peak_b =
        calibration.max_b_raw - calibration.min_b_raw;
    const float min_mean_raw =
        kSlaveCurrentSenseHardware.adc_raw_max *
        kSlaveCurrentSenseCalibrationMeanMinRatio;
    const float max_mean_raw =
        kSlaveCurrentSenseHardware.adc_raw_max *
        kSlaveCurrentSenseCalibrationMeanMaxRatio;
    const SlaveCurrentSenseSyncStats sync_after =
        context.current_sense.syncStats();
    calibration.valid =
        prime_ok &&
        valid_reads == calibration_reads &&
        calibration.mean_a_raw >= min_mean_raw &&
        calibration.mean_a_raw <= max_mean_raw &&
        calibration.mean_b_raw >= min_mean_raw &&
        calibration.mean_b_raw <= max_mean_raw &&
        calibration.stddev_a_raw <=
            kSlaveCurrentSenseCalibrationStddevMaxRaw &&
        calibration.stddev_b_raw <=
            kSlaveCurrentSenseCalibrationStddevMaxRaw &&
        peak_to_peak_a <= kSlaveCurrentSenseCalibrationPeakToPeakMaxRaw &&
        peak_to_peak_b <= kSlaveCurrentSenseCalibrationPeakToPeakMaxRaw &&
        sync_after.adc_read_errors == sync_before.adc_read_errors &&
        sync_after.rail_count == sync_before.rail_count &&
        sync_after.reject_count == sync_before.reject_count;
    context.current_sense.setCalibrationStats(calibration);

    if (!calibration.valid) {
        context.current_sense.offset_ia = saved_offset_ia;
        context.current_sense.offset_ib = saved_offset_ib;
        context.current_sense.offset_ic = saved_offset_ic;
        context.current_sense.gain_a = saved_gain_a;
        context.current_sense.gain_b = saved_gain_b;
        context.current_sense.gain_c = saved_gain_c;
#if SLAVE_VOFA_TUNER_ENABLED
        Serial.printf("# [Slave] motor_diag axis=%s current_sense offset_cal failed mode=centered_zero vlim=%.3fV center=%.3fV samples=%lu/%lu mean=%.1f,%.1f std=%.1f,%.1f minmax=%d:%d,%d:%d p2p=%d,%d stale=%lu rail=%lu reject=%lu adc_errors=%lu\n",
#else
        Serial.printf("[Slave] motor_diag axis=%s current_sense offset_cal failed mode=centered_zero vlim=%.3fV center=%.3fV samples=%lu/%lu mean=%.1f,%.1f std=%.1f,%.1f minmax=%d:%d,%d:%d p2p=%d,%d stale=%lu rail=%lu reject=%lu adc_errors=%lu\n",
#endif
                      context.axis_name,
                      runtime_voltage_limit,
                      centered_zero_voltage,
                      static_cast<unsigned long>(valid_reads),
                      static_cast<unsigned long>(calibration_reads),
                      calibration.mean_a_raw,
                      calibration.mean_b_raw,
                      calibration.stddev_a_raw,
                      calibration.stddev_b_raw,
                      calibration.min_a_raw,
                      calibration.max_a_raw,
                      calibration.min_b_raw,
                      calibration.max_b_raw,
                      peak_to_peak_a,
                      peak_to_peak_b,
                      static_cast<unsigned long>(sync_after.stale_count),
                      static_cast<unsigned long>(sync_after.rail_count),
                      static_cast<unsigned long>(sync_after.reject_count),
                      static_cast<unsigned long>(sync_after.adc_read_errors));
        return false;
    }

    context.current_sense.offset_ia =
        calibration.mean_a_raw *
        kSlaveCurrentSenseHardware.adc_raw_to_voltage_v;
    context.current_sense.offset_ib =
        calibration.mean_b_raw *
        kSlaveCurrentSenseHardware.adc_raw_to_voltage_v;
    context.current_sense.offset_ic = saved_offset_ic;
    context.current_sense.gain_a = saved_gain_a;
    context.current_sense.gain_b = saved_gain_b;
    context.current_sense.gain_c = saved_gain_c;

#if SLAVE_VOFA_TUNER_ENABLED
    Serial.printf("# [Slave] motor_diag axis=%s current_sense offset_cal mode=centered_zero vlim=%.3fV center=%.3fV settle=%ums samples=%lu mean=%.1f,%.1f std=%.1f,%.1f minmax=%d:%d,%d:%d p2p=%d,%d ia=%.4fV ib=%.4fV pwm=%luHz adc=%luHz phase=%luHz skew=%luus\n",
#else
    Serial.printf("[Slave] motor_diag axis=%s current_sense offset_cal mode=centered_zero vlim=%.3fV center=%.3fV settle=%ums samples=%lu mean=%.1f,%.1f std=%.1f,%.1f minmax=%d:%d,%d:%d p2p=%d,%d ia=%.4fV ib=%.4fV pwm=%luHz adc=%luHz phase=%luHz skew=%luus\n",
#endif
                  context.axis_name,
                  runtime_voltage_limit,
                  centered_zero_voltage,
                  static_cast<unsigned int>(kSlaveCurrentSenseDiag.offset_settle_ms),
                  static_cast<unsigned long>(valid_reads),
                  calibration.mean_a_raw,
                  calibration.mean_b_raw,
                  calibration.stddev_a_raw,
                  calibration.stddev_b_raw,
                  calibration.min_a_raw,
                  calibration.max_a_raw,
                  calibration.min_b_raw,
                  calibration.max_b_raw,
                  peak_to_peak_a,
                  peak_to_peak_b,
                  context.current_sense.offset_ia,
                  context.current_sense.offset_ib,
                  static_cast<unsigned long>(sync_after.pwm_event_hz),
                  static_cast<unsigned long>(sync_after.adc_conversion_hz),
                  static_cast<unsigned long>(sync_after.phase_sample_hz),
                  static_cast<unsigned long>(sync_after.pair_skew_us));
    return true;
#else
    (void)context;
    return true;
#endif
}

bool verifySlaveCurrentSenseRuntimeBaseline(
    SlaveMotorDiagnosticsContext &context,
    const char *stage,
    bool arm_runtime_validation) {
#if (SLAVE_X_MOTOR_HW_ENABLED || SLAVE_Y_MOTOR_HW_ENABLED) && SLAVE_ENABLE_CURRENT_SENSE
    const SlaveCurrentSenseSyncStats sync_before =
        context.current_sense.syncStats();
    const SlaveCurrentSenseCalibrationStats calibration =
        sync_before.calibration;
    if (!calibration.valid) {
        return false;
    }

    context.driver.enable();
    context.motor.setPhaseVoltage(0.0f,
                                  0.0f,
                                  context.motor.electrical_angle);
    delay(kSlaveCurrentSenseDiag.offset_settle_ms);
    const bool prime_ok = primeSlaveCurrentSenseAdc(context);

    SlaveRawStatsAccumulator stats_a = {
        0U,
        0.0,
        0.0,
        static_cast<int>(kSlaveCurrentSenseHardware.adc_raw_max),
        0,
    };
    SlaveRawStatsAccumulator stats_b = stats_a;
    SlaveCurrentSenseRawPair pair = {};
    (void)context.current_sense.snapshotRawPair(pair);
    uint32_t pair_sequence = pair.sequence;
    uint32_t valid_reads = 0U;
    const uint32_t verify_reads = kSlaveCurrentSenseDiag.offset_reads;
    while (prime_ok && valid_reads < verify_reads) {
        if (!waitForNextSlaveCurrentSensePair(context.current_sense,
                                              pair_sequence,
                                              pair)) {
            break;
        }
        pair_sequence = pair.sequence;
        addSlaveRawSample(stats_a, pair.raw_a);
        addSlaveRawSample(stats_b, pair.raw_b);
        valid_reads++;
    }

    const float mean_a = static_cast<float>(stats_a.mean);
    const float mean_b = static_cast<float>(stats_b.mean);
    const float stddev_a = slaveRawStddev(stats_a);
    const float stddev_b = slaveRawStddev(stats_b);
    const int min_a = valid_reads > 0U ? stats_a.min_raw : 0;
    const int max_a = valid_reads > 0U ? stats_a.max_raw : 0;
    const int min_b = valid_reads > 0U ? stats_b.min_raw : 0;
    const int max_b = valid_reads > 0U ? stats_b.max_raw : 0;
    const int peak_to_peak_a = max_a - min_a;
    const int peak_to_peak_b = max_b - min_b;
    const float delta_a = fabsf(mean_a - calibration.mean_a_raw);
    const float delta_b = fabsf(mean_b - calibration.mean_b_raw);
    const SlaveCurrentSenseSyncStats sync_after =
        context.current_sense.syncStats();
    const bool valid =
        prime_ok &&
        valid_reads == verify_reads &&
        delta_a <=
            static_cast<float>(kSlaveCurrentSenseRuntimeBaselineDeltaMaxRaw) &&
        delta_b <=
            static_cast<float>(kSlaveCurrentSenseRuntimeBaselineDeltaMaxRaw) &&
        stddev_a <= kSlaveCurrentSenseCalibrationStddevMaxRaw &&
        stddev_b <= kSlaveCurrentSenseCalibrationStddevMaxRaw &&
        peak_to_peak_a <= kSlaveCurrentSenseCalibrationPeakToPeakMaxRaw &&
        peak_to_peak_b <= kSlaveCurrentSenseCalibrationPeakToPeakMaxRaw &&
        sync_after.adc_read_errors == sync_before.adc_read_errors &&
        sync_after.rail_count == sync_before.rail_count &&
        sync_after.reject_count == sync_before.reject_count;

#if SLAVE_VOFA_TUNER_ENABLED
    Serial.printf("# [Slave] motor_diag axis=%s current_sense runtime_zero stage=%s valid=%u armed=%u samples=%lu mean=%.1f,%.1f delta=%.1f,%.1f std=%.1f,%.1f p2p=%d,%d startup_stale=%lu startup_rail=%lu startup_reject=%lu\n",
#else
    Serial.printf("[Slave] motor_diag axis=%s current_sense runtime_zero stage=%s valid=%u armed=%u samples=%lu mean=%.1f,%.1f delta=%.1f,%.1f std=%.1f,%.1f p2p=%d,%d startup_stale=%lu startup_rail=%lu startup_reject=%lu\n",
#endif
                  context.axis_name,
                  stage,
                  valid ? 1U : 0U,
                  (valid && arm_runtime_validation) ? 1U : 0U,
                  static_cast<unsigned long>(valid_reads),
                  mean_a,
                  mean_b,
                  delta_a,
                  delta_b,
                  stddev_a,
                  stddev_b,
                  peak_to_peak_a,
                  peak_to_peak_b,
                  static_cast<unsigned long>(sync_after.stale_count),
                  static_cast<unsigned long>(sync_after.rail_count),
                  static_cast<unsigned long>(sync_after.reject_count));

    if (!valid) {
        context.driver.setPwm(0.0f, 0.0f, 0.0f);
        context.driver.disable();
        return false;
    }

    if (arm_runtime_validation) {
        context.current_sense.armRuntimeValidation();
    }
    return true;
#else
    (void)context;
    (void)stage;
    (void)arm_runtime_validation;
    return true;
#endif
}

void runSlaveCurrentSenseProbe(SlaveMotorDiagnosticsContext &context) {
#if (SLAVE_X_MOTOR_HW_ENABLED || SLAVE_Y_MOTOR_HW_ENABLED) && \
    SLAVE_ENABLE_CURRENT_SENSE && SLAVE_ENABLE_CURRENT_SENSE_DIAG_TEST
    const float probe_voltage_v = clampFloat(kSlaveCurrentSenseDiag.voltage_v,
                                             0.0f,
                                             context.motor_config.voltage.motor_limit_v);
    Serial.printf("[Slave] current_probe axis=%s start voltage=%.2fV settle=%ums\n",
                  context.axis_name,
                  probe_voltage_v,
                  static_cast<unsigned int>(kSlaveCurrentSenseDiag.settle_ms));
    context.driver.enable();
    context.driver.setPwm(0.0f, 0.0f, 0.0f);
    delay(kSlaveCurrentSenseDiag.settle_ms);
    logSlaveCurrentProbeSample(context, "idle0", "settled", 0.0f, 0.0f, 0.0f);
    runSlaveCurrentProbePoint(context, "phase_u", probe_voltage_v, 0.0f, 0.0f);
    runSlaveCurrentProbePoint(context, "phase_v", 0.0f, probe_voltage_v, 0.0f);
    runSlaveCurrentProbePoint(context, "phase_w", 0.0f, 0.0f, probe_voltage_v);
    context.driver.setPwm(0.0f, 0.0f, 0.0f);
    delay(kSlaveCurrentSenseDiag.settle_ms);
    logSlaveCurrentProbeSample(context, "idle1", "settled", 0.0f, 0.0f, 0.0f);
    context.driver.disable();
#else
    (void)context;
#endif
}
