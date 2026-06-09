#include "slave/diagnostics/current_probe.h"

#include <Arduino.h>

#include "common/math/angle_math.h"
#include "common/math/clamp.h"
#include "slave/config/slave_config.h"

#if (SLAVE_X_MOTOR_HW_ENABLED || SLAVE_Y_MOTOR_HW_ENABLED) && SLAVE_ENABLE_CURRENT_SENSE
namespace {

constexpr int kSlaveCurrentSenseOffsetRailMarginRaw = 32;

#if SLAVE_ENABLE_CURRENT_SENSE_DIAG_TEST
// 直接读取 ADC 原始电压，不经过 offset/gain，便于观察硬件采样基线。
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
    context.current_sense.refreshFastScale();

    const PhaseCurrent_s sample = context.current_sense.getPhaseCurrents();

    context.current_sense.offset_ia = saved_offset_ia;
    context.current_sense.offset_ib = saved_offset_ib;
    context.current_sense.offset_ic = saved_offset_ic;
    context.current_sense.gain_a = saved_gain_a;
    context.current_sense.gain_b = saved_gain_b;
    context.current_sense.gain_c = saved_gain_c;
    context.current_sense.refreshFastScale();

    return sample;
}
#endif

// 预读 ADC，让采样链路进入稳定状态后再做 offset 校准。
void primeSlaveCurrentSenseAdc(SlaveMotorDiagnosticsContext &context) {
    for (uint16_t i = 0; i < kSlaveCurrentSenseDiag.adc_prime_reads; ++i) {
        (void)context.current_sense.readRawA();
        (void)context.current_sense.readRawB();
    }
}

bool isSlaveCurrentSenseOffsetRawValid(int raw) {
    const int max_raw = static_cast<int>(kSlaveCurrentSenseHardware.adc_raw_max);
    return raw > kSlaveCurrentSenseOffsetRailMarginRaw &&
           raw < (max_raw - kSlaveCurrentSenseOffsetRailMarginRaw);
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
    const int raw_adc_a = context.current_sense.readRawA();
    const int raw_adc_b = context.current_sense.readRawB();
    const int raw_unmasked_a = context.current_sense.readRawUnmaskedA();
    const int raw_unmasked_b = context.current_sense.readRawUnmaskedB();
    const float delta_mv_a = (voltage.a - context.current_sense.offset_ia) * 1000.0f;
    const float delta_mv_b = (voltage.b - context.current_sense.offset_ib) * 1000.0f;
    context.sensor.update();
    Serial.printf("[Slave] current_probe axis=%s %s %s pwm=%.2f,%.2f,%.2f va=%.4fV vb=%.4fV dmv_a=%.1f dmv_b=%.1f ia=%.4fA ib=%.4fA raw_adc=%d,%d raw16=%d,%d raw=%u angle=%.2fdeg\n",
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
                  raw_adc_a,
                  raw_adc_b,
                  raw_unmasked_a,
                  raw_unmasked_b,
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

// 电机不输出时采样平均值作为 offset，后续电流 = (电压 - offset) * gain。
bool calibrateSlaveCurrentSenseOffsets(SlaveMotorDiagnosticsContext &context) {
#if (SLAVE_X_MOTOR_HW_ENABLED || SLAVE_Y_MOTOR_HW_ENABLED) && SLAVE_ENABLE_CURRENT_SENSE
    const float saved_gain_a = context.current_sense.gain_a;
    const float saved_gain_b = context.current_sense.gain_b;
    const float saved_gain_c = context.current_sense.gain_c;
    const float saved_offset_ic = context.current_sense.offset_ic;
    const uint32_t calibration_reads =
        (kSlaveCurrentSenseDiag.offset_reads > 0)
            ? kSlaveCurrentSenseDiag.offset_reads
            : 1UL;

    // DengFoc 在 EN/PWM 活动后 ADC 零点会漂到运行态偏置，offset 必须在同状态下测。
    context.driver.enable();
    context.driver.setPwm(0.0f, 0.0f, 0.0f);
    delay(kSlaveCurrentSenseDiag.offset_settle_ms);
    primeSlaveCurrentSenseAdc(context);

    context.current_sense.offset_ia = 0.0f;
    context.current_sense.offset_ib = 0.0f;
    context.current_sense.offset_ic = 0.0f;
    context.current_sense.gain_a = 1.0f;
    context.current_sense.gain_b = 1.0f;
    context.current_sense.gain_c = 1.0f;
    context.current_sense.refreshFastScale();

    float sum_a_v = 0.0f;
    float sum_b_v = 0.0f;
    uint32_t valid_reads = 0;
    uint32_t saturated_reads = 0;
    for (uint32_t i = 0; i < calibration_reads; ++i) {
        const int raw_a = context.current_sense.readRawA();
        const int raw_b = context.current_sense.readRawB();
        if (!isSlaveCurrentSenseOffsetRawValid(raw_a) ||
            !isSlaveCurrentSenseOffsetRawValid(raw_b)) {
            saturated_reads++;
            delay(1);
            continue;
        }
        sum_a_v += static_cast<float>(raw_a) * kSlaveCurrentSenseHardware.adc_raw_to_voltage_v;
        sum_b_v += static_cast<float>(raw_b) * kSlaveCurrentSenseHardware.adc_raw_to_voltage_v;
        valid_reads++;
        delay(1);
    }

    if (valid_reads == 0) {
        context.current_sense.offset_ia = 0.0f;
        context.current_sense.offset_ib = 0.0f;
        context.current_sense.offset_ic = saved_offset_ic;
        context.current_sense.gain_a = saved_gain_a;
        context.current_sense.gain_b = saved_gain_b;
        context.current_sense.gain_c = saved_gain_c;
        context.current_sense.refreshFastScale();
        const int runtime_raw_adc_a = context.current_sense.readRawA();
        const int runtime_raw_adc_b = context.current_sense.readRawB();
        const int runtime_raw_unmasked_a = context.current_sense.readRawUnmaskedA();
        const int runtime_raw_unmasked_b = context.current_sense.readRawUnmaskedB();
        context.driver.setPwm(0.0f, 0.0f, 0.0f);
        context.driver.disable();
#if SLAVE_VOFA_TUNER_ENABLED
        Serial.printf("# [Slave] motor_diag axis=%s current_sense offset_cal failed saturated valid=%lu skipped=%lu raw_adc=%d,%d raw16=%d,%d\n",
#else
        Serial.printf("[Slave] motor_diag axis=%s current_sense offset_cal failed saturated valid=%lu skipped=%lu raw_adc=%d,%d raw16=%d,%d\n",
#endif
                      context.axis_name,
                      static_cast<unsigned long>(valid_reads),
                      static_cast<unsigned long>(saturated_reads),
                      runtime_raw_adc_a,
                      runtime_raw_adc_b,
                      runtime_raw_unmasked_a,
                      runtime_raw_unmasked_b);
        return false;
    }

    context.current_sense.offset_ia = sum_a_v / static_cast<float>(valid_reads);
    context.current_sense.offset_ib = sum_b_v / static_cast<float>(valid_reads);
    context.current_sense.offset_ic = saved_offset_ic;
    context.current_sense.gain_a = saved_gain_a;
    context.current_sense.gain_b = saved_gain_b;
    context.current_sense.gain_c = saved_gain_c;
    context.current_sense.refreshFastScale();

    const int runtime_raw_adc_a = context.current_sense.readRawA();
    const int runtime_raw_adc_b = context.current_sense.readRawB();
    const int runtime_raw_unmasked_a = context.current_sense.readRawUnmaskedA();
    const int runtime_raw_unmasked_b = context.current_sense.readRawUnmaskedB();
    context.driver.setPwm(0.0f, 0.0f, 0.0f);
    context.driver.disable();

#if SLAVE_VOFA_TUNER_ENABLED
    Serial.printf("# [Slave] motor_diag axis=%s current_sense offset_cal mode=driver_enabled_pwm0 settle=%ums samples=%lu valid=%lu skipped=%lu ia=%.4fV ib=%.4fV raw_adc=%d,%d raw16=%d,%d\n",
#else
    Serial.printf("[Slave] motor_diag axis=%s current_sense offset_cal mode=driver_enabled_pwm0 settle=%ums samples=%lu valid=%lu skipped=%lu ia=%.4fV ib=%.4fV raw_adc=%d,%d raw16=%d,%d\n",
#endif
                  context.axis_name,
                  static_cast<unsigned int>(kSlaveCurrentSenseDiag.offset_settle_ms),
                  static_cast<unsigned long>(calibration_reads),
                  static_cast<unsigned long>(valid_reads),
                  static_cast<unsigned long>(saturated_reads),
                  context.current_sense.offset_ia,
                  context.current_sense.offset_ib,
                  runtime_raw_adc_a,
                  runtime_raw_adc_b,
                  runtime_raw_unmasked_a,
                  runtime_raw_unmasked_b);
    return true;
#else
    (void)context;
    return true;
#endif
}

// 分别注入 U/V/W 相，观察 ia/ib 符号和幅值，用于确认采样通道与方向。
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
