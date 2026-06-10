#pragma once

#include <stdint.h>

#include "common/math/axis_math.h"

struct SlaveXMotorStepTiming {
    uint32_t loop_foc_us;
    uint32_t move_us;
    uint32_t sensor_us;
    uint32_t loop_foc_ran;
};

struct SlaveMotorCurrentAxisSnapshot {
    bool motor_ready;
    bool current_sense_ready;
    int raw_adc_a;
    int raw_adc_b;
    float raw_adc_a_v;
    float raw_adc_b_v;
    bool sync_ready;
    uint32_t pwm_event_hz;
    uint32_t adc_conversion_hz;
    uint32_t phase_sample_hz;
    uint32_t pair_sequence;
    uint32_t pair_age_us;
    uint32_t pair_skew_us;
    uint32_t adc_read_errors;
    uint16_t adc_consecutive_errors;
    uint32_t adc_stale_count;
    uint32_t adc_rail_count;
    uint32_t adc_reject_count;
    bool calibration_valid;
    uint32_t calibration_samples;
    float calibration_mean_a_raw;
    float calibration_mean_b_raw;
    float calibration_stddev_a_raw;
    float calibration_stddev_b_raw;
    int calibration_min_a_raw;
    int calibration_max_a_raw;
    int calibration_min_b_raw;
    int calibration_max_b_raw;
    float offset_ia_v;
    float offset_ib_v;
    float current_q_a;
    float current_d_a;
    float voltage_q_v;
    float voltage_d_v;
};

struct SlaveMotorCurrentSnapshot {
    SlaveMotorCurrentAxisSnapshot x;
    SlaveMotorCurrentAxisSnapshot y;
};

enum SlaveMotorTuningMode : uint8_t {
    SLAVE_MOTOR_TUNING_CURRENT_Q = 1,
    SLAVE_MOTOR_TUNING_VELOCITY = 2,
    SLAVE_MOTOR_TUNING_ANGLE = 3,
};

struct SlaveMotorTuningFeedback {
    bool ready;
    float shaft_angle_rad;
    float shaft_velocity_rad_s;
    float current_q_a;
    float current_d_a;
    float voltage_q_v;
    float voltage_d_v;
    float current_setpoint_a;
    float velocity_setpoint_rad_s;
    float angle_setpoint_rad;
    float raw_adc_a_v;
    float raw_adc_b_v;
};

// 设置紫光 MOS 输出；只允许安全任务调用。
void setUvOutput(bool enabled);

// 上电第一步调用：关闭 UV 和电机使能，建立失效保护默认状态。
void configureSlaveSafeOutputs();

// 传感器与电机初始化由 SLAVE_RUN_MODE 选择调用路径。
bool setupSlaveXSensorHardware();
bool setupSlaveXMotorHardware();
bool setupSlaveYSensorHardware();
bool setupSlaveYMotorOpenLoopHardware();
bool setupSlaveYMotorClosedLoopHardware();

void captureSlaveCurrentSenseRadioBaseline();
void logSlaveCurrentSenseRadioFreezeProbe();
bool finalizeSlaveCurrentSenseRuntimeValidation();

bool sampleSlaveYSensorForStatus(float *angle_rad, uint16_t *raw_angle);
SlaveMotorCurrentSnapshot snapshotSlaveMotorCurrent();
bool configureSlaveMotorTuning(AxisId axis,
                               uint8_t mode,
                               float p,
                               float i,
                               float d,
                               float current_limit_a,
                               float voltage_limit_v,
                               float velocity_limit_rad_s);
void restoreSlaveMotorTuning(AxisId axis);
SlaveMotorTuningFeedback snapshotSlaveMotorTuning(AxisId axis);

float applySlaveXMotorTarget(float target_angle_rad,
                             float fallback_actual_angle_rad,
                             SlaveXMotorStepTiming *timing);

float runSlaveXMotorPerfIsolationStep(float target_angle_rad,
                                      float fallback_actual_angle_rad,
                                      SlaveXMotorStepTiming *timing);

float applySlaveYMotorTarget(float target_angle_rad,
                             float fallback_actual_angle_rad,
                             SlaveXMotorStepTiming *timing);

float applySlaveXMotorTarget(float target_angle_rad, float fallback_actual_angle_rad);
float runSlaveXMotorPerfIsolationStep(float target_angle_rad, float fallback_actual_angle_rad);
float applySlaveYMotorTarget(float target_angle_rad, float fallback_actual_angle_rad);
