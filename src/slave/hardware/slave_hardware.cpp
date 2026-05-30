#include "slave/hardware/slave_hardware.h"

#include <Arduino.h>
#include <SPI.h>
#include <board/board_pins_slave.h>
#include <math.h>

#include "common/system_state.h"
#include "slave/config/slave_config.h"
#include "slave/hardware/slave_mt6701_sensor.h"
#include "slave/modes/mode_traits.h"

#if SLAVE_X_MOTOR_HW_ENABLED || SLAVE_Y_MOTOR_HW_ENABLED
#include <SimpleFOC.h>
#endif

namespace {

#if SLAVE_X_SENSOR_HW_ENABLED
bool xSensorReady = false;
SPIClass xSsiBus(FSPI);
SlaveMt6701Sensor xSensor = SlaveMt6701Sensor(board_pins_slave::ENCODER1_CS_X);
#endif

#if SLAVE_X_MOTOR_HW_ENABLED
bool xMotorReady = false;
BLDCMotor xMotor = BLDCMotor(SLAVE_X_MOTOR_POLE_PAIRS);
BLDCDriver3PWM xDriver = BLDCDriver3PWM(
    board_pins_slave::MOTOR1_PWM_U_X,
    board_pins_slave::MOTOR1_PWM_V_X,
    board_pins_slave::MOTOR1_PWM_W_X,
    board_pins_slave::MOTOR1_EN_X);
#endif

#if SLAVE_Y_SENSOR_HW_ENABLED
bool ySensorReady = false;
SPIClass ySsiBus(HSPI);
SlaveMt6701Sensor ySensor = SlaveMt6701Sensor(board_pins_slave::ENCODER2_CS_Y);
#endif

#if SLAVE_Y_MOTOR_HW_ENABLED
bool yMotorReady = false;
BLDCMotor yMotor = BLDCMotor(SLAVE_Y_MOTOR_POLE_PAIRS);
BLDCDriver3PWM yDriver = BLDCDriver3PWM(
    board_pins_slave::MOTOR2_PWM_U_Y,
    board_pins_slave::MOTOR2_PWM_V_Y,
    board_pins_slave::MOTOR2_PWM_W_Y,
    board_pins_slave::MOTOR2_EN_Y);
#endif

#if SLAVE_X_MOTOR_HW_ENABLED || SLAVE_Y_MOTOR_HW_ENABLED
int motorDriverDisabledLevel() {
    return (SLAVE_DRIVER_ENABLE_ACTIVE_HIGH != 0) ? LOW : HIGH;
}
#endif

#if SLAVE_UV_HW_ENABLED
int uvMosLevelFor(bool enabled) {
    return enabled ? board_pins_slave::UV_MOS_ACTIVE_LEVEL
                   : board_pins_slave::UV_MOS_INACTIVE_LEVEL;
}
#endif

#if SLAVE_Y_MOTOR_HW_ENABLED
bool initSlaveYMotorHardware(bool open_loop) {
    yDriver.enable_active_high = SLAVE_DRIVER_ENABLE_ACTIVE_HIGH != 0;
    yDriver.voltage_power_supply = kSlaveYMotorFoc.voltage.supply_v;
    yDriver.voltage_limit = kSlaveYMotorFoc.voltage.driver_limit_v;
    if (!yDriver.init()) {
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }

    yMotor.linkDriver(&yDriver);
    yMotor.voltage_sensor_align = kSlaveYMotorFoc.voltage.align_v;
    yMotor.voltage_limit = open_loop
                               ? fminf(kSlaveYMotorFoc.voltage.motor_limit_v,
                                       kSlaveYMotorFoc.voltage.open_loop_limit_v)
                               : kSlaveYMotorFoc.voltage.motor_limit_v;
    yMotor.velocity_limit = kSlaveYMotorFoc.limit.velocity_rad_s;
    yMotor.torque_controller = TorqueControlType::voltage;
    yMotor.controller = open_loop ? MotionControlType::angle_openloop
                                  : MotionControlType::angle;

#if SLAVE_Y_SENSOR_HW_ENABLED
    if (!open_loop) {
        if (!ySensorReady && !setupSlaveYSensorHardware()) {
            yDriver.disable();
            return false;
        }
        yMotor.linkSensor(&ySensor);
    }
#else
    if (!open_loop) {
        yDriver.disable();
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }
#endif

    yMotor.P_angle.P = kSlaveYMotorFoc.position.p;
    yMotor.P_angle.limit = kSlaveYMotorFoc.limit.velocity_rad_s;
    yMotor.PID_velocity.P = kSlaveYMotorFoc.velocity.p;
    yMotor.PID_velocity.I = kSlaveYMotorFoc.velocity.i;
    yMotor.PID_velocity.D = kSlaveYMotorFoc.velocity.d;
    yMotor.PID_velocity.output_ramp = kSlaveYMotorFoc.velocity.output_ramp;
    yMotor.PID_velocity.limit = kSlaveYMotorFoc.voltage.motor_limit_v;
    yMotor.LPF_velocity.Tf = kSlaveYMotorFoc.filter.velocity_tf;
    yMotor.LPF_angle.Tf = kSlaveYMotorFoc.filter.angle_tf;

    yMotor.init();
    if (open_loop) {
        yMotorReady = true;
        return true;
    }

    if (!yMotor.initFOC()) {
        yDriver.disable();
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }
    yMotorReady = true;
    return true;
}
#endif

}  // namespace

void setUvOutput(bool enabled) {
#if !SLAVE_UV_HW_ENABLED
    (void)enabled;
    sysData.link.uv_out = false;
    return;
#else
    static bool last_enabled = false;
    static bool initialized = false;
    if (initialized && enabled == last_enabled) {
        sysData.link.uv_out = enabled;
        return;
    }

    digitalWrite(board_pins_slave::UV_MOS, uvMosLevelFor(enabled));
    sysData.link.uv_out = enabled;
    last_enabled = enabled;
    initialized = true;
#endif
}

void configureSlaveSafeOutputs() {
    pinMode(board_pins_slave::UV_MOS, OUTPUT);
    digitalWrite(board_pins_slave::UV_MOS, board_pins_slave::UV_MOS_INACTIVE_LEVEL);
    setUvOutput(false);

#if SLAVE_X_MOTOR_HW_ENABLED || SLAVE_Y_MOTOR_HW_ENABLED
    pinMode(board_pins_slave::MOTOR_DRIVER_EN, OUTPUT);
    digitalWrite(board_pins_slave::MOTOR_DRIVER_EN, motorDriverDisabledLevel());
#endif
}

bool setupSlaveXSensorHardware() {
#if SLAVE_X_SENSOR_HW_ENABLED
    if (xSensorReady) {
        return true;
    }
    xSsiBus.begin(board_pins_slave::ENCODER1_CLK_X,
                  board_pins_slave::ENCODER1_DO_X,
                  -1,
                  board_pins_slave::ENCODER1_CS_X);
    xSensor.init(&xSsiBus);
    xSensorReady = true;
    return true;
#else
    addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
    return false;
#endif
}

bool setupSlaveXMotorHardware() {
#if SLAVE_X_MOTOR_HW_ENABLED && SLAVE_X_SENSOR_HW_ENABLED
    if (xMotorReady) {
        return true;
    }
    if (!setupSlaveXSensorHardware()) {
        return false;
    }

    xDriver.enable_active_high = SLAVE_DRIVER_ENABLE_ACTIVE_HIGH != 0;
    xDriver.voltage_power_supply = kSlaveXMotorFoc.voltage.supply_v;
    xDriver.voltage_limit = kSlaveXMotorFoc.voltage.driver_limit_v;
    if (!xDriver.init()) {
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }

    xMotor.linkSensor(&xSensor);
    xMotor.linkDriver(&xDriver);
    xMotor.voltage_sensor_align = kSlaveXMotorFoc.voltage.align_v;
    xMotor.voltage_limit = kSlaveXMotorFoc.voltage.motor_limit_v;
    xMotor.velocity_limit = kSlaveXMotorFoc.limit.velocity_rad_s;
    xMotor.torque_controller = TorqueControlType::voltage;
    xMotor.controller = MotionControlType::angle;
    xMotor.P_angle.P = kSlaveXMotorFoc.position.p;
    xMotor.P_angle.limit = kSlaveXMotorFoc.limit.velocity_rad_s;
    xMotor.PID_velocity.P = kSlaveXMotorFoc.velocity.p;
    xMotor.PID_velocity.I = kSlaveXMotorFoc.velocity.i;
    xMotor.PID_velocity.D = kSlaveXMotorFoc.velocity.d;
    xMotor.PID_velocity.output_ramp = kSlaveXMotorFoc.velocity.output_ramp;
    xMotor.PID_velocity.limit = kSlaveXMotorFoc.voltage.motor_limit_v;
    xMotor.LPF_velocity.Tf = kSlaveXMotorFoc.filter.velocity_tf;
    xMotor.LPF_angle.Tf = kSlaveXMotorFoc.filter.angle_tf;

    xMotor.init();
    if (!xMotor.initFOC()) {
        xDriver.disable();
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }
    xMotorReady = true;
    return true;
#else
    addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
    return false;
#endif
}

bool setupSlaveYSensorHardware() {
#if SLAVE_Y_SENSOR_HW_ENABLED
    if (ySensorReady) {
        return true;
    }
    ySsiBus.begin(board_pins_slave::ENCODER2_CLK_Y,
                  board_pins_slave::ENCODER2_DO_Y,
                  -1,
                  board_pins_slave::ENCODER2_CS_Y);
    ySensor.init(&ySsiBus);
    ySensorReady = true;
    return true;
#else
    addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
    return false;
#endif
}

bool setupSlaveYMotorOpenLoopHardware() {
#if SLAVE_Y_MOTOR_HW_ENABLED
    return initSlaveYMotorHardware(true);
#else
    addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
    return false;
#endif
}

bool setupSlaveYMotorClosedLoopHardware() {
#if SLAVE_Y_MOTOR_HW_ENABLED && SLAVE_Y_SENSOR_HW_ENABLED
    return initSlaveYMotorHardware(false);
#else
    addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
    return false;
#endif
}

float applySlaveXMotorTarget(float target_angle_rad,
                             float fallback_actual_angle_rad,
                             SlaveXMotorStepTiming *timing) {
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    if (timing != nullptr) {
        *timing = {};
    }
#else
    (void)timing;
#endif
#if SLAVE_X_MOTOR_HW_ENABLED && SLAVE_X_SENSOR_HW_ENABLED
    if (!xMotorReady) {
        (void)target_angle_rad;
        return fallback_actual_angle_rad;
    }
    static uint32_t loop_foc_divider = 0;
    const uint32_t foc_interval =
        (SLAVE_X_FOC_EVERY_N_STEPS > 0) ? static_cast<uint32_t>(SLAVE_X_FOC_EVERY_N_STEPS) : 1UL;
    const bool run_loop_foc = loop_foc_divider == 0;
    loop_foc_divider++;
    if (loop_foc_divider >= foc_interval) {
        loop_foc_divider = 0;
    }

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t foc_start_us = micros();
    uint32_t move_start_us = foc_start_us;
#endif
    if (run_loop_foc) {
        xMotor.loopFOC();
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
        move_start_us = micros();
#endif
    }
    static uint32_t move_divider = 0;
    const uint32_t move_interval =
        (SLAVE_X_MOVE_EVERY_N_STEPS > 0) ? static_cast<uint32_t>(SLAVE_X_MOVE_EVERY_N_STEPS) : 1UL;
    const bool run_move = move_divider == 0;
    move_divider++;
    if (move_divider >= move_interval) {
        move_divider = 0;
    }
    if (run_move) {
        xMotor.move(target_angle_rad);
    }
    const float actual_angle_rad = xMotor.shaft_angle;
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t done_us = micros();
    if (timing != nullptr) {
        timing->loop_foc_us = run_loop_foc ? (move_start_us - foc_start_us) : 0UL;
        timing->move_us = run_move ? (done_us - move_start_us) : 0UL;
        timing->sensor_us = run_loop_foc ? xSensor.lastReadDurationUs() : 0UL;
        timing->loop_foc_ran = run_loop_foc ? 1UL : 0UL;
    }
#endif
    return actual_angle_rad;
#else
    (void)target_angle_rad;
    return fallback_actual_angle_rad;
#endif
}

float applySlaveXMotorTarget(float target_angle_rad, float fallback_actual_angle_rad) {
    return applySlaveXMotorTarget(target_angle_rad, fallback_actual_angle_rad, nullptr);
}

float runSlaveXMotorPerfIsolationStep(float target_angle_rad,
                                      float fallback_actual_angle_rad,
                                      SlaveXMotorStepTiming *timing) {
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    if (timing != nullptr) {
        *timing = {};
    }
#else
    (void)timing;
#endif

#if SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_MODE_TIMER_EMPTY
    (void)target_angle_rad;
    return fallback_actual_angle_rad;
#elif SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_MODE_SENSOR_ONLY
#if SLAVE_X_SENSOR_HW_ENABLED
    if (!xSensorReady) {
        return fallback_actual_angle_rad;
    }
    xSensor.update();
    const float actual_angle_rad = xSensor.getMechanicalAngle();
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    if (timing != nullptr) {
        timing->sensor_us = xSensor.lastReadDurationUs();
    }
#endif
    return actual_angle_rad;
#else
    (void)target_angle_rad;
    return fallback_actual_angle_rad;
#endif
#else
#if SLAVE_X_MOTOR_HW_ENABLED && SLAVE_X_SENSOR_HW_ENABLED
    if (!xMotorReady) {
        (void)target_angle_rad;
        return fallback_actual_angle_rad;
    }
#if SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_MODE_LOOPFOC_ONLY
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t foc_start_us = micros();
#endif
    xMotor.loopFOC();
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t foc_done_us = micros();
    if (timing != nullptr) {
        timing->loop_foc_us = foc_done_us - foc_start_us;
        timing->sensor_us = xSensor.lastReadDurationUs();
        timing->loop_foc_ran = 1UL;
    }
#endif
    return xMotor.shaft_angle;
#elif SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_MODE_MOVE_ONLY
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t move_start_us = micros();
#endif
    xMotor.move(target_angle_rad);
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t move_done_us = micros();
    if (timing != nullptr) {
        timing->move_us = move_done_us - move_start_us;
    }
#endif
    return xMotor.shaft_angle;
#else
    return applySlaveXMotorTarget(target_angle_rad, fallback_actual_angle_rad, timing);
#endif
#else
    (void)target_angle_rad;
    return fallback_actual_angle_rad;
#endif
#endif
}

float runSlaveXMotorPerfIsolationStep(float target_angle_rad, float fallback_actual_angle_rad) {
    return runSlaveXMotorPerfIsolationStep(target_angle_rad, fallback_actual_angle_rad, nullptr);
}

float applySlaveYMotorTarget(float target_angle_rad,
                             float fallback_actual_angle_rad,
                             SlaveXMotorStepTiming *timing) {
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    if (timing != nullptr) {
        *timing = {};
    }
#else
    (void)timing;
#endif
#if SLAVE_Y_MOTOR_HW_ENABLED
    if (!yMotorReady) {
        (void)target_angle_rad;
        return fallback_actual_angle_rad;
    }
    static uint32_t loop_foc_divider = 0;
    const uint32_t foc_interval =
        (SLAVE_Y_FOC_EVERY_N_STEPS > 0) ? static_cast<uint32_t>(SLAVE_Y_FOC_EVERY_N_STEPS) : 1UL;
    const bool run_loop_foc = loop_foc_divider == 0;
    loop_foc_divider++;
    if (loop_foc_divider >= foc_interval) {
        loop_foc_divider = 0;
    }

    const bool y_open_loop = slaveRunModeUsesOpenLoopMotor(AXIS_Y);
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t foc_start_us = micros();
    uint32_t move_start_us = foc_start_us;
#endif
    if (run_loop_foc && !y_open_loop) {
        yMotor.loopFOC();
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
        move_start_us = micros();
#endif
    }
    static uint32_t move_divider = 0;
    const uint32_t move_interval =
        (SLAVE_Y_MOVE_EVERY_N_STEPS > 0) ? static_cast<uint32_t>(SLAVE_Y_MOVE_EVERY_N_STEPS) : 1UL;
    const bool run_move = move_divider == 0;
    move_divider++;
    if (move_divider >= move_interval) {
        move_divider = 0;
    }
    if (run_move) {
        yMotor.move(target_angle_rad);
    }
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t done_us = micros();
    if (timing != nullptr) {
        timing->loop_foc_us =
            (run_loop_foc && !y_open_loop)
                ? (move_start_us - foc_start_us)
                : 0UL;
        timing->move_us = run_move ? (done_us - move_start_us) : 0UL;
#if SLAVE_Y_SENSOR_HW_ENABLED
        timing->sensor_us =
            (run_loop_foc && !y_open_loop)
                ? ySensor.lastReadDurationUs()
                : 0UL;
#else
        timing->sensor_us = 0UL;
#endif
        timing->loop_foc_ran =
            (run_loop_foc && !y_open_loop) ? 1UL : 0UL;
    }
#endif
    return yMotor.shaft_angle;
#else
    (void)target_angle_rad;
    return fallback_actual_angle_rad;
#endif
}

float applySlaveYMotorTarget(float target_angle_rad, float fallback_actual_angle_rad) {
    return applySlaveYMotorTarget(target_angle_rad, fallback_actual_angle_rad, nullptr);
}

bool sampleSlaveYSensorForStatus(float *angle_rad, uint16_t *raw_angle) {
#if SLAVE_Y_SENSOR_HW_ENABLED
    if (!ySensorReady) {
        return false;
    }
    ySensor.update();
    if (angle_rad != nullptr) {
        *angle_rad = ySensor.getMechanicalAngle();
    }
    if (raw_angle != nullptr) {
        *raw_angle = ySensor.rawAngle();
    }
    return true;
#else
    (void)angle_rad;
    (void)raw_angle;
    return false;
#endif
}
