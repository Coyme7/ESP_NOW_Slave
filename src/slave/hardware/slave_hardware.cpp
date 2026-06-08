#include "slave/hardware/slave_hardware.h"

#include <Arduino.h>
#include <SPI.h>
#include <board/board_pins_slave.h>
#include <math.h>

#include "common/system_state.h"
#include "slave/config/slave_config.h"
#include "slave/diagnostics/current_probe.h"
#include "slave/hardware/slave_current_sense_adc1.h"
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
#if SLAVE_ENABLE_CURRENT_SENSE
SlaveAdc1CurrentSense xCurrentSense = SlaveAdc1CurrentSense(
    kSlaveCurrentSenseHardware.shunt_ohm,
    kSlaveCurrentSenseHardware.gain,
    board_pins_slave::MOTOR1_CURRENT_A_X,
    board_pins_slave::MOTOR1_CURRENT_B_X);
bool xCurrentSenseReady = false;
#endif
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
#if SLAVE_ENABLE_CURRENT_SENSE
SlaveAdc1CurrentSense yCurrentSense = SlaveAdc1CurrentSense(
    kSlaveCurrentSenseHardware.shunt_ohm,
    kSlaveCurrentSenseHardware.gain,
    board_pins_slave::MOTOR2_CURRENT_A_Y,
    board_pins_slave::MOTOR2_CURRENT_B_Y);
bool yCurrentSenseReady = false;
#endif
#endif

#if SLAVE_X_MOTOR_HW_ENABLED || SLAVE_Y_MOTOR_HW_ENABLED
int motorDriverDisabledLevel() {
    return (SLAVE_DRIVER_ENABLE_ACTIVE_HIGH != 0) ? LOW : HIGH;
}

__attribute__((unused)) Direction slaveFocDirectionFromSign(int8_t direction_sign) {
    return (direction_sign < 0) ? Direction::CCW : Direction::CW;
}

__attribute__((unused)) const char *slaveFocDirectionName(Direction direction) {
    switch (direction) {
        case Direction::CW:
            return "CW";
        case Direction::CCW:
            return "CCW";
        case Direction::UNKNOWN:
        default:
            return "UNKNOWN";
    }
}

#if SLAVE_ENABLE_CURRENT_SENSE
DQCurrent_s dqCurrentFromLastSample(SlaveAdc1CurrentSense &current_sense,
                                    float electrical_angle) {
    const PhaseCurrent_s phase_current = current_sense.lastPhaseCurrents();
    const ABCurrent_s ab_current = current_sense.getABCurrents(phase_current);
    return current_sense.getDQCurrents(ab_current, electrical_angle);
}
#endif
#endif

#if SLAVE_UV_HW_ENABLED
int uvMosLevelFor(bool enabled) {
    return enabled ? board_pins_slave::UV_MOS_ACTIVE_LEVEL
                   : board_pins_slave::UV_MOS_INACTIVE_LEVEL;
}
#endif

#if SLAVE_X_MOTOR_HW_ENABLED || SLAVE_Y_MOTOR_HW_ENABLED
void markSlaveMotorInitFault(BLDCDriver3PWM &driver) {
    driver.disable();
    setUvOutput(false);
    addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
}

bool initSlaveMotorFocForAxis(const char *axis,
                              BLDCMotor &motor,
                              BLDCDriver3PWM &driver,
                              int8_t direction_sign,
                              float zero_electric_angle_rad) {
#if SLAVE_SKIP_FOC_ALIGNMENT_ON_STARTUP
    motor.sensor_direction = slaveFocDirectionFromSign(direction_sign);
    motor.zero_electric_angle = zero_electric_angle_rad;
#if SLAVE_BOOT_LOG_ENABLED
    Serial.printf("[Slave] motor_diag axis=%s foc_startup=fast skip_align=1 direction=%s zero=%.6frad\n",
                  axis,
                  slaveFocDirectionName(motor.sensor_direction),
                  motor.zero_electric_angle);
#endif
#else
#if SLAVE_BOOT_LOG_ENABLED
    Serial.printf("[Slave] motor_diag axis=%s foc_startup=calibrate skip_align=0\n", axis);
#endif
#endif

    if (!motor.initFOC()) {
        markSlaveMotorInitFault(driver);
        return false;
    }

#if SLAVE_BOOT_LOG_ENABLED
    Serial.printf("[Slave] motor_diag axis=%s foc_ready direction=%s zero=%.6frad\n",
                  axis,
                  slaveFocDirectionName(motor.sensor_direction),
                  motor.zero_electric_angle);
#endif
    return true;
}
#endif

#if SLAVE_X_MOTOR_HW_ENABLED || SLAVE_Y_MOTOR_HW_ENABLED
void applySlaveMotorFocConfig(BLDCMotor &motor,
                              const SlaveMotorFocConfig &config,
                              bool open_loop) {
    motor.voltage_sensor_align = config.voltage.align_v;
    motor.voltage_limit = open_loop
                              ? fminf(config.voltage.motor_limit_v,
                                      config.voltage.open_loop_limit_v)
                              : config.voltage.motor_limit_v;
    motor.velocity_limit = config.limit.velocity_rad_s;
    motor.current_limit = config.limit.current_a;

    if (open_loop) {
        motor.torque_controller = TorqueControlType::voltage;
        motor.controller = MotionControlType::angle_openloop;
    } else {
#if SLAVE_ENABLE_CURRENT_SENSE
        motor.torque_controller = TorqueControlType::foc_current;
#if SLAVE_ENABLE_ZERO_CURRENT_TEST
        motor.controller = MotionControlType::torque;
#else
        motor.controller = MotionControlType::angle;
#endif
#else
        motor.torque_controller = TorqueControlType::voltage;
        motor.controller = MotionControlType::angle;
#endif
    }

    motor.P_angle.P = config.position.p;
    motor.P_angle.I = config.position.i;
    motor.P_angle.D = config.position.d;
    motor.P_angle.limit = config.limit.velocity_rad_s;
    motor.PID_velocity.P = config.velocity.p;
    motor.PID_velocity.I = config.velocity.i;
    motor.PID_velocity.D = config.velocity.d;
    motor.PID_velocity.output_ramp = config.velocity.output_ramp;
    motor.PID_velocity.limit =
#if SLAVE_ENABLE_CURRENT_SENSE
        open_loop ? config.voltage.motor_limit_v : config.limit.current_a;
#else
        config.voltage.motor_limit_v;
#endif
    motor.LPF_velocity.Tf = config.filter.velocity_tf;
    motor.LPF_angle.Tf = config.filter.angle_tf;

    motor.PID_current_q.P = config.current_loop.q.p;
    motor.PID_current_q.I = config.current_loop.q.i;
    motor.PID_current_q.D = config.current_loop.q.d;
    motor.PID_current_q.output_ramp = config.current_loop.q.output_ramp;
    motor.PID_current_q.limit = config.voltage.motor_limit_v;
    motor.PID_current_d.P = config.current_loop.d.p;
    motor.PID_current_d.I = config.current_loop.d.i;
    motor.PID_current_d.D = config.current_loop.d.d;
    motor.PID_current_d.output_ramp = config.current_loop.d.output_ramp;
    motor.PID_current_d.limit = config.voltage.motor_limit_v;
    motor.LPF_current_q.Tf = config.current_loop.lpf_tf;
    motor.LPF_current_d.Tf = config.current_loop.lpf_tf;
}

BLDCMotor *slaveMotorForAxis(AxisId axis) {
    if (axis == AXIS_X) {
#if SLAVE_X_MOTOR_HW_ENABLED
        return xMotorReady ? &xMotor : nullptr;
#else
        return nullptr;
#endif
    }
#if SLAVE_Y_MOTOR_HW_ENABLED
    return yMotorReady ? &yMotor : nullptr;
#else
    return nullptr;
#endif
}

const SlaveMotorFocConfig &slaveMotorConfigForAxis(AxisId axis) {
    return axis == AXIS_X ? kSlaveXMotorFoc : kSlaveYMotorFoc;
}

__attribute__((unused)) const char *slaveTorqueControllerName(bool open_loop) {
    if (open_loop) {
        return "voltage_open_loop";
    }
#if SLAVE_ENABLE_CURRENT_SENSE
    return "foc_current";
#else
    return "voltage";
#endif
}
#endif

#if (SLAVE_X_MOTOR_HW_ENABLED || SLAVE_Y_MOTOR_HW_ENABLED) && SLAVE_ENABLE_CURRENT_SENSE
bool initSlaveCurrentSenseForAxis(const char *axis_name,
                                  BLDCMotor &motor,
                                  BLDCDriver3PWM &driver,
                                  SlaveAdc1CurrentSense &current_sense,
                                  SlaveMt6701Sensor &sensor,
                                  const SlaveMotorFocConfig &motor_config,
                                  int current_pin_a,
                                  int current_pin_b,
                                  int current_gain_sign_a,
                                  int current_gain_sign_b) {
    current_sense.linkDriver(&driver);
    current_sense.skip_align = kSlaveCurrentSenseHardware.skip_align;
#if SLAVE_BOOT_LOG_ENABLED
    Serial.printf("[Slave] motor_diag axis=%s current_sense adc1 shunt=%.4fohm gain=%.2f pins=%d,%d raw_to_v=%.9f skip_align=%u\n",
                  axis_name,
                  kSlaveCurrentSenseHardware.shunt_ohm,
                  kSlaveCurrentSenseHardware.gain,
                  current_pin_a,
                  current_pin_b,
                  kSlaveCurrentSenseHardware.adc_raw_to_voltage_v,
                  kSlaveCurrentSenseHardware.skip_align ? 1 : 0);
#endif
    if (!current_sense.init()) {
        markSlaveMotorInitFault(driver);
        return false;
    }

    SlaveMotorDiagnosticsContext diagnostics = {
        axis_name,
        motor,
        driver,
        current_sense,
        sensor,
        motor_config,
    };
    if (!calibrateSlaveCurrentSenseOffsets(diagnostics)) {
        markSlaveMotorInitFault(driver);
        return false;
    }
    current_sense.gain_a *= (current_gain_sign_a < 0) ? -1.0f : 1.0f;
    current_sense.gain_b *= (current_gain_sign_b < 0) ? -1.0f : 1.0f;
#if SLAVE_BOOT_LOG_ENABLED
    Serial.printf("[Slave] motor_diag axis=%s current_sense offsets ia=%.3fV ib=%.3fV gain_a=%.2f gain_b=%.2f sign_a=%d sign_b=%d raw_adc=%d,%d raw16=%d,%d\n",
                  axis_name,
                  current_sense.offset_ia,
                  current_sense.offset_ib,
                  current_sense.gain_a,
                  current_sense.gain_b,
                  current_gain_sign_a,
                  current_gain_sign_b,
                  current_sense.readRawA(),
                  current_sense.readRawB(),
                  current_sense.readRawUnmaskedA(),
                  current_sense.readRawUnmaskedB());
#endif
    motor.linkCurrentSense(&current_sense);
#if SLAVE_ENABLE_CURRENT_SENSE_DIAG_TEST
    runSlaveCurrentSenseProbe(diagnostics);
    markSlaveMotorInitFault(driver);
    return false;
#endif
    return true;
}
#endif

#if SLAVE_Y_MOTOR_HW_ENABLED
bool initSlaveYMotorHardware(bool open_loop) {
#if SLAVE_Y_SENSOR_HW_ENABLED
    if (!open_loop) {
        if (!ySensorReady && !setupSlaveYSensorHardware()) {
            setUvOutput(false);
            return false;
        }
    }
#else
    if (!open_loop) {
        setUvOutput(false);
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }
#endif

    yDriver.enable_active_high = SLAVE_DRIVER_ENABLE_ACTIVE_HIGH != 0;
    yDriver.voltage_power_supply = kSlaveYMotorFoc.voltage.supply_v;
    yDriver.voltage_limit = kSlaveYMotorFoc.voltage.driver_limit_v;
    if (!yDriver.init()) {
        markSlaveMotorInitFault(yDriver);
        return false;
    }
    yDriver.disable();

#if SLAVE_Y_SENSOR_HW_ENABLED
    if (!open_loop) {
        yMotor.linkSensor(&ySensor);
    }
#endif
    yMotor.linkDriver(&yDriver);
    applySlaveMotorFocConfig(yMotor, kSlaveYMotorFoc, open_loop);
#if SLAVE_BOOT_LOG_ENABLED
    Serial.printf("[Slave] motor_diag axis=Y torque_controller=%s current_limit=%.3fA vlim=%.2fV align=%.2fV pp=%u\n",
                  slaveTorqueControllerName(open_loop),
                  kSlaveYMotorFoc.limit.current_a,
                  yMotor.voltage_limit,
                  kSlaveYMotorFoc.voltage.align_v,
                  static_cast<unsigned int>(SLAVE_Y_MOTOR_POLE_PAIRS));
#endif

#if SLAVE_ENABLE_CURRENT_SENSE && SLAVE_Y_SENSOR_HW_ENABLED
    if (!open_loop) {
        if (!initSlaveCurrentSenseForAxis("Y",
                                          yMotor,
                                          yDriver,
                                          yCurrentSense,
                                          ySensor,
                                          kSlaveYMotorFoc,
                                          board_pins_slave::MOTOR2_CURRENT_A_Y,
                                          board_pins_slave::MOTOR2_CURRENT_B_Y,
                                          kSlaveYCurrentSenseAxis.gain_sign_a,
                                          kSlaveYCurrentSenseAxis.gain_sign_b)) {
            return false;
        }
        yCurrentSenseReady = true;
    }
#endif

    yMotor.init();
    if (open_loop) {
        yMotorReady = true;
        return true;
    }

    if (!initSlaveMotorFocForAxis("Y",
                                  yMotor,
                                  yDriver,
                                  SLAVE_Y_FOC_SENSOR_DIRECTION,
                                  SLAVE_Y_ZERO_ELECTRIC_ANGLE_RAD)) {
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
        markSlaveMotorInitFault(xDriver);
        return false;
    }
    xDriver.disable();

    xMotor.linkSensor(&xSensor);
    xMotor.linkDriver(&xDriver);
    applySlaveMotorFocConfig(xMotor, kSlaveXMotorFoc, false);
#if SLAVE_BOOT_LOG_ENABLED
    Serial.printf("[Slave] motor_diag axis=X torque_controller=%s current_limit=%.3fA vlim=%.2fV align=%.2fV pp=%u\n",
                  slaveTorqueControllerName(false),
                  kSlaveXMotorFoc.limit.current_a,
                  xMotor.voltage_limit,
                  kSlaveXMotorFoc.voltage.align_v,
                  static_cast<unsigned int>(SLAVE_X_MOTOR_POLE_PAIRS));
#endif

#if SLAVE_ENABLE_CURRENT_SENSE
    if (!initSlaveCurrentSenseForAxis("X",
                                      xMotor,
                                      xDriver,
                                      xCurrentSense,
                                      xSensor,
                                      kSlaveXMotorFoc,
                                      board_pins_slave::MOTOR1_CURRENT_A_X,
                                      board_pins_slave::MOTOR1_CURRENT_B_X,
                                      kSlaveXCurrentSenseAxis.gain_sign_a,
                                      kSlaveXCurrentSenseAxis.gain_sign_b)) {
        return false;
    }
    xCurrentSenseReady = true;
#endif

    xMotor.init();
    if (!initSlaveMotorFocForAxis("X",
                                  xMotor,
                                  xDriver,
                                  SLAVE_X_FOC_SENSOR_DIRECTION,
                                  SLAVE_X_ZERO_ELECTRIC_ANGLE_RAD)) {
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
#if SLAVE_ENABLE_CURRENT_SENSE && SLAVE_ENABLE_ZERO_CURRENT_TEST
        xMotor.move(0.0f);
#else
        xMotor.move(target_angle_rad);
#endif
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
#if SLAVE_ENABLE_CURRENT_SENSE && SLAVE_ENABLE_ZERO_CURRENT_TEST
    xMotor.move(0.0f);
#else
    xMotor.move(target_angle_rad);
#endif
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
#if SLAVE_ENABLE_CURRENT_SENSE && SLAVE_ENABLE_ZERO_CURRENT_TEST
        yMotor.move(0.0f);
#else
        yMotor.move(target_angle_rad);
#endif
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

SlaveMotorCurrentSnapshot snapshotSlaveMotorCurrent() {
    SlaveMotorCurrentSnapshot snapshot = {};
#if SLAVE_X_MOTOR_HW_ENABLED
    snapshot.x.motor_ready = xMotorReady;
#if SLAVE_ENABLE_CURRENT_SENSE
    const DQCurrent_s x_current = xCurrentSenseReady
                                      ? dqCurrentFromLastSample(xCurrentSense, xMotor.electrical_angle)
                                      : xMotor.current;
    snapshot.x.current_q_a = x_current.q;
    snapshot.x.current_d_a = x_current.d;
#else
    snapshot.x.current_q_a = xMotor.current.q;
    snapshot.x.current_d_a = xMotor.current.d;
#endif
    snapshot.x.voltage_q_v = xMotor.voltage.q;
    snapshot.x.voltage_d_v = xMotor.voltage.d;
#if SLAVE_ENABLE_CURRENT_SENSE
    snapshot.x.current_sense_ready = xCurrentSenseReady;
    snapshot.x.raw_adc_a = xCurrentSense.lastRawA();
    snapshot.x.raw_adc_b = xCurrentSense.lastRawB();
    snapshot.x.offset_ia_v = xCurrentSense.offset_ia;
    snapshot.x.offset_ib_v = xCurrentSense.offset_ib;
#endif
#endif

#if SLAVE_Y_MOTOR_HW_ENABLED
    snapshot.y.motor_ready = yMotorReady;
#if SLAVE_ENABLE_CURRENT_SENSE
    const DQCurrent_s y_current = yCurrentSenseReady
                                      ? dqCurrentFromLastSample(yCurrentSense, yMotor.electrical_angle)
                                      : yMotor.current;
    snapshot.y.current_q_a = y_current.q;
    snapshot.y.current_d_a = y_current.d;
#else
    snapshot.y.current_q_a = yMotor.current.q;
    snapshot.y.current_d_a = yMotor.current.d;
#endif
    snapshot.y.voltage_q_v = yMotor.voltage.q;
    snapshot.y.voltage_d_v = yMotor.voltage.d;
#if SLAVE_ENABLE_CURRENT_SENSE
    snapshot.y.current_sense_ready = yCurrentSenseReady;
    snapshot.y.raw_adc_a = yCurrentSense.lastRawA();
    snapshot.y.raw_adc_b = yCurrentSense.lastRawB();
    snapshot.y.offset_ia_v = yCurrentSense.offset_ia;
    snapshot.y.offset_ib_v = yCurrentSense.offset_ib;
#endif
#endif
    return snapshot;
}

bool configureSlaveMotorTuning(AxisId axis,
                               uint8_t mode,
                               float p,
                               float i,
                               float d,
                               float current_limit_a,
                               float voltage_limit_v,
                               float velocity_limit_rad_s) {
#if SLAVE_X_MOTOR_HW_ENABLED || SLAVE_Y_MOTOR_HW_ENABLED
    BLDCMotor *motor = slaveMotorForAxis(axis);
    if (motor == nullptr || slaveRunModeUsesOpenLoopMotor(axis)) {
        return false;
    }

    const SlaveMotorFocConfig &config = slaveMotorConfigForAxis(axis);
    applySlaveMotorFocConfig(*motor, config, false);
    motor->current_limit = fminf(current_limit_a, config.limit.current_a);
    motor->voltage_limit = fminf(voltage_limit_v, config.voltage.motor_limit_v);
    motor->velocity_limit = fminf(velocity_limit_rad_s, config.limit.velocity_rad_s);
    motor->PID_current_q.limit = motor->voltage_limit;
    motor->PID_current_d.limit = motor->voltage_limit;
    motor->PID_velocity.limit = motor->current_limit;
    motor->P_angle.limit = motor->velocity_limit;

    switch (mode) {
        case SLAVE_MOTOR_TUNING_CURRENT_Q:
#if SLAVE_ENABLE_CURRENT_SENSE
            motor->controller = MotionControlType::torque;
            motor->PID_current_q.P = p;
            motor->PID_current_q.I = i;
            motor->PID_current_q.D = d;
            motor->PID_current_q.reset();
            motor->PID_current_d.reset();
            break;
#else
            return false;
#endif
        case SLAVE_MOTOR_TUNING_VELOCITY:
            motor->controller = MotionControlType::velocity;
            motor->PID_velocity.P = p;
            motor->PID_velocity.I = i;
            motor->PID_velocity.D = d;
            motor->PID_velocity.reset();
            break;
        case SLAVE_MOTOR_TUNING_ANGLE:
            motor->controller = MotionControlType::angle;
            motor->P_angle.P = p;
            motor->P_angle.I = i;
            motor->P_angle.D = d;
            motor->P_angle.reset();
            motor->PID_velocity.reset();
            break;
        default:
            return false;
    }
    motor->target = 0.0f;
    motor->current_sp = 0.0f;
    return true;
#else
    (void)axis;
    (void)mode;
    (void)p;
    (void)i;
    (void)d;
    (void)current_limit_a;
    (void)voltage_limit_v;
    (void)velocity_limit_rad_s;
    return false;
#endif
}

void restoreSlaveMotorTuning(AxisId axis) {
#if SLAVE_X_MOTOR_HW_ENABLED || SLAVE_Y_MOTOR_HW_ENABLED
    BLDCMotor *motor = slaveMotorForAxis(axis);
    if (motor == nullptr) {
        return;
    }
    applySlaveMotorFocConfig(*motor, slaveMotorConfigForAxis(axis), false);
    motor->PID_current_q.reset();
    motor->PID_current_d.reset();
    motor->PID_velocity.reset();
    motor->P_angle.reset();
    motor->target = motor->shaft_angle;
    motor->current_sp = 0.0f;
#else
    (void)axis;
#endif
}

SlaveMotorTuningFeedback snapshotSlaveMotorTuning(AxisId axis) {
    SlaveMotorTuningFeedback feedback = {};
#if SLAVE_X_MOTOR_HW_ENABLED || SLAVE_Y_MOTOR_HW_ENABLED
    BLDCMotor *motor = slaveMotorForAxis(axis);
    if (motor == nullptr) {
        return feedback;
    }
    feedback.ready = true;
    feedback.shaft_angle_rad = motor->shaft_angle;
    feedback.shaft_velocity_rad_s = motor->shaft_velocity;
    feedback.current_q_a = motor->current.q;
    feedback.current_d_a = motor->current.d;
    feedback.voltage_q_v = motor->voltage.q;
    feedback.voltage_d_v = motor->voltage.d;
    feedback.current_setpoint_a = motor->current_sp;
    feedback.velocity_setpoint_rad_s = motor->shaft_velocity_sp;
    feedback.angle_setpoint_rad = motor->shaft_angle_sp;
#else
    (void)axis;
#endif
    return feedback;
}
