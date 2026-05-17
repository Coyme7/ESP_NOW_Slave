#include "slave/hardware/slave_hardware.h"

#include <Arduino.h>
#include <board/board_pins_slave.h>
#include <math.h>

#include "common/system_state.h"
#include "slave/config/slave_config.h"
#include "slave/hardware/slave_mt6701_sensor.h"

#if SLAVE_X_MOTOR_HW_ENABLED || SLAVE_Y_MOTOR_HW_ENABLED
#include <SimpleFOC.h>
#endif

// 从机硬件适配层。
// 这里集中处理 UV MOS、SimpleFOC X 轴对象和安全输出，运动算法不直接碰板级引脚。
// Y 轴当前只保持禁用状态，后续接入时应按同样方式独立封装。

namespace {

#if SLAVE_X_MOTOR_HW_ENABLED
bool xMotorReady = false;
#endif

#ifndef SLAVE_X_MOTOR_POLE_PAIRS
// 当前 2804 临时测试默认值，沿用主机侧 2804 配置。
// 后续接回 2208 云台时，请用 build_flags 覆盖或改为实测 2208 极对数。
#define SLAVE_X_MOTOR_POLE_PAIRS 7
#endif

#if SLAVE_X_MOTOR_HW_ENABLED
BLDCMotor xMotor = BLDCMotor(SLAVE_X_MOTOR_POLE_PAIRS);
BLDCDriver3PWM xDriver = BLDCDriver3PWM(
    board_pins_slave::MOTOR1_PWM_U_X,
    board_pins_slave::MOTOR1_PWM_V_X,
    board_pins_slave::MOTOR1_PWM_W_X,
    board_pins_slave::MOTOR1_EN_X);
SlaveMt6701Sensor xSensor = SlaveMt6701Sensor(board_pins_slave::ENCODER1_CS_X);
#endif

#if SLAVE_Y_SENSOR_HW_ENABLED
bool ySensorReady = false;
SlaveMt6701Sensor ySensor = SlaveMt6701Sensor(board_pins_slave::ENCODER2_CS_Y);
#endif

#if SLAVE_Y_MOTOR_HW_ENABLED
bool yMotorReady = false;
BLDCMotor yMotor = BLDCMotor(SLAVE_X_MOTOR_POLE_PAIRS);
BLDCDriver3PWM yDriver = BLDCDriver3PWM(
    board_pins_slave::MOTOR2_PWM_U_Y,
    board_pins_slave::MOTOR2_PWM_V_Y,
    board_pins_slave::MOTOR2_PWM_W_Y,
    board_pins_slave::MOTOR2_EN_Y);
#endif

}  // namespace

void setUvPen(bool enabled) {
#if !SLAVE_UV_HW_ENABLED
    (void)enabled;
    sysData.link.pen_down = false;
    return;
#else
    // 记录上一次输出状态，避免安全任务 1 kHz 重复写同样的 GPIO。
    // 即便输出未变化，也同步 sysData.link.pen_down，保证串口状态反映真实 UV 输出。
    static bool last_enabled = false;
    static bool initialized = false;
    if (initialized && enabled == last_enabled) {
        sysData.link.pen_down = enabled;
        return;
    }

    // 两路 MOS 同步控制。当前设计中 UV 默认关闭，只有安全联锁允许时才拉高。
    digitalWrite(board_pins_slave::UV_MOS_1, enabled ? HIGH : LOW);
    digitalWrite(board_pins_slave::UV_MOS_2, enabled ? HIGH : LOW);
    sysData.link.pen_down = enabled;
    last_enabled = enabled;
    initialized = true;
#endif
}

void configureSlaveSafeOutputs() {
    // 启动安全：Wi-Fi 和 RTOS 任务启动前，先关闭紫光和两路电机使能。
    pinMode(board_pins_slave::UV_MOS_1, OUTPUT);
    pinMode(board_pins_slave::UV_MOS_2, OUTPUT);
    digitalWrite(board_pins_slave::UV_MOS_1, LOW);
    digitalWrite(board_pins_slave::UV_MOS_2, LOW);
    setUvPen(false);

#if SLAVE_X_MOTOR_HW_ENABLED
    pinMode(board_pins_slave::MOTOR1_EN_X, OUTPUT);
    digitalWrite(board_pins_slave::MOTOR1_EN_X, LOW);
#endif
#if SLAVE_Y_MOTOR_HW_ENABLED
    pinMode(board_pins_slave::MOTOR2_EN_Y, OUTPUT);
    digitalWrite(board_pins_slave::MOTOR2_EN_Y, LOW);
#endif
}

bool setupSlaveXMotorHardware() {
#if SLAVE_X_MOTOR_HW_ENABLED
    // 输入：当前临时 MT6701 + 2804，后续正式 2208 云台需重新标定。
    // 输出：限压角度控制就绪；故障时关闭驱动并锁存 MOTOR_OUTPUT_DISABLED。
    SPI.begin(board_pins_slave::ENCODER1_CLK_X,
              board_pins_slave::ENCODER1_DO_X,
              -1,
              board_pins_slave::ENCODER1_CS_X);
    xSensor.init(&SPI);

    // 从机当前使用电压角度控制，先给驱动和电机都设置保守限压。
    // 真实云台调试时需要在不开 UV 的情况下逐步验证方向、零点和温升。
    xDriver.enable_active_high = SLAVE_DRIVER_ENABLE_ACTIVE_HIGH != 0;
    xDriver.voltage_power_supply = kSlaveMotorFoc.supply_voltage_v;
    xDriver.voltage_limit = kSlaveMotorFoc.driver_voltage_limit_v;
    if (!xDriver.init()) {
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }

    xMotor.linkSensor(&xSensor);
    xMotor.linkDriver(&xDriver);

    // angle 模式下 target_angle_rad 是机械角目标。velocity_limit 防止大幅命令突变时追得过猛。
    xMotor.voltage_sensor_align = kSlaveMotorFoc.align_voltage_v;
    xMotor.voltage_limit = kSlaveMotorFoc.motor_voltage_limit_v;
    xMotor.velocity_limit = kSlaveMotorFoc.velocity_limit_rad_s;
    xMotor.torque_controller = TorqueControlType::voltage;
    xMotor.controller = MotionControlType::angle;

    // 位置环和速度环使用保守默认值。首次真实闭环只追求稳定收敛，
    // 不追求快；如果发散，优先检查方向/零点/相序，而不是继续加大增益。
    xMotor.P_angle.P = kSlaveMotorFoc.angle_p;
    xMotor.P_angle.limit = kSlaveMotorFoc.velocity_limit_rad_s;
    xMotor.PID_velocity.P = kSlaveMotorFoc.velocity_pid_p;
    xMotor.PID_velocity.I = kSlaveMotorFoc.velocity_pid_i;
    xMotor.PID_velocity.D = kSlaveMotorFoc.velocity_pid_d;
    xMotor.PID_velocity.output_ramp = kSlaveMotorFoc.velocity_pid_ramp;
    xMotor.PID_velocity.limit = kSlaveMotorFoc.motor_voltage_limit_v;
    xMotor.LPF_velocity.Tf = kSlaveMotorFoc.velocity_lpf_tf;
    xMotor.LPF_angle.Tf = kSlaveMotorFoc.angle_lpf_tf;

    // initFOC 失败时禁用驱动并锁存故障，后续控制步不会继续真实输出。
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

bool setupSlaveYHardware() {
#if SLAVE_Y_SENSOR_HW_ENABLED
    // Y 轴传感器只在显式开启时初始化。默认配置不访问 Y SPI、Y CS 或 Y 电机引脚。
    SPI.begin(board_pins_slave::ENCODER2_CLK_Y,
              board_pins_slave::ENCODER2_DO_Y,
              -1,
              board_pins_slave::ENCODER2_CS_Y);
    ySensor.init(&SPI);
    ySensorReady = true;
#endif

#if SLAVE_Y_MOTOR_HW_ENABLED
    // Y 电机真实输出必须在传感器、方向、零点和限位确认后再开启。
    yDriver.enable_active_high = SLAVE_DRIVER_ENABLE_ACTIVE_HIGH != 0;
    yDriver.voltage_power_supply = kSlaveMotorFoc.supply_voltage_v;
    yDriver.voltage_limit = kSlaveMotorFoc.driver_voltage_limit_v;
    if (!yDriver.init()) {
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }

    yMotor.linkDriver(&yDriver);
    yMotor.voltage_sensor_align = kSlaveMotorFoc.align_voltage_v;
    yMotor.voltage_limit = kSlaveMotorFoc.motor_voltage_limit_v;
    yMotor.velocity_limit = kSlaveMotorFoc.velocity_limit_rad_s;
    yMotor.torque_controller = TorqueControlType::voltage;
    if (SLAVE_Y_BRINGUP_MODE == SLAVE_Y_BRINGUP_MOTOR_OPEN_LOOP) {
        // YMotorOpenLoop 只用于低电压小幅度方向验证，不进入高增益闭环。
        yMotor.controller = MotionControlType::angle_openloop;
        yMotor.voltage_limit = fminf(kSlaveMotorFoc.motor_voltage_limit_v, 0.4f);
    } else {
        yMotor.linkSensor(&ySensor);
        yMotor.controller = MotionControlType::angle;
    }
    yMotor.P_angle.P = kSlaveMotorFoc.angle_p;
    yMotor.P_angle.limit = kSlaveMotorFoc.velocity_limit_rad_s;
    yMotor.PID_velocity.P = kSlaveMotorFoc.velocity_pid_p;
    yMotor.PID_velocity.I = kSlaveMotorFoc.velocity_pid_i;
    yMotor.PID_velocity.D = kSlaveMotorFoc.velocity_pid_d;
    yMotor.PID_velocity.output_ramp = kSlaveMotorFoc.velocity_pid_ramp;
    yMotor.PID_velocity.limit = kSlaveMotorFoc.motor_voltage_limit_v;
    yMotor.LPF_velocity.Tf = kSlaveMotorFoc.velocity_lpf_tf;
    yMotor.LPF_angle.Tf = kSlaveMotorFoc.angle_lpf_tf;

    yMotor.init();
    if (SLAVE_Y_BRINGUP_MODE == SLAVE_Y_BRINGUP_MOTOR_OPEN_LOOP) {
        yMotorReady = true;
        return true;
    }
    if (SLAVE_Y_BRINGUP_MODE != SLAVE_Y_BRINGUP_CLOSED_LOOP &&
        SLAVE_RUN_MODE != SLAVE_MODE_DUAL_XY_HW &&
        SLAVE_RUN_MODE != SLAVE_MODE_SINGLE_Y_SYNC) {
        yDriver.disable();
        return false;
    }
    if (!yMotor.initFOC()) {
        yDriver.disable();
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }
    yMotorReady = true;
    return true;
#elif SLAVE_Y_SENSOR_HW_ENABLED
    return ySensorReady;
#else
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
#if SLAVE_X_MOTOR_HW_ENABLED
    // 真实硬件路径：本地控制步中更新 FOC 并写入角度目标。
    // 这里不做 Serial、ESP-NOW 或 UV GPIO 操作，保持控制路径干净。
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
    // 硬件关闭时使用运动模块计算的仿真角度，仍能让遥测和 UV 联锁逻辑跑起来。
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
#if SLAVE_X_MOTOR_HW_ENABLED
    if (!xMotorReady) {
        (void)target_angle_rad;
        return fallback_actual_angle_rad;
    }

#if SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_MODE_TIMER_EMPTY
    (void)target_angle_rad;
    return fallback_actual_angle_rad;
#elif SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_MODE_SENSOR_ONLY
    xSensor.update();
    const float actual_angle_rad = xSensor.getMechanicalAngle();
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    if (timing != nullptr) {
        timing->sensor_us = xSensor.lastReadDurationUs();
    }
#endif
    return actual_angle_rad;
#elif SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_MODE_LOOPFOC_ONLY
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
#elif SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_MODE_LOOPFOC_MOVE
    return applySlaveXMotorTarget(target_angle_rad, fallback_actual_angle_rad, timing);
#else
    return applySlaveXMotorTarget(target_angle_rad, fallback_actual_angle_rad, timing);
#endif
#else
    (void)target_angle_rad;
    return fallback_actual_angle_rad;
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

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t foc_start_us = micros();
    uint32_t move_start_us = foc_start_us;
#endif
    if (run_loop_foc && SLAVE_Y_BRINGUP_MODE != SLAVE_Y_BRINGUP_MOTOR_OPEN_LOOP) {
        yMotor.loopFOC();
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
        move_start_us = micros();
#endif
    }
    yMotor.move(target_angle_rad);
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t done_us = micros();
    if (timing != nullptr) {
        timing->loop_foc_us =
            (run_loop_foc && SLAVE_Y_BRINGUP_MODE != SLAVE_Y_BRINGUP_MOTOR_OPEN_LOOP)
                ? (move_start_us - foc_start_us)
                : 0UL;
        timing->move_us = done_us - move_start_us;
        timing->sensor_us =
            (run_loop_foc && SLAVE_Y_BRINGUP_MODE != SLAVE_Y_BRINGUP_MOTOR_OPEN_LOOP)
                ? ySensor.lastReadDurationUs()
                : 0UL;
        timing->loop_foc_ran =
            (run_loop_foc && SLAVE_Y_BRINGUP_MODE != SLAVE_Y_BRINGUP_MOTOR_OPEN_LOOP) ? 1UL : 0UL;
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

bool refreshSlaveYSensorForBringup(float *angle_rad, uint16_t *raw_angle) {
#if SLAVE_Y_SENSOR_HW_ENABLED
    if (SLAVE_Y_BRINGUP_MODE != SLAVE_Y_BRINGUP_SENSOR_ONLY) {
        return false;
    }
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
