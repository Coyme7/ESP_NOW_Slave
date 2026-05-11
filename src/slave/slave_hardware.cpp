#include "slave/slave_hardware.h"

#include <Arduino.h>
#include <board/board_pins_slave.h>

#include "common/mt6701_ssi_sensor.h"
#include "common/system_state.h"
#include "slave/slave_config.h"

#if SLAVE_X_MOTOR_HW_ENABLED
#include <SimpleFOC.h>
#endif

// 从机硬件适配层。
// 这里集中处理 UV MOS、SimpleFOC X 轴对象和安全输出，运动算法不直接碰板级引脚。
// Y 轴当前只保持禁用状态，后续接入时应按同样方式独立封装。

namespace {

#if SLAVE_X_MOTOR_HW_ENABLED
bool xMotorReady = false;

#ifndef SLAVE_X_MOTOR_POLE_PAIRS
// 当前 2804 临时测试默认值，沿用主机侧 2804 配置。
// 后续接回 2208 云台时，请用 build_flags 覆盖或改为实测 2208 极对数。
#define SLAVE_X_MOTOR_POLE_PAIRS 7
#endif

BLDCMotor xMotor = BLDCMotor(SLAVE_X_MOTOR_POLE_PAIRS);
BLDCDriver3PWM xDriver = BLDCDriver3PWM(
    board_pins_slave::MOTOR1_PWM_U_X,
    board_pins_slave::MOTOR1_PWM_V_X,
    board_pins_slave::MOTOR1_PWM_W_X,
    board_pins_slave::MOTOR1_EN_X);
Mt6701SsiSensor xSensor = Mt6701SsiSensor(board_pins_slave::ENCODER1_CS_X);
#endif

}  // namespace

void setUvPen(bool enabled) {
    // 记录上一次输出状态，避免安全任务 1 kHz 重复写同样的 GPIO。
    // 即便输出未变化，也同步 sysData.pen_down，保证串口状态反映真实 UV 输出。
    static bool last_enabled = false;
    static bool initialized = false;
    if (initialized && enabled == last_enabled) {
        sysData.pen_down = enabled;
        return;
    }

    // 两路 MOS 同步控制。当前设计中 UV 默认关闭，只有安全联锁允许时才拉高。
    digitalWrite(board_pins_slave::UV_MOS_1, enabled ? HIGH : LOW);
    digitalWrite(board_pins_slave::UV_MOS_2, enabled ? HIGH : LOW);
    sysData.pen_down = enabled;
    last_enabled = enabled;
    initialized = true;
}

void configureSlaveSafeOutputs() {
    // 启动安全：Wi-Fi 和 RTOS 任务启动前，先关闭紫光和两路电机使能。
    pinMode(board_pins_slave::UV_MOS_1, OUTPUT);
    pinMode(board_pins_slave::UV_MOS_2, OUTPUT);
    setUvPen(false);

    pinMode(board_pins_slave::MOTOR1_EN_X, OUTPUT);
    pinMode(board_pins_slave::MOTOR2_EN_Y, OUTPUT);
    digitalWrite(board_pins_slave::MOTOR1_EN_X, LOW);
    digitalWrite(board_pins_slave::MOTOR2_EN_Y, LOW);
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

float applySlaveXMotorTarget(float target_angle_rad, float fallback_actual_angle_rad) {
#if SLAVE_X_MOTOR_HW_ENABLED
    // 真实硬件路径：10 kHz 控制步中更新 FOC 并写入角度目标。
    // 这里不做 Serial、ESP-NOW 或 UV GPIO 操作，保持控制路径干净。
    if (!xMotorReady) {
        (void)target_angle_rad;
        return fallback_actual_angle_rad;
    }
    xMotor.loopFOC();
    xMotor.move(target_angle_rad);
    return xMotor.shaft_angle;
#else
    // 硬件关闭时使用运动模块计算的仿真角度，仍能让遥测和 UV 联锁逻辑跑起来。
    (void)target_angle_rad;
    return fallback_actual_angle_rad;
#endif
}
