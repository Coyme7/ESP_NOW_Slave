#pragma once

#include "common/protocol/protocol_types.h"
#include "common/protocol/protocol_units.h"
#include "slave/modes/mode_traits.h"

// 从机配置非法组合检查。
// 普通 config 文件不散落 static_assert，便于集中审计 bring-up 风险。
static_assert(COMMON_CONTRACT_VERSION == 1, "common contract version mismatch");
static_assert(sizeof(MasterCommandPacket) == 24, "MasterCommandPacket layout drifted");
static_assert(sizeof(TrajectorySegmentPacket) == 32, "TrajectorySegmentPacket layout drifted");
static_assert(sizeof(SlaveTelemetryPacket) == 44, "SlaveTelemetryPacket layout drifted");
static_assert(sizeof(MasterCommandPacket) <= 250, "ESP-NOW v1 payload limit exceeded");
static_assert(sizeof(TrajectorySegmentPacket) <= 250, "ESP-NOW v1 payload limit exceeded");
static_assert(sizeof(SlaveTelemetryPacket) <= 250, "ESP-NOW v1 payload limit exceeded");

static_assert(SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_X_5KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_Y_5KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_2KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_DRY_RUN_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_YSENSOR_ONLY_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_Y_OPEN_LOOP_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_Y_CLOSED_LOOP_ID,
              "invalid SLAVE_RUN_MODE");

static_assert(SLAVE_STARTUP_APP_MODE == SLAVE_STARTUP_APP_MANUAL_DRAW_ID ||
                  SLAVE_STARTUP_APP_MODE == SLAVE_STARTUP_APP_AUTO_DRAW_ID ||
                  SLAVE_STARTUP_APP_MODE == SLAVE_STARTUP_APP_BLE_SAFE_ID ||
                  SLAVE_STARTUP_APP_MODE == SLAVE_STARTUP_APP_DIAGNOSTICS_ID,
              "invalid SLAVE_STARTUP_APP_MODE");

static_assert(SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_TIMER_EMPTY ||
                  SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_SENSOR_ONLY ||
                  SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_LOOPFOC_ONLY ||
                  SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_MOVE_ONLY ||
                  SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_LOOPFOC_MOVE ||
                  SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_FULL_CONTROL,
              "invalid SLAVE_CONTROL_PERF_MODE");

static_assert(SLAVE_CONTROL_LOOP_PERIOD_US > 0,
              "SLAVE_CONTROL_LOOP_PERIOD_US must be greater than 0");
static_assert((1000UL % SLAVE_CONTROL_LOOP_PERIOD_US) == 0,
              "SLAVE_CONTROL_LOOP_PERIOD_US must divide 1000us FreeRTOS tick");
static_assert(SLAVE_CONTROL_LOOP_PERIOD_US == slaveRunModeNominalPeriodUs(),
              "SLAVE_RUN_MODE frequency must match SLAVE_CONTROL_LOOP_PERIOD_US");

static_assert(SLAVE_PLANNER_EVERY_N_STEPS > 0,
              "SLAVE_PLANNER_EVERY_N_STEPS must be greater than 0");
static_assert(SLAVE_RUNTIME_PUBLISH_EVERY_N_STEPS > 0,
              "SLAVE_RUNTIME_PUBLISH_EVERY_N_STEPS must be greater than 0");
static_assert(SLAVE_MOTION_SNAPSHOT_EVERY_N_STEPS > 0,
              "SLAVE_MOTION_SNAPSHOT_EVERY_N_STEPS must be greater than 0");
static_assert(SLAVE_X_FOC_EVERY_N_STEPS > 0,
              "SLAVE_X_FOC_EVERY_N_STEPS must be greater than 0");
static_assert(SLAVE_X_MOVE_EVERY_N_STEPS > 0,
              "SLAVE_X_MOVE_EVERY_N_STEPS must be greater than 0");
static_assert(SLAVE_Y_FOC_EVERY_N_STEPS > 0,
              "SLAVE_Y_FOC_EVERY_N_STEPS must be greater than 0");
static_assert(SLAVE_Y_MOVE_EVERY_N_STEPS > 0,
              "SLAVE_Y_MOVE_EVERY_N_STEPS must be greater than 0");

static_assert(!(SLAVE_AUTO_DRAW_ENABLED && !SLAVE_ESPNOW_ENABLED),
              "SLAVE_AUTO_DRAW_ENABLED requires SLAVE_ESPNOW_ENABLED");
static_assert(!(slaveRunModeNeedsMotorHardware(AXIS_X) &&
                SLAVE_X_MOTOR_HW_ENABLED &&
                !SLAVE_X_SENSOR_HW_ENABLED),
              "X motor output requires X sensor hardware");
static_assert(!(slaveRunModeNeedsMotorHardware(AXIS_Y) &&
                !slaveRunModeUsesOpenLoopMotor(AXIS_Y) &&
                SLAVE_Y_MOTOR_HW_ENABLED &&
                !SLAVE_Y_SENSOR_HW_ENABLED),
              "Y closed-loop motor output requires Y sensor hardware");

static_assert(!(SLAVE_UV_HW_ENABLED && SLAVE_RUN_MODE != SLAVE_MODE_DUAL_XY_2KHZ_ID),
              "SLAVE_UV_HW_ENABLED requires SLAVE_MODE_DUAL_XY_2KHZ");
static_assert(!(SLAVE_UV_HW_ENABLED && !SLAVE_DUAL_XY_HARDWARE_ENABLED),
              "SLAVE_UV_HW_ENABLED requires SLAVE_DUAL_XY_HARDWARE_ENABLED");
static_assert(!(SLAVE_UV_HW_ENABLED && !SLAVE_ESPNOW_ENABLED),
              "SLAVE_UV_HW_ENABLED requires SLAVE_ESPNOW_ENABLED");
static_assert(!(SLAVE_UV_HW_ENABLED && !SLAVE_AUTO_DRAW_ENABLED),
              "SLAVE_UV_HW_ENABLED requires SLAVE_AUTO_DRAW_ENABLED");

static_assert(SLAVE_FAST_SENSOR_READER_ENABLED,
              "Slave control hot path requires MT6701 fast reader");
static_assert(SLAVE_ESPNOW_CHANNEL >= 1 && SLAVE_ESPNOW_CHANNEL <= 14,
              "SLAVE_ESPNOW_CHANNEL must be in 1..14");
static_assert(SLAVE_STATUS_LOOP_PERIOD_MS > 0,
              "SLAVE_STATUS_LOOP_PERIOD_MS must be greater than 0");
static_assert(SLAVE_TIMING_DIAG_LEVEL >= 0 && SLAVE_TIMING_DIAG_LEVEL <= 2,
              "SLAVE_TIMING_DIAG_LEVEL must be 0, 1, or 2");

static_assert(PLOT_X_HALF_RANGE_MM > 0.0f && PLOT_Y_HALF_RANGE_MM > 0.0f,
              "plot half range must be greater than 0");
static_assert(DEFAULT_THROW_DISTANCE_MM > 0.0f,
              "DEFAULT_THROW_DISTANCE_MM must be greater than 0");
static_assert(A4_WIDTH_MM == A4_PORTRAIT_WIDTH_MM &&
                  A4_HEIGHT_MM == A4_PORTRAIT_HEIGHT_MM,
              "default A4 paper geometry must be portrait");
static_assert(A4_WIDTH_MM > 0.0f && A4_HEIGHT_MM > 0.0f,
              "A4 paper geometry must be greater than 0");

static_assert(kSlaveXAxisLimitMinMmConfig >= -PLOT_X_HALF_RANGE_MM &&
                  kSlaveXAxisLimitMaxMmConfig <= PLOT_X_HALF_RANGE_MM &&
                  kSlaveXAxisLimitMinMmConfig < kSlaveXAxisLimitMaxMmConfig,
              "X axis software limits must stay inside X paper half range");
static_assert(kSlaveYAxisLimitMinMmConfig >= -PLOT_Y_HALF_RANGE_MM &&
                  kSlaveYAxisLimitMaxMmConfig <= PLOT_Y_HALF_RANGE_MM &&
                  kSlaveYAxisLimitMinMmConfig < kSlaveYAxisLimitMaxMmConfig,
              "Y axis software limits must stay inside Y paper half range");
static_assert(kSlaveXSettleErrorMmConfig >= 0.0f &&
                  kSlaveXSettleErrorMmConfig <= PLOT_X_HALF_RANGE_MM,
              "kSlaveXSettleErrorMmConfig must stay inside X half range");
static_assert(kSlaveYSettleErrorMmConfig >= 0.0f &&
                  kSlaveYSettleErrorMmConfig <= PLOT_Y_HALF_RANGE_MM,
              "kSlaveYSettleErrorMmConfig must stay inside Y half range");

constexpr bool slaveConfigDirectionSignValid(int8_t sign) {
    return sign == -1 || sign == 1;
}

constexpr bool slaveConfigPaperSignValid(float sign) {
    return sign == -1.0f || sign == 1.0f;
}

constexpr bool slaveConfigMotorFocValid(const SlaveMotorFocConfig &config) {
    return config.voltage.supply_v > 0.0f &&
           config.voltage.driver_limit_v > 0.0f &&
           config.voltage.driver_limit_v <= config.voltage.supply_v &&
           config.voltage.motor_limit_v > 0.0f &&
           config.voltage.motor_limit_v <= config.voltage.driver_limit_v &&
           config.voltage.open_loop_limit_v > 0.0f &&
           config.voltage.open_loop_limit_v <= config.voltage.motor_limit_v &&
           config.voltage.align_v > 0.0f &&
           config.voltage.align_v <= config.voltage.driver_limit_v &&
           config.limit.velocity_rad_s > 0.0f &&
           config.position.p >= 0.0f &&
           config.velocity.p >= 0.0f &&
           config.velocity.i >= 0.0f &&
           config.velocity.d >= 0.0f &&
           config.velocity.output_ramp >= 0.0f &&
           config.filter.velocity_tf >= 0.0f &&
           config.filter.angle_tf >= 0.0f;
}

constexpr bool slaveConfigAxisValid(const SlaveAxisConfig &config) {
    return slaveConfigDirectionSignValid(config.geometry.direction_sign) &&
           config.geometry.throw_distance_mm > 0.0f &&
           config.geometry.half_range_mm > 0.0f &&
           config.tracking.settle_error_rad >= 0.0f &&
           config.tracking.simulated_response_alpha >= 0.0f &&
           config.tracking.simulated_response_alpha <= 1.0f;
}

constexpr bool slaveConfigAxisLimitValid(const SlaveAxisLimitConfig &limit,
                                         const SlaveAxisConfig &axis) {
    return limit.min_mm >= -axis.geometry.half_range_mm &&
           limit.max_mm <= axis.geometry.half_range_mm &&
           limit.min_mm < limit.max_mm;
}

constexpr bool slaveConfigPaperGeometryValid(const PaperGeometry &geometry) {
    return geometry.width_mm > 0.0f &&
           geometry.height_mm > 0.0f &&
           geometry.distance_mm > 0.0f &&
           slaveConfigPaperSignValid(geometry.x_sign) &&
           slaveConfigPaperSignValid(geometry.y_sign);
}

constexpr bool slaveConfigTrajectoryValid(const SlaveTrajectoryConfig &config) {
    return config.draw_speed_mm_s > 0.0f &&
           config.lift_speed_mm_s > 0.0f &&
           config.accel_mm_s2 > 0.0f &&
           config.command_deadband_mm >= 0.0f &&
           config.command_deadband_mm <= PLOT_X_HALF_RANGE_MM &&
           config.command_deadband_mm <= PLOT_Y_HALF_RANGE_MM;
}

constexpr bool slaveConfigAxisTrajectoryValid(const SlaveAxisTrajectoryConfig &config,
                                              float half_range_mm) {
    return config.settle_error_mm >= 0.0f &&
           config.settle_error_mm <= half_range_mm;
}

static_assert(slaveConfigMotorFocValid(kSlaveXMotorFoc),
              "kSlaveXMotorFoc has invalid voltage, PID, or filter values");
static_assert(slaveConfigMotorFocValid(kSlaveYMotorFoc),
              "kSlaveYMotorFoc has invalid voltage, PID, or filter values");

static_assert(slaveConfigAxisValid(kSlaveXAxis),
              "kSlaveXAxis has invalid geometry or tracking values");
static_assert(slaveConfigAxisValid(kSlaveYAxis),
              "kSlaveYAxis has invalid geometry or tracking values");
static_assert(slaveConfigAxisLimitValid(kSlaveXAxisLimit, kSlaveXAxis),
              "kSlaveXAxisLimit must stay inside X paper half range");
static_assert(slaveConfigAxisLimitValid(kSlaveYAxisLimit, kSlaveYAxis),
              "kSlaveYAxisLimit must stay inside Y paper half range");

static_assert(slaveConfigPaperGeometryValid(kSlavePaperGeometry),
              "kSlavePaperGeometry has invalid size, distance, or direction");

static_assert(slaveConfigTrajectoryValid(kSlaveTrajectory),
              "kSlaveTrajectory has invalid speed, acceleration, or deadband");
static_assert(slaveConfigAxisTrajectoryValid(kSlaveXTrajectory, PLOT_X_HALF_RANGE_MM),
              "kSlaveXTrajectory settle error is invalid");
static_assert(slaveConfigAxisTrajectoryValid(kSlaveYTrajectory, PLOT_Y_HALF_RANGE_MM),
              "kSlaveYTrajectory settle error is invalid");
