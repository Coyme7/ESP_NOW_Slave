#include "slave/control/slave_motion.h"

#include <Arduino.h>
#include <math.h>

#include "common/protocol/protocol_units.h"
#include "common/system_state.h"
#include "slave/comm/slave_transport.h"
#include "slave/config/slave_config.h"
#include "slave/control/slave_axis_controller.h"
#include "slave/control/slave_coordinate_mapper.h"
#include "slave/control/slave_runtime_snapshot.h"
#include "slave/control/slave_trajectory_smoother.h"
#include "slave/hardware/slave_hardware.h"
#include "slave/safety/slave_safety.h"

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
#define SLAVE_TIMING_NOW_US() micros()
#endif

namespace {

struct SlaveLocalControlRuntime {
    float target_x_mm;
    float smooth_x_mm;
    float target_angle_rad;
    float actual_angle_rad;
    float target_y_mm;
    float smooth_y_mm;
    float target_y_angle_rad;
    float actual_y_angle_rad;
    bool x_limit;
    bool y_limit;
    bool x_clamped;
    bool y_clamped;
    bool boundary_hit;
    uint8_t command_valid;
    uint8_t pen_down;
};

struct SlaveAxisPlannerResult {
    float target_mm;
    float smooth_mm;
    float target_angle_rad;
    bool limit;
    bool clamped;
};

SlaveLocalControlRuntime runtime = {};
SlaveMotionSnapshot motionSnapshot = {};
portMUX_TYPE motionSnapshotMux = portMUX_INITIALIZER_UNLOCKED;

constexpr bool controlRunsXAxis() {
    return SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_X_SYNC ||
           SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_FRAME ||
           SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_HW;
}

constexpr bool controlRunsYAxis() {
    return SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_Y_SYNC ||
           SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_FRAME ||
           SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_HW;
}

int16_t selectAxisCommandNorm(AxisId axis, const SlaveRtCommand &command) {
    if (command.valid == 0 || command.mode != MODE_COLLAB_DRAW) {
        return 0;
    }

    switch (SLAVE_RUN_MODE) {
        case SLAVE_MODE_SINGLE_X_SYNC:
            return (axis == AXIS_X) ? command.x_norm : 0;
        case SLAVE_MODE_SINGLE_Y_SYNC:
            return (axis == AXIS_Y) ? command.y_norm : 0;
        case SLAVE_MODE_DUAL_XY_FRAME:
        case SLAVE_MODE_DUAL_XY_HW:
            return (axis == AXIS_X) ? command.x_norm : command.y_norm;
        default:
            return 0;
    }
}

SlaveTrajectorySmootherOutput updateAxisSmoother(SlaveTrajectorySmootherState &state,
                                                 float target_mm,
                                                 float dt_s,
                                                 bool pen_down) {
    const SlaveTrajectorySmootherInput input = {
        target_mm,
        dt_s,
        pen_down ? kSlaveTrajectory.draw_speed_mm_s : kSlaveTrajectory.lift_speed_mm_s,
        kSlaveTrajectory.accel_mm_s2,
        kSlaveTrajectory.command_deadband_mm,
    };
    return updateSlaveTrajectorySmoother(state, input);
}

bool isAxisAtLimit(AxisId axis, float mm, bool clamped) {
    const float half_range_mm = slaveAxisHalfRangeMm(axis);
    return clamped || fabsf(mm) >= (half_range_mm - 0.001f);
}

float simulatedAxisAngle(float actual_angle_rad,
                         float target_angle_rad,
                         const SlaveAxisConfig &config) {
    return actual_angle_rad + ((target_angle_rad - actual_angle_rad) * config.simulated_response_alpha);
}

SlaveRtCommand readPlannerCommandForStep(uint32_t now_us, uint16_t &faults) {
    SlaveRtCommand command = snapshotSlaveRtCommand();
    if (!isSlaveRtCommandFresh(command, now_us)) {
        // 只改 planner 本地副本，不清 SlaveRtCommand。
        // SlaveRtCommand 表示最后一次通过校验的命令，避免 timeout 后旧包绕过 stale gate。
        command.valid = 0;
        command.pen_down = 0;
        faults |= FAULT_COMMAND_TIMEOUT;
    }
    return command;
}

SlaveAxisPlannerResult planAxisTarget(AxisId axis,
                                      bool axis_enabled,
                                      int16_t command_norm,
                                      float dt_s,
                                      bool pen_down,
                                      SlaveTrajectorySmootherState &smoother) {
    SlaveAxisPlannerResult result = {};
    if (!axis_enabled) {
        // 未启用轴不能进入平滑、坐标映射或硬件访问链路。
        return result;
    }

    result.target_mm =
        slaveClampAxisPaperMm(axis, slaveAxisNormToPaperMm(axis, command_norm), &result.clamped);
    result.limit = isAxisAtLimit(axis, result.target_mm, result.clamped);

    const SlaveTrajectorySmootherOutput smooth =
        updateAxisSmoother(smoother, result.target_mm, dt_s, pen_down);
    result.smooth_mm = smooth.position_mm;
    result.target_angle_rad = slaveAxisPaperMmToGimbalAngleRad(axis, result.smooth_mm);
    return result;
}

void updateRuntimeFromPlanner(const SlaveAxisPlannerResult &x_plan,
                              const SlaveAxisPlannerResult &y_plan,
                              const SlaveRtCommand &command) {
    runtime.boundary_hit = x_plan.limit || y_plan.limit;
    runtime.target_x_mm = x_plan.target_mm;
    runtime.smooth_x_mm = x_plan.smooth_mm;
    runtime.target_angle_rad = x_plan.target_angle_rad;
    runtime.target_y_mm = y_plan.target_mm;
    runtime.smooth_y_mm = y_plan.smooth_mm;
    runtime.target_y_angle_rad = y_plan.target_angle_rad;
    runtime.x_limit = x_plan.limit;
    runtime.y_limit = y_plan.limit;
    runtime.x_clamped = x_plan.clamped;
    runtime.y_clamped = y_plan.clamped;
    runtime.command_valid = command.valid;
    runtime.pen_down = command.pen_down;
}

bool shouldPublishRuntimeToSysData() {
#if SLAVE_RUNTIME_PUBLISH_EVERY_N_STEPS <= 1
    return true;
#else
    static uint32_t publish_divider = 0;
    publish_divider++;
    if (publish_divider >= SLAVE_RUNTIME_PUBLISH_EVERY_N_STEPS) {
        publish_divider = 0;
        return true;
    }
    return false;
#endif
}

bool shouldPublishMotionSnapshot() {
#if SLAVE_MOTION_SNAPSHOT_EVERY_N_STEPS <= 1
    return true;
#else
    static uint32_t publish_divider = 0;
    const bool should_publish = publish_divider == 0;
    publish_divider++;
    if (publish_divider >= SLAVE_MOTION_SNAPSHOT_EVERY_N_STEPS) {
        publish_divider = 0;
    }
    return should_publish;
#endif
}

SlaveMotionSnapshot makeMotionSnapshot(const SlaveLocalControlRuntime &state) {
    SlaveMotionSnapshot snapshot = {};
    snapshot.target_x_mm = state.target_x_mm;
    snapshot.smooth_x_mm = state.smooth_x_mm;
    snapshot.target_angle_rad = state.target_angle_rad;
    snapshot.actual_angle_rad = state.actual_angle_rad;
    snapshot.x_track_err_mrad = (state.target_angle_rad - state.actual_angle_rad) * 1000.0f;
    snapshot.target_y_mm = state.target_y_mm;
    snapshot.smooth_y_mm = state.smooth_y_mm;
    snapshot.target_y_angle_rad = state.target_y_angle_rad;
    snapshot.actual_y_angle_rad = state.actual_y_angle_rad;
    snapshot.y_track_err_mrad = (state.target_y_angle_rad - state.actual_y_angle_rad) * 1000.0f;
    snapshot.x_limit = state.x_limit;
    snapshot.y_limit = state.y_limit;
    snapshot.x_clamped = state.x_clamped;
    snapshot.y_clamped = state.y_clamped;
    snapshot.boundary_hit = state.boundary_hit;
    snapshot.command_valid = state.command_valid;
    snapshot.pen_down = state.pen_down;
    return snapshot;
}

void publishMotionSnapshot(const SlaveLocalControlRuntime &state) {
    const SlaveMotionSnapshot snapshot = makeMotionSnapshot(state);
    portENTER_CRITICAL(&motionSnapshotMux);
    motionSnapshot = snapshot;
    portEXIT_CRITICAL(&motionSnapshotMux);
}

void publishRuntimeToSysData(const SlaveLocalControlRuntime &state) {
    // sysData 只服务低频显示和历史兼容；安全和遥测读取 SlaveMotionSnapshot。
    sysData.slave.boundary_hit = state.boundary_hit;
    sysData.slave.target_x_mm = state.target_x_mm;
    sysData.slave.smooth_x_mm = state.smooth_x_mm;
    sysData.slave.target_angle_rad = state.target_angle_rad;
    sysData.slave.actual_angle_rad = state.actual_angle_rad;
    sysData.slave.target_y_mm = state.target_y_mm;
    sysData.slave.smooth_y_mm = state.smooth_y_mm;
    sysData.slave.target_y_angle_rad = state.target_y_angle_rad;
    sysData.slave.actual_y_angle_rad = state.actual_y_angle_rad;
    sysData.slave.x_limit_hit = state.x_limit;
    sysData.slave.y_limit_hit = state.y_limit;
    sysData.slave.x_clamped = state.x_clamped;
    sysData.slave.y_clamped = state.y_clamped;
    sysData.slave.angle_deg = radToDeg(state.actual_angle_rad);
    sysData.slave.x_pos =
        normToPercent(slaveAxisGimbalAngleRadToNorm(AXIS_X, state.actual_angle_rad));
    sysData.slave.y_pos =
        normToPercent(slaveAxisGimbalAngleRadToNorm(AXIS_Y, state.actual_y_angle_rad));
}

void publishControlStateIfDue(const SlaveLocalControlRuntime &state) {
    // 快照供 safety/telemetry/status 使用；sysData 只保留低频显示和历史兼容字段。
    if (shouldPublishMotionSnapshot()) {
        publishMotionSnapshot(state);
    }
    if (shouldPublishRuntimeToSysData()) {
        publishRuntimeToSysData(state);
    }
}

}  // namespace

// 将协议 x_norm 转成纸面毫米坐标，范围为 -125..+125 mm。
float xNormToPaperMm(int16_t x_norm) {
    return slaveXNormToPaperMm(x_norm);
}

// 将纸面 X 毫米位置转成云台 X 轴目标角。
float paperMmToGimbalAngleRad(float x_mm) {
    return slavePaperMmToGimbalAngleRad(x_mm);
}

// 将云台实际角反推回协议归一化坐标，供遥测 x_actual_norm 使用。
int16_t gimbalAngleRadToXNorm(float angle_rad) {
    return slaveGimbalAngleRadToXNorm(angle_rad);
}

SlaveMotionSnapshot snapshotSlaveMotion() {
    SlaveMotionSnapshot snapshot = {};
    portENTER_CRITICAL(&motionSnapshotMux);
    snapshot = motionSnapshot;
    portEXIT_CRITICAL(&motionSnapshotMux);
    return snapshot;
}

void runSlaveControlPerfIsolationStep(float dt_s, SlaveControlStepTiming *timing) {
    (void)dt_s;
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    if (timing != nullptr) {
        *timing = {};
    }
#else
    (void)timing;
#endif

#if SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_MODE_TIMER_EMPTY
    return;
#else
    const float target_angle_rad = kSlaveXAxis.center_angle_rad;
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    SlaveXMotorStepTiming motor_timing = {};
    const uint32_t motor_start_us = SLAVE_TIMING_NOW_US();
    runtime.actual_angle_rad =
        runSlaveXMotorPerfIsolationStep(target_angle_rad, runtime.actual_angle_rad, &motor_timing);
    const uint32_t motor_done_us = SLAVE_TIMING_NOW_US();
#else
    runtime.actual_angle_rad =
        runSlaveXMotorPerfIsolationStep(target_angle_rad, runtime.actual_angle_rad, nullptr);
#endif

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    if (timing != nullptr) {
        timing->motor_us = motor_done_us - motor_start_us;
        timing->x_sensor_us = motor_timing.sensor_us;
        timing->x_foc_us = motor_timing.loop_foc_us;
        timing->x_foc_ran = motor_timing.loop_foc_ran;
        timing->x_move_us = motor_timing.move_us;
        timing->state_us = 0;
    }
#endif
#endif
}

void runSlaveControlPerfIsolationStep(float dt_s) {
    runSlaveControlPerfIsolationStep(dt_s, nullptr);
}

void runSlavePlannerStep(float dt_s, SlaveControlStepTiming *timing) {
    static SlaveTrajectorySmootherState x_smoother = {};
    static SlaveTrajectorySmootherState y_smoother = {};

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t command_start_us = SLAVE_TIMING_NOW_US();
#else
    (void)timing;
#endif
    const uint32_t now_us = micros();
    uint16_t faults = FAULT_NONE;

    const SlaveRtCommand command = readPlannerCommandForStep(now_us, faults);

    constexpr bool x_axis_enabled = controlRunsXAxis();
    constexpr bool y_axis_enabled = controlRunsYAxis();
    const int16_t x_norm = x_axis_enabled ? selectAxisCommandNorm(AXIS_X, command) : 0;
    const int16_t y_norm = y_axis_enabled ? selectAxisCommandNorm(AXIS_Y, command) : 0;

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t trajectory_start_us = SLAVE_TIMING_NOW_US();
#endif
    const bool pen_down = command.pen_down != 0;
    const SlaveAxisPlannerResult x_plan =
        planAxisTarget(AXIS_X, x_axis_enabled, x_norm, dt_s, pen_down, x_smoother);
    const SlaveAxisPlannerResult y_plan =
        planAxisTarget(AXIS_Y, y_axis_enabled, y_norm, dt_s, pen_down, y_smoother);
    if (x_plan.limit || y_plan.limit) {
        faults |= FAULT_BOUNDARY_HIT;
    }
    updateRuntimeFromPlanner(x_plan, y_plan, command);

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t publish_start_us = SLAVE_TIMING_NOW_US();
#endif
    const uint16_t runtime_faults =
        faults |
        ((SLAVE_UV_HW_ENABLED && sysData.slave.uv_interlock_blocked) ? FAULT_UV_INTERLOCK : FAULT_NONE);
    static uint16_t last_runtime_faults = 0xffffU;
    if (runtime_faults != last_runtime_faults) {
        publishProtocolFaults(runtime_faults);
        last_runtime_faults = runtime_faults;
    }

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t done_us = SLAVE_TIMING_NOW_US();
    if (timing != nullptr) {
        timing->command_us = trajectory_start_us - command_start_us;
        timing->trajectory_us = publish_start_us - trajectory_start_us;
        timing->publish_us = done_us - publish_start_us;
    }
#endif
}

void runSlaveMotorStep(SlaveControlStepTiming *timing) {
    constexpr bool x_axis_enabled = controlRunsXAxis();
    constexpr bool y_axis_enabled = controlRunsYAxis();

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t motor_start_us = SLAVE_TIMING_NOW_US();
#endif
    const float x_fallback_actual =
        x_axis_enabled
            ? simulatedAxisAngle(runtime.actual_angle_rad, runtime.target_angle_rad, kSlaveXAxis)
            : 0.0f;
#if SLAVE_Y_SIMULATION_ENABLED
    const float y_fallback_actual =
        y_axis_enabled
            ? simulatedAxisAngle(runtime.actual_y_angle_rad, runtime.target_y_angle_rad, kSlaveYAxis)
            : 0.0f;
#else
    const float y_fallback_actual = 0.0f;
#endif

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    SlaveXMotorStepTiming x_motor_timing = {};
    SlaveXMotorStepTiming y_motor_timing = {};
    const float x_actual_angle_rad =
        x_axis_enabled ? applySlaveXMotorTarget(runtime.target_angle_rad, x_fallback_actual, &x_motor_timing)
                       : x_fallback_actual;
    const float y_actual_angle_rad =
        y_axis_enabled ? applySlaveYMotorTarget(runtime.target_y_angle_rad, y_fallback_actual, &y_motor_timing)
                       : y_fallback_actual;
#else
    const float x_actual_angle_rad =
        x_axis_enabled ? applySlaveXMotorTarget(runtime.target_angle_rad, x_fallback_actual, nullptr)
                       : x_fallback_actual;
    const float y_actual_angle_rad =
        y_axis_enabled ? applySlaveYMotorTarget(runtime.target_y_angle_rad, y_fallback_actual, nullptr)
                       : y_fallback_actual;
#endif

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t state_start_us = SLAVE_TIMING_NOW_US();
#endif
    runtime.actual_angle_rad = x_actual_angle_rad;
    runtime.actual_y_angle_rad = y_actual_angle_rad;
    publishControlStateIfDue(runtime);

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t done_us = SLAVE_TIMING_NOW_US();
    if (timing != nullptr) {
        timing->motor_us = state_start_us - motor_start_us;
        timing->x_sensor_us = x_motor_timing.sensor_us;
        timing->x_foc_us = x_motor_timing.loop_foc_us;
        timing->x_foc_ran = x_motor_timing.loop_foc_ran;
        timing->x_move_us = x_motor_timing.move_us;
        timing->y_sensor_us = y_motor_timing.sensor_us;
        timing->y_foc_us = y_motor_timing.loop_foc_us;
        timing->y_foc_ran = y_motor_timing.loop_foc_ran;
        timing->y_move_us = y_motor_timing.move_us;
        timing->state_us = done_us - state_start_us;
    }
#else
    (void)timing;
#endif
}

void runSlaveControlStep(float dt_s, SlaveControlStepTiming *timing) {
#if SLAVE_CONTROL_PERF_MODE != SLAVE_PERF_MODE_FULL_CONTROL
    runSlaveControlPerfIsolationStep(dt_s, timing);
    return;
#endif

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    if (timing != nullptr) {
        *timing = {};
    }
#else
    (void)timing;
#endif

    static uint32_t planner_divider = 0;
    static float planner_dt_accum_s = 0.0f;
    planner_dt_accum_s += dt_s;

    const bool run_planner = planner_divider == 0;
    planner_divider++;
    if (planner_divider >= SLAVE_PLANNER_EVERY_N_STEPS) {
        planner_divider = 0;
    }

    if (run_planner) {
        runSlavePlannerStep(planner_dt_accum_s, timing);
        planner_dt_accum_s = 0.0f;
    }
    runSlaveMotorStep(timing);
}

void runSlaveControlStep(float dt_s) {
    runSlaveControlStep(dt_s, nullptr);
}
