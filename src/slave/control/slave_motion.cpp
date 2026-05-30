#include "slave/control/slave_motion.h"

#include <Arduino.h>
#include <math.h>

#include "common/protocol/protocol_types.h"
#include "common/protocol/protocol_units.h"
#include "common/system_state.h"
#include "slave/comm/slave_transport.h"
#include "slave/config/slave_config.h"
#include "slave/control/slave_auto_draw_runtime.h"
#include "slave/control/slave_axis_controller.h"
#include "slave/control/slave_coordinate_mapper.h"
#include "slave/control/slave_motion_state.h"
#include "slave/control/slave_trajectory_smoother.h"
#include "slave/hardware/slave_hardware.h"
#include "slave/modes/mode_guard.h"
#include "slave/safety/slave_safety.h"

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
#define SLAVE_TIMING_NOW_US() micros()
#endif

namespace {

struct SlaveAxisPlannerResult {
    float target_mm;
    float smooth_mm;
    float target_angle_rad;
    bool limit;
    bool clamped;
};

SlaveLocalControlRuntime runtime = {};

int16_t selectAxisCommandNorm(AxisId axis, const SlaveRtCommand &command) {
    if (command.valid == 0 || !slaveProtocolModeAllowsRemoteTarget(command.mode)) {
        return 0;
    }

    return (axis == AXIS_X) ? command.x_norm : command.y_norm;
}

SlaveTrajectorySmootherOutput updateAxisSmoother(AxisId axis,
                                                 SlaveTrajectorySmootherState &state,
                                                 float target_mm,
                                                 float dt_s,
                                                 bool pen_req) {
    const SlaveTrajectorySmootherInput input = {
        target_mm,
        slaveAxisLimitMinMm(axis),
        slaveAxisLimitMaxMm(axis),
        dt_s,
        pen_req ? kSlaveTrajectory.draw_speed_mm_s : kSlaveTrajectory.lift_speed_mm_s,
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
    return actual_angle_rad + ((target_angle_rad - actual_angle_rad) * config.tracking.simulated_response_alpha);
}

SlaveRtCommand readPlannerCommandForStep(uint32_t now_us, uint16_t &faults) {
    SlaveRtCommand command = snapshotSlaveRtCommand();
    if (!isSlaveRtCommandFresh(command, now_us)) {
        // 只改 planner 本地副本，不清 SlaveRtCommand。
        // SlaveRtCommand 表示最后一次通过校验的命令，避免 timeout 后旧包绕过 stale gate。
        command.valid = 0;
        command.pen_req = 0;
        faults |= FAULT_COMMAND_TIMEOUT;
    }
    return command;
}

SlaveAxisPlannerResult planAxisTarget(AxisId axis,
                                      bool axis_enabled,
                                      int16_t command_norm,
                                      float dt_s,
                                      bool pen_req,
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
        updateAxisSmoother(axis, smoother, result.target_mm, dt_s, pen_req);
    result.smooth_mm = smooth.position_mm;
    result.target_angle_rad = slaveLimitedAxisPaperMmToGimbalAngleRad(axis, result.smooth_mm);
    return result;
}

#if SLAVE_AUTO_DRAW_ENABLED
SlaveAxisPlannerResult planAxisPaperTarget(AxisId axis,
                                           bool axis_enabled,
                                           float target_mm,
                                           float dt_s,
                                           bool pen_req,
                                           SlaveTrajectorySmootherState &smoother) {
    SlaveAxisPlannerResult result = {};
    if (!axis_enabled) {
        return result;
    }

    result.target_mm = slaveClampAxisPaperMm(axis, target_mm, &result.clamped);
    result.limit = isAxisAtLimit(axis, result.target_mm, result.clamped);

    const SlaveTrajectorySmootherOutput smooth =
        updateAxisSmoother(axis, smoother, result.target_mm, dt_s, pen_req);
    result.smooth_mm = smooth.position_mm;
    result.target_angle_rad = slaveLimitedAxisPaperMmToGimbalAngleRad(axis, result.smooth_mm);
    return result;
}
#endif

void updateRuntimeFromPlanner(const SlaveAxisPlannerResult &x_plan,
                              const SlaveAxisPlannerResult &y_plan,
                              const SlaveRtCommand &command,
                              bool effective_pen_req,
                              uint8_t draw_state,
                              uint8_t draw_progress_pct,
                              uint16_t trajectory_task_id,
                              uint8_t trajectory_segment_count,
                              uint8_t trajectory_segment_cursor,
                              uint8_t trajectory_received_count,
                              uint8_t trajectory_status_flags,
                              uint32_t trajectory_received_mask_low,
                              uint16_t trajectory_received_mask_high) {
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
    runtime.pen_req = effective_pen_req ? 1U : 0U;
    runtime.draw_state = draw_state;
    runtime.draw_progress_pct = draw_progress_pct;
    runtime.trajectory_task_id = trajectory_task_id;
    runtime.trajectory_segment_count = trajectory_segment_count;
    runtime.trajectory_segment_cursor = trajectory_segment_cursor;
    runtime.trajectory_received_count = trajectory_received_count;
    runtime.trajectory_status_flags = trajectory_status_flags;
    runtime.trajectory_received_mask_low = trajectory_received_mask_low;
    runtime.trajectory_received_mask_high = trajectory_received_mask_high;
}

}  // namespace

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
    const float target_angle_rad = kSlaveXAxis.geometry.center_angle_rad;
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

    constexpr bool x_axis_enabled = slaveRunModeRunsAxis(AXIS_X);
    constexpr bool y_axis_enabled = slaveRunModeRunsAxis(AXIS_Y);
    const int16_t x_norm = x_axis_enabled ? selectAxisCommandNorm(AXIS_X, command) : 0;
    const int16_t y_norm = y_axis_enabled ? selectAxisCommandNorm(AXIS_Y, command) : 0;

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t trajectory_start_us = SLAVE_TIMING_NOW_US();
#endif
    bool pen_req = command.pen_req != 0;
    uint8_t draw_state = DRAW_STATE_IDLE;
    uint8_t draw_progress_pct = 0U;
    uint16_t trajectory_task_id = 0U;
    uint8_t trajectory_segment_count = 0U;
    uint8_t trajectory_segment_cursor = 0U;
    uint8_t trajectory_received_count = 0U;
    uint8_t trajectory_status_flags = TRAJECTORY_STATUS_NONE;
    uint32_t trajectory_received_mask_low = 0U;
    uint16_t trajectory_received_mask_high = 0U;
    SlaveAxisPlannerResult x_plan = {};
    SlaveAxisPlannerResult y_plan = {};
#if SLAVE_AUTO_DRAW_ENABLED
    const bool trajectory_requested = slaveCommandRequestsTrajectory(command);
    const SlaveAutoDrawRuntimeOutput draw = updateSlaveAutoDrawRuntime(trajectory_requested, dt_s);
    if (trajectory_requested || draw.draw_state != DRAW_STATE_IDLE) {
        pen_req = draw.pen_req;
        draw_state = draw.draw_state;
        draw_progress_pct = draw.progress_pct;
        trajectory_task_id = draw.trajectory_task_id;
        trajectory_segment_count = draw.trajectory_segment_count;
        trajectory_segment_cursor = draw.trajectory_segment_cursor;
        trajectory_received_count = draw.trajectory_received_count;
        trajectory_status_flags = draw.trajectory_status_flags;
        trajectory_received_mask_low = draw.trajectory_received_mask_low;
        trajectory_received_mask_high = draw.trajectory_received_mask_high;
        const float draw_x_mm = draw.has_paper_target ? draw.x_mm : runtime.target_x_mm;
        const float draw_y_mm = draw.has_paper_target ? draw.y_mm : runtime.target_y_mm;
        x_plan = planAxisPaperTarget(AXIS_X, x_axis_enabled, draw_x_mm, dt_s, pen_req, x_smoother);
        y_plan = planAxisPaperTarget(AXIS_Y, y_axis_enabled, draw_y_mm, dt_s, pen_req, y_smoother);
    } else
#endif
    {
        x_plan = planAxisTarget(AXIS_X, x_axis_enabled, x_norm, dt_s, pen_req, x_smoother);
        y_plan = planAxisTarget(AXIS_Y, y_axis_enabled, y_norm, dt_s, pen_req, y_smoother);
    }
    if (x_plan.limit || y_plan.limit) {
        faults |= FAULT_BOUNDARY_HIT;
    }
    updateRuntimeFromPlanner(x_plan,
                             y_plan,
                             command,
                             pen_req,
                             draw_state,
                             draw_progress_pct,
                             trajectory_task_id,
                             trajectory_segment_count,
                             trajectory_segment_cursor,
                             trajectory_received_count,
                             trajectory_status_flags,
                             trajectory_received_mask_low,
                             trajectory_received_mask_high);

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
    constexpr bool x_axis_enabled = slaveRunModeRunsAxis(AXIS_X);
    constexpr bool y_axis_enabled = slaveRunModeRunsAxis(AXIS_Y);
    constexpr bool x_motor_enabled = slaveRunModeDrivesAxis(AXIS_X);
    constexpr bool y_motor_enabled = slaveRunModeDrivesAxis(AXIS_Y);

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
        x_motor_enabled ? applySlaveXMotorTarget(runtime.target_angle_rad, x_fallback_actual, &x_motor_timing)
                        : x_fallback_actual;
    const float y_actual_angle_rad =
        y_motor_enabled ? applySlaveYMotorTarget(runtime.target_y_angle_rad, y_fallback_actual, &y_motor_timing)
                        : y_fallback_actual;
#else
    const float x_actual_angle_rad =
        x_motor_enabled ? applySlaveXMotorTarget(runtime.target_angle_rad, x_fallback_actual, nullptr)
                        : x_fallback_actual;
    const float y_actual_angle_rad =
        y_motor_enabled ? applySlaveYMotorTarget(runtime.target_y_angle_rad, y_fallback_actual, nullptr)
                        : y_fallback_actual;
#endif

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t state_start_us = SLAVE_TIMING_NOW_US();
#endif
    runtime.actual_angle_rad = x_actual_angle_rad;
    runtime.actual_y_angle_rad = y_actual_angle_rad;
    publishSlaveControlStateIfDue(runtime);

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
