#include "slave/status/slave_status.h"

#include <Arduino.h>

#include "common/protocol/protocol_types.h"
#include "common/protocol/protocol_units.h"
#include "common/system_state.h"
#include "slave/comm/slave_transport.h"
#include "slave/config/slave_config.h"
#include "slave/control/slave_coordinate_mapper.h"
#include "slave/control/slave_motion.h"
#include "slave/hardware/slave_hardware.h"
#include "slave/modes/mode_guard.h"
#include "slave/modes/mode_manager.h"
#include "slave/modes/mode_table.h"
#include "slave/safety/slave_safety.h"
#include "slave/tasks/slave_tasks.h"

#include <string.h>

namespace {

const char *drawStateName(uint8_t state) {
    switch (state) {
        case DRAW_STATE_IDLE:
            return "Idle";
        case DRAW_STATE_RUNNING:
            return "Running";
        case DRAW_STATE_FINISHED:
            return "Finished";
        case DRAW_STATE_BLOCKED:
            return "Blocked";
        case DRAW_STATE_LOADING:
            return "Loading";
        default:
            return "Unknown";
    }
}

const char *trajectoryPhaseName(uint8_t flags) {
    if ((flags & TRAJECTORY_STATUS_RUNNING) != 0U) {
        return "Running";
    }
    if ((flags & TRAJECTORY_STATUS_COMPLETE) != 0U) {
        return "Complete";
    }
    if ((flags & TRAJECTORY_STATUS_READY) != 0U) {
        return "Ready";
    }
    if ((flags & TRAJECTORY_STATUS_LOADING) != 0U) {
        return "Loading";
    }
    if ((flags & TRAJECTORY_STATUS_BLOCKED) != 0U) {
        return "Blocked";
    }
    return "None";
}

void trimDiagLine(char *line) {
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n' || line[len - 1] == ' ' || line[len - 1] == '\t')) {
        line[len - 1] = '\0';
        len--;
    }

    char *start = line;
    while (*start == ' ' || *start == '\t') {
        start++;
    }
    if (start != line) {
        memmove(line, start, strlen(start) + 1);
    }
}

void printSlaveDiagHelp() {
    Serial.println("[SlaveDiag] commands: help, mode, axis, paper, link, stats, fault clear, pen up, pen down_req, uv status, draw test, dryrun on, dryrun off");
}

void printSlaveModeDiag() {
    const SlaveRuntimeModeSnapshot runtime = getSlaveRuntimeModeSnapshot();
    const ModeCapability capability = slaveModeCapability();
    Serial.printf("[SlaveDiag] run_mode=%s default_app=%s app=%s requested=%s accepted=%u rejected=%u caps=0x%04x control_rate=%luhz outer_rate=%luhz run_mode_id=%u auto_draw_compiled=%u proto=%u flags=0x%04x requests=%lu last_change=%lums\n",
                  slaveRunModeName(),
                  slaveAppModeName(slaveDefaultAppMode()),
                  slaveAppModeName(static_cast<SlaveAppMode>(runtime.active_mode)),
                  slaveAppModeName(static_cast<SlaveAppMode>(runtime.requested_mode)),
                  static_cast<unsigned int>(runtime.request_accepted),
                  static_cast<unsigned int>(runtime.request_rejected),
                  static_cast<unsigned int>(capability.flags),
                  static_cast<unsigned long>(capability.control_rate_hz),
                  static_cast<unsigned long>(capability.outer_rate_hz),
                  static_cast<unsigned int>(SLAVE_RUN_MODE),
                  SLAVE_AUTO_DRAW_ENABLED ? 1 : 0,
                  static_cast<unsigned int>(runtime.last_protocol_mode),
                  static_cast<unsigned int>(runtime.last_command_flags),
                  static_cast<unsigned long>(runtime.request_count),
                  static_cast<unsigned long>(runtime.last_change_ms));
}

void printSlaveAxisDiag() {
    const SlaveRtCommand command = snapshotSlaveRtCommand();
    const SlaveMotionSnapshot motion = snapshotSlaveMotion();
    Serial.printf("[SlaveDiag] axis xcmd=%d ycmd=%d target=%.2f,%.2fmm smooth=%.2f,%.2fmm target_angle=%.4f,%.4frad actual=%.4f,%.4frad limit=%u/%u clamped=%u/%u fresh=%u pen_eff=%u draw=%s:%u%%\n",
                  command.x_norm,
                  command.y_norm,
                  motion.target_x_mm,
                  motion.target_y_mm,
                  motion.smooth_x_mm,
                  motion.smooth_y_mm,
                  motion.target_angle_rad,
                  motion.target_y_angle_rad,
                  motion.actual_angle_rad,
                  motion.actual_y_angle_rad,
                  motion.x_limit ? 1 : 0,
                  motion.y_limit ? 1 : 0,
                  motion.x_clamped ? 1 : 0,
                  motion.y_clamped ? 1 : 0,
                  isSlaveRtCommandFresh(command, micros()) ? 1 : 0,
                  static_cast<unsigned int>(motion.pen_req),
                  drawStateName(motion.draw_state),
                  static_cast<unsigned int>(motion.draw_progress_pct));
}

void printSlavePaperDiag() {
    Serial.printf("[SlaveDiag] paper width=%.1fmm height=%.1fmm distance=%.1fmm center=%.1f,%.1fmm sign=%d,%d angle_center=%.4f,%.4frad half=%.1f,%.1fmm limit_x=%.1f..%.1fmm limit_y=%.1f..%.1fmm settle=%.2f,%.2fmm\n",
                  kSlavePaperGeometry.width_mm,
                  kSlavePaperGeometry.height_mm,
                  kSlavePaperGeometry.distance_mm,
                  kSlavePaperGeometry.center_x_mm,
                  kSlavePaperGeometry.center_y_mm,
                  static_cast<int>(kSlavePaperGeometry.x_sign),
                  static_cast<int>(kSlavePaperGeometry.y_sign),
                  kSlavePaperGeometry.x_center_angle_rad,
                  kSlavePaperGeometry.y_center_angle_rad,
                  slaveAxisHalfRangeMm(AXIS_X),
                  slaveAxisHalfRangeMm(AXIS_Y),
                  slaveAxisLimitMinMm(AXIS_X),
                  slaveAxisLimitMaxMm(AXIS_X),
                  slaveAxisLimitMinMm(AXIS_Y),
                  slaveAxisLimitMaxMm(AXIS_Y),
                  kSlaveXTrajectory.settle_error_mm,
                  kSlaveYTrajectory.settle_error_mm);
}

void printSlaveLinkDiag() {
    const SlaveRtCommand command = snapshotSlaveRtCommand();
    const uint32_t now_us = micros();
    const uint32_t age_ms = command.valid ? ((now_us - command.last_rx_us) / 1000UL) : 0;
    Serial.printf("[SlaveDiag] link state=%u valid=%u fresh=%u seq=%lu age=%lums rxok=%lu rxrej=%lu stale=%lu duplicate=%lu send=%lu/%lu last_send=%u mode=%u flags=0x%04x\n",
                  static_cast<unsigned int>(sysData.link.link_state),
                  command.valid ? 1 : 0,
                  isSlaveRtCommandFresh(command, now_us) ? 1 : 0,
                  static_cast<unsigned long>(command.seq),
                  static_cast<unsigned long>(age_ms),
                  static_cast<unsigned long>(sysData.link.espnow_recv_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_reject_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_stale_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_duplicate_count),
                  static_cast<unsigned long>(sysData.link.espnow_send_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_send_fail_count),
                  static_cast<unsigned int>(sysData.link.last_send_ok),
                  static_cast<unsigned int>(command.mode),
                  static_cast<unsigned int>(command.command_flags));
}

void printSlaveStatsDiag() {
    const SlaveControlHealthSnapshot health = getSlaveControlHealthSnapshot();
    Serial.printf("[SlaveDiag] stats dt=%lu/%luus step=%lu/%luus miss=%lu/%lu cmd=%lu/%lu traj=%lu/%lu motor=%lu/%lu x_foc=%lu/%lu y_foc=%lu/%lu\n",
                  static_cast<unsigned long>(health.last_dt_us),
                  static_cast<unsigned long>(health.max_dt_us),
                  static_cast<unsigned long>(health.step_us),
                  static_cast<unsigned long>(health.step_max_us),
                  static_cast<unsigned long>(health.missed_delta),
                  static_cast<unsigned long>(health.missed_total),
                  static_cast<unsigned long>(health.command_us),
                  static_cast<unsigned long>(health.command_max_us),
                  static_cast<unsigned long>(health.trajectory_us),
                  static_cast<unsigned long>(health.trajectory_max_us),
                  static_cast<unsigned long>(health.motor_us),
                  static_cast<unsigned long>(health.motor_max_us),
                  static_cast<unsigned long>(health.x_foc_us),
                  static_cast<unsigned long>(health.x_foc_max_us),
                  static_cast<unsigned long>(health.y_foc_us),
                  static_cast<unsigned long>(health.y_foc_max_us));
}

void printSlaveUvDiag() {
    const uint32_t now_us = micros();
    const uint16_t reasons = evaluateSlaveUvBlockReasons(now_us);
    Serial.printf("[SlaveDiag] uv hw=%u allowed=%u out=%u blocked=%u reasons=0x%04x pen_state=%u active_faults=0x%04x latched_faults=0x%04x\n",
                  SLAVE_UV_HW_ENABLED ? 1 : 0,
                  reasons == UV_BLOCK_NONE ? 1 : 0,
                  sysData.link.uv_out ? 1 : 0,
                  sysData.slave.uv_interlock_blocked ? 1 : 0,
                  static_cast<unsigned int>(reasons),
                  static_cast<unsigned int>(sysData.slave.pen_state),
                  static_cast<unsigned int>(getActiveFaultFlags()),
                  static_cast<unsigned int>(getLatchedFaultFlags()));
}

void handleSlaveDiagCommand(char *line) {
    trimDiagLine(line);
    if (line[0] == '\0') {
        return;
    }

    if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
        printSlaveDiagHelp();
    } else if (strcmp(line, "mode") == 0) {
        printSlaveModeDiag();
    } else if (strcmp(line, "axis") == 0) {
        printSlaveAxisDiag();
    } else if (strcmp(line, "paper") == 0) {
        printSlavePaperDiag();
    } else if (strcmp(line, "link") == 0) {
        printSlaveLinkDiag();
    } else if (strcmp(line, "stats") == 0) {
        printSlaveStatsDiag();
    } else if (strcmp(line, "fault clear") == 0) {
        clearLocalFaults();
        Serial.printf("[SlaveDiag] fault cleared active=0x%04x latched=0x%04x merged=0x%04x\n",
                      static_cast<unsigned int>(getActiveFaultFlags()),
                      static_cast<unsigned int>(getLatchedFaultFlags()),
                      static_cast<unsigned int>(sysData.link.protocol_fault_flags));
    } else if (strcmp(line, "pen up") == 0 || strcmp(line, "pen down_req") == 0) {
        Serial.println("[SlaveDiag] pen is command-driven; use master button/packet, slave shell will not override UV path");
        printSlaveUvDiag();
    } else if (strcmp(line, "uv status") == 0) {
        printSlaveUvDiag();
    } else if (strcmp(line, "draw test") == 0) {
        const SlaveRtCommand command = snapshotSlaveRtCommand();
        const SlaveMotionSnapshot motion = snapshotSlaveMotion();
        Serial.printf("[SlaveDiag] draw_test auto_draw_compiled=%u app=%s draw_mode=%u dry_run=%u trajectory_req=%u proto=%u flags=0x%04x draw=%s:%u%% traj=%s task=%u rx=%u/%u cursor=%u tflags=0x%02x mask=%04x:%08lx pen_eff=%u\n",
                      SLAVE_AUTO_DRAW_ENABLED ? 1 : 0,
                      slaveAppModeName(currentSlaveAppMode()),
                      slaveAppModeIsDrawMode() ? 1 : 0,
                      slaveAppModeIsDryRun() ? 1 : 0,
                      slaveCommandRequestsTrajectory(command) ? 1 : 0,
                      static_cast<unsigned int>(command.mode),
                      static_cast<unsigned int>(command.command_flags),
                      drawStateName(motion.draw_state),
                      static_cast<unsigned int>(motion.draw_progress_pct),
                      trajectoryPhaseName(motion.trajectory_status_flags),
                      static_cast<unsigned int>(motion.trajectory_task_id),
                      static_cast<unsigned int>(motion.trajectory_received_count),
                      static_cast<unsigned int>(motion.trajectory_segment_count),
                      static_cast<unsigned int>(motion.trajectory_segment_cursor),
                      static_cast<unsigned int>(motion.trajectory_status_flags),
                      static_cast<unsigned int>(motion.trajectory_received_mask_high),
                      static_cast<unsigned long>(motion.trajectory_received_mask_low),
                      static_cast<unsigned int>(motion.pen_req));
    } else if (strcmp(line, "dryrun on") == 0 || strcmp(line, "dryrun off") == 0) {
        Serial.printf("[SlaveDiag] dryrun follows app mode and packet flags; startup=%s app=%s uv_reason=0x%04x\n",
                      slaveRunModeName(),
                      slaveAppModeName(currentSlaveAppMode()),
                      static_cast<unsigned int>(sysData.slave.uv_block_reasons));
    } else {
        Serial.printf("[SlaveDiag] unknown command: %s\n", line);
        printSlaveDiagHelp();
    }
}

}  // namespace

// 从机低频状态打印。这里允许 Serial.printf，因为它运行在 Core 0 状态任务中。
void printSlaveStatusLine() {
#if SLAVE_STATUS_LOG_ENABLED
    const SlaveRtCommand command = snapshotSlaveRtCommand();
    const SlaveMotionSnapshot motion = snapshotSlaveMotion();
    const uint32_t now_us = micros();
    const bool command_fresh = isSlaveRtCommandFresh(command, now_us);
    const uint32_t command_age_ms =
        command.valid ? ((now_us - command.last_rx_us) / 1000UL) : 0;

    const float actual_x_mm =
        normToUnit(slaveAxisGimbalAngleRadToNorm(AXIS_X, motion.actual_angle_rad)) *
        slaveAxisHalfRangeMm(AXIS_X);
    const float actual_y_mm =
        normToUnit(slaveAxisGimbalAngleRadToNorm(AXIS_Y, motion.actual_y_angle_rad)) *
        slaveAxisHalfRangeMm(AXIS_Y);
    const float x_total_err_mm = motion.target_x_mm - actual_x_mm;
    const float y_total_err_mm = motion.target_y_mm - actual_y_mm;
#if SLAVE_STATUS_TIMING_LOG_ENABLED
    const SlaveControlHealthSnapshot health = getSlaveControlHealthSnapshot();
#endif
    const uint16_t active_faults = getActiveFaultFlags();
    const uint16_t latched_faults = getLatchedFaultFlags();

    float y_sensor_angle_rad = 0.0f;
    uint16_t y_sensor_raw = 0;
#if SLAVE_STATUS_Y_SENSOR_BRINGUP_LOG_ENABLED
    const bool y_sensor_sampled = sampleSlaveYSensorForStatus(&y_sensor_angle_rad, &y_sensor_raw);
#else
    const bool y_sensor_sampled = false;
#endif

#if SLAVE_STATUS_SUMMARY_LOG_ENABLED
    const SlaveRuntimeModeSnapshot runtime = getSlaveRuntimeModeSnapshot();
    Serial.printf("[Slave] startup=%s app=%s requested=%s rejected=%u perf=%s link=%u rx=%lu rxok=%lu rxrej=%lu stale=%lu duplicate=%lu fresh=%u proto=%u flags=0x%04x active_faults=0x%04x latched_faults=0x%04x faults=0x%04x pen_state=%u pen_eff=%u draw=%s:%u%% traj=%s task=%u rx=%u/%u cursor=%u tflags=0x%02x uv_hw=%u uv_out=%u uv_block=%u uv_reason=0x%04x age=%lums\n",
                  slaveRunModeName(),
                  slaveAppModeName(static_cast<SlaveAppMode>(runtime.active_mode)),
                  slaveAppModeName(static_cast<SlaveAppMode>(runtime.requested_mode)),
                  static_cast<unsigned int>(runtime.request_rejected),
                  slaveControlPerfModeName(),
                  static_cast<unsigned int>(sysData.link.link_state),
                  static_cast<unsigned long>(sysData.link.last_command_seq),
                  static_cast<unsigned long>(sysData.link.espnow_recv_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_reject_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_stale_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_duplicate_count),
                  command_fresh ? 1 : 0,
                  static_cast<unsigned int>(command.mode),
                  static_cast<unsigned int>(command.command_flags),
                  static_cast<unsigned int>(active_faults),
                  static_cast<unsigned int>(latched_faults),
                  static_cast<unsigned int>(sysData.link.protocol_fault_flags),
                  static_cast<unsigned int>(sysData.slave.pen_state),
                  static_cast<unsigned int>(motion.pen_req),
                  drawStateName(motion.draw_state),
                  static_cast<unsigned int>(motion.draw_progress_pct),
                  trajectoryPhaseName(motion.trajectory_status_flags),
                  static_cast<unsigned int>(motion.trajectory_task_id),
                  static_cast<unsigned int>(motion.trajectory_received_count),
                  static_cast<unsigned int>(motion.trajectory_segment_count),
                  static_cast<unsigned int>(motion.trajectory_segment_cursor),
                  static_cast<unsigned int>(motion.trajectory_status_flags),
                  SLAVE_UV_HW_ENABLED ? 1 : 0,
                  sysData.link.uv_out ? 1 : 0,
                  sysData.slave.uv_interlock_blocked ? 1 : 0,
                  static_cast<unsigned int>(sysData.slave.uv_block_reasons),
                  static_cast<unsigned long>(command_age_ms));
#endif

#if SLAVE_STATUS_XY_LOG_ENABLED
    Serial.printf("[SlaveXY] xcmd=%d ycmd=%d x_cmd=%.2fmm y_cmd=%.2fmm x_smooth=%.2fmm y_smooth=%.2fmm x_target=%.4frad y_target=%.4frad x_actual=%.4frad y_actual=%.4frad x_track_err=%.1fmrad y_track_err=%.1fmrad x_total_err=%.2fmm y_total_err=%.2fmm x_limit=%u y_limit=%u x_clamped=%u y_clamped=%u y_raw=%u y_angle=%.4frad y_sample=%u\n",
                  command.x_norm,
                  command.y_norm,
                  motion.target_x_mm,
                  motion.target_y_mm,
                  motion.smooth_x_mm,
                  motion.smooth_y_mm,
                  motion.target_angle_rad,
                  motion.target_y_angle_rad,
                  motion.actual_angle_rad,
                  motion.actual_y_angle_rad,
                  motion.x_track_err_mrad,
                  motion.y_track_err_mrad,
                  x_total_err_mm,
                  y_total_err_mm,
                  motion.x_limit ? 1 : 0,
                  motion.y_limit ? 1 : 0,
                  motion.x_clamped ? 1 : 0,
                  motion.y_clamped ? 1 : 0,
                  static_cast<unsigned int>(y_sensor_raw),
                  y_sensor_angle_rad,
                  y_sensor_sampled ? 1 : 0);
#endif

#if SLAVE_STATUS_TIMING_LOG_ENABLED
    Serial.printf("[SlaveTiming] diag_level=%u ctrl_dt=%lu ctrl_max=%lu step_us=%lu step_max=%lu over_period=%lu over_75pct=%lu over_50pct=%lu over_200=%lu over_300=%lu cmd_us=%lu/%lu traj_us=%lu/%lu motor_us=%lu/%lu x_sensor_us=%lu/%lu x_foc_us=%lu/%lu x_move_us=%lu/%lu y_sensor_us=%lu/%lu y_foc_us=%lu/%lu y_move_us=%lu/%lu state_us=%lu/%lu pub_us=%lu/%lu x_foc_run=%lu x_foc_skip=%lu x_foc_div=%lu y_foc_run=%lu y_foc_skip=%lu y_foc_div=%lu ctrl_miss_delta=%lu ctrl_miss=%lu\n",
                  static_cast<unsigned int>(SLAVE_TIMING_DIAG_LEVEL),
                  static_cast<unsigned long>(health.last_dt_us),
                  static_cast<unsigned long>(health.max_dt_us),
                  static_cast<unsigned long>(health.step_us),
                  static_cast<unsigned long>(health.step_max_us),
                  static_cast<unsigned long>(health.step_over_period_delta),
                  static_cast<unsigned long>(health.step_over_75pct_delta),
                  static_cast<unsigned long>(health.step_over_50pct_delta),
                  static_cast<unsigned long>(health.step_over_200_delta),
                  static_cast<unsigned long>(health.step_over_300_delta),
                  static_cast<unsigned long>(health.command_us),
                  static_cast<unsigned long>(health.command_max_us),
                  static_cast<unsigned long>(health.trajectory_us),
                  static_cast<unsigned long>(health.trajectory_max_us),
                  static_cast<unsigned long>(health.motor_us),
                  static_cast<unsigned long>(health.motor_max_us),
                  static_cast<unsigned long>(health.x_sensor_us),
                  static_cast<unsigned long>(health.x_sensor_max_us),
                  static_cast<unsigned long>(health.x_foc_us),
                  static_cast<unsigned long>(health.x_foc_max_us),
                  static_cast<unsigned long>(health.x_move_us),
                  static_cast<unsigned long>(health.x_move_max_us),
                  static_cast<unsigned long>(health.y_sensor_us),
                  static_cast<unsigned long>(health.y_sensor_max_us),
                  static_cast<unsigned long>(health.y_foc_us),
                  static_cast<unsigned long>(health.y_foc_max_us),
                  static_cast<unsigned long>(health.y_move_us),
                  static_cast<unsigned long>(health.y_move_max_us),
                  static_cast<unsigned long>(health.state_us),
                  static_cast<unsigned long>(health.state_max_us),
                  static_cast<unsigned long>(health.publish_us),
                  static_cast<unsigned long>(health.publish_max_us),
                  static_cast<unsigned long>(health.x_foc_run_delta),
                  static_cast<unsigned long>(health.x_foc_skip_delta),
                  static_cast<unsigned long>(health.x_foc_divisor),
                  static_cast<unsigned long>(health.y_foc_run_delta),
                  static_cast<unsigned long>(health.y_foc_skip_delta),
                  static_cast<unsigned long>(health.y_foc_divisor),
                  static_cast<unsigned long>(health.missed_delta),
                  static_cast<unsigned long>(health.missed_total));
#endif
#endif
}

void processSlaveDiagShell() {
    static char line[80] = {};
    static size_t len = 0;

    while (Serial.available() > 0) {
        const int ch = Serial.read();
        if (ch < 0) {
            break;
        }

        if (ch == '\r' || ch == '\n') {
            line[len] = '\0';
            handleSlaveDiagCommand(line);
            len = 0;
            line[0] = '\0';
            continue;
        }

        if (len + 1 < sizeof(line)) {
            line[len++] = static_cast<char>(ch);
        } else {
            len = 0;
            line[0] = '\0';
            Serial.println("[SlaveDiag] command too long");
        }
    }
}
