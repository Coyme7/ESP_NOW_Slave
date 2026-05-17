#include "slave/status/slave_status.h"

#include <Arduino.h>

#include "common/protocol/protocol_units.h"
#include "common/system_state.h"
#include "slave/comm/slave_transport.h"
#include "slave/config/slave_config.h"
#include "slave/control/slave_coordinate_mapper.h"
#include "slave/control/slave_motion.h"
#include "slave/hardware/slave_hardware.h"
#include "slave/safety/slave_safety.h"
#include "slave/tasks/slave_tasks.h"

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
    const bool y_sensor_sampled = refreshSlaveYSensorForBringup(&y_sensor_angle_rad, &y_sensor_raw);
#else
    const bool y_sensor_sampled = false;
#endif

#if SLAVE_STATUS_SUMMARY_LOG_ENABLED
    Serial.printf("[Slave] mode=%s perf=%s rx=%lu rxok=%lu rxrej=%lu stale=%lu duplicate=%lu fresh=%u active_faults=0x%04x latched_faults=0x%04x faults=0x%04x uv_hw=%u uv_out=%u uv_block=%u age=%lums y_bringup=%s\n",
                  slaveRunModeName(),
                  slaveControlPerfModeName(),
                  static_cast<unsigned long>(sysData.link.last_command_seq),
                  static_cast<unsigned long>(sysData.link.espnow_recv_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_reject_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_stale_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_duplicate_count),
                  command_fresh ? 1 : 0,
                  static_cast<unsigned int>(active_faults),
                  static_cast<unsigned int>(latched_faults),
                  static_cast<unsigned int>(sysData.link.protocol_fault_flags),
                  SLAVE_UV_HW_ENABLED ? 1 : 0,
                  sysData.link.pen_down ? 1 : 0,
                  sysData.slave.uv_interlock_blocked ? 1 : 0,
                  static_cast<unsigned long>(command_age_ms),
                  slaveYBringupModeName());
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
