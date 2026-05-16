#include "slave/status/slave_status.h"

#include <Arduino.h>

#include "common/protocol/protocol_units.h"
#include "common/system_state.h"
#include "slave/control/slave_motion.h"
#include "slave/tasks/slave_tasks.h"
#include "slave/comm/slave_transport.h"

// 从机状态打印模块。
// 这里允许 Serial.printf，因为它只在 Core 0 的低频状态任务中运行；
// 任何控制热路径都不应调用本函数。

void printSlaveStatusLine() {
    // 读取最近一次有效主机命令快照，状态行中的 xcmd 来自这里。
    const MasterCommandPacket command = snapshotMasterCommand();

    // 没有有效命令时 age 显示 0，避免启动阶段出现无意义的大数。
    const uint32_t now_us = micros();
    const uint32_t command_age_ms =
        sysData.link.command_valid ? ((now_us - sysData.link.last_rx_us) / 1000UL) : 0;

    // 用毫弧度显示跟踪误差，比直接显示 rad 更容易观察 UV 联锁是否会放行。
    const float tracking_error_mrad =
        (sysData.slave.target_angle_rad - sysData.slave.actual_angle_rad) * 1000.0f;
    const float actual_x_mm =
        normToUnit(gimbalAngleRadToXNorm(sysData.slave.actual_angle_rad)) * PLOT_X_HALF_RANGE_MM;
    const float cmd_smooth_err_mm = sysData.slave.target_x_mm - sysData.slave.smooth_x_mm;
    const float total_err_mm = sysData.slave.target_x_mm - actual_x_mm;
    const SlaveControlHealthSnapshot health = getSlaveControlHealthSnapshot();
    const uint16_t active_faults = getActiveFaultFlags();
    const uint16_t latched_faults = getLatchedFaultFlags();

    // 字段含义在 Instruction.md 中有对应说明。保持一行输出，便于双串口同时观察。
    Serial.printf("[Slave] rx=%lu xcmd=%d x_cmd=%.1fmm x_smooth=%.1fmm x=%.1f%% target=%.3frad actual=%.3frad cmd_smooth_err_mm=%.2f track_err_mrad=%.1f total_err_mm=%.2f uv=%u uv_block=%u age=%lums ctrl_dt=%luus ctrl_max=%luus ctrl_miss_delta=%lu ctrl_miss=%lu step_us=%lu step_max=%luus send=%lu/%lu rxok=%lu rxbad=%lu last=%u active_faults=0x%04x latched_faults=0x%04x faults=0x%04x\n",
                  static_cast<unsigned long>(sysData.link.last_command_seq),
                  command.x_norm,
                  sysData.slave.target_x_mm,
                  sysData.slave.smooth_x_mm,
                  sysData.slave.x_pos,
                  sysData.slave.target_angle_rad,
                  sysData.slave.actual_angle_rad,
                  cmd_smooth_err_mm,
                  tracking_error_mrad,
                  total_err_mm,
                  sysData.link.pen_down ? 1 : 0,
                  sysData.slave.uv_interlock_blocked ? 1 : 0,
                  static_cast<unsigned long>(command_age_ms),
                  static_cast<unsigned long>(health.last_dt_us),
                  static_cast<unsigned long>(health.max_dt_us),
                  static_cast<unsigned long>(health.missed_delta),
                  static_cast<unsigned long>(health.missed_total),
                  static_cast<unsigned long>(health.step_us),
                  static_cast<unsigned long>(health.step_max_us),
                  static_cast<unsigned long>(sysData.link.espnow_send_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_send_fail_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_reject_count),
                  static_cast<unsigned int>(sysData.link.last_send_ok),
                  static_cast<unsigned int>(active_faults),
                  static_cast<unsigned int>(latched_faults),
                  static_cast<unsigned int>(sysData.link.protocol_fault_flags));
    Serial.printf("[SlaveTiming] cmd_us=%lu/%lu traj_us=%lu/%lu motor_us=%lu/%lu foc_us=%lu/%lu move_us=%lu/%lu angle_us=%lu/%lu state_us=%lu/%lu pub_us=%lu/%lu foc_run=%lu foc_skip=%lu foc_div=%lu\n",
                  static_cast<unsigned long>(health.command_us),
                  static_cast<unsigned long>(health.command_max_us),
                  static_cast<unsigned long>(health.trajectory_us),
                  static_cast<unsigned long>(health.trajectory_max_us),
                  static_cast<unsigned long>(health.motor_us),
                  static_cast<unsigned long>(health.motor_max_us),
                  static_cast<unsigned long>(health.foc_us),
                  static_cast<unsigned long>(health.foc_max_us),
                  static_cast<unsigned long>(health.move_us),
                  static_cast<unsigned long>(health.move_max_us),
                  static_cast<unsigned long>(health.angle_read_us),
                  static_cast<unsigned long>(health.angle_read_max_us),
                  static_cast<unsigned long>(health.state_us),
                  static_cast<unsigned long>(health.state_max_us),
                  static_cast<unsigned long>(health.publish_us),
                  static_cast<unsigned long>(health.publish_max_us),
                  static_cast<unsigned long>(health.foc_run_delta),
                  static_cast<unsigned long>(health.foc_skip_delta),
                  static_cast<unsigned long>(health.foc_divisor));
}
