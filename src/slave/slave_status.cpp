#include "slave/slave_status.h"

#include <Arduino.h>

#include "common/system_state.h"
#include "slave/slave_tasks.h"
#include "slave/slave_transport.h"

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

    // 字段含义在 Instruction.md 中有对应说明。保持一行输出，便于双串口同时观察。
    Serial.printf("[Slave] rx=%lu xcmd=%d x=%.1f%% target=%.3frad actual=%.3frad err=%.1fmrad uv=%u uv_block=%u age=%lums ctrl_dt=%luus ctrl_miss=%lu send=%lu/%lu rxok=%lu rxbad=%lu last=%u faults=0x%04x\n",
                  static_cast<unsigned long>(sysData.link.last_command_seq),
                  command.x_norm,
                  sysData.slave.x_pos,
                  sysData.slave.target_angle_rad,
                  sysData.slave.actual_angle_rad,
                  tracking_error_mrad,
                  sysData.link.pen_down ? 1 : 0,
                  sysData.slave.uv_interlock_blocked ? 1 : 0,
                  static_cast<unsigned long>(command_age_ms),
                  static_cast<unsigned long>(getSlaveControlLastDtUs()),
                  static_cast<unsigned long>(getSlaveControlTimerMissedTicks()),
                  static_cast<unsigned long>(sysData.link.espnow_send_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_send_fail_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_reject_count),
                  static_cast<unsigned int>(sysData.link.last_send_ok),
                  static_cast<unsigned int>(sysData.link.protocol_fault_flags));
}
