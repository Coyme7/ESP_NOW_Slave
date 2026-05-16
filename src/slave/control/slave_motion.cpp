#include "slave/control/slave_motion.h"

#include <math.h>

#include "common/system_state.h"
#include "slave/config/slave_config.h"
#include "slave/control/slave_coordinate_mapper.h"
#include "slave/control/slave_trajectory_smoother.h"
#include "slave/hardware/slave_hardware.h"
#include "slave/safety/slave_safety.h"
#include "slave/comm/slave_transport.h"

// 从机运动模块。
// 数据流：MasterCommandPacket.x_norm -> 纸面毫米 -> 云台目标角 ->
// 真实电机或仿真跟随 -> 实际角/实际 x_norm 遥测。
// 这里位于从机本地控制热路径，不做无线发送、串口输出或 UV GPIO 写。

float xNormToPaperMm(int16_t x_norm) {
    return slaveXNormToPaperMm(x_norm);
}

float paperMmToGimbalAngleRad(float x_mm) {
    return slavePaperMmToGimbalAngleRad(x_mm);
}

int16_t gimbalAngleRadToXNorm(float angle_rad) {
    return slaveGimbalAngleRadToXNorm(angle_rad);
}

void runSlaveControlStep(float dt_s, SlaveControlStepTiming *timing) {
    static SlaveTrajectorySmootherState x_smoother = {};
    if (timing != nullptr) {
        *timing = {};
    }
    const uint32_t command_start_us = micros();

    // now_us 用于命令超时判断；faults 只记录本步实时状态，历史故障由 system_state 单独锁存。
    const uint32_t now_us = micros();
    uint16_t faults = FAULT_NONE;

    // ESP-NOW 回调只维护最新命令快照；控制步通过短临界区复制后使用稳定副本。
    const MasterCommandPacket command = snapshotMasterCommand();

    if (isSlaveCommandTimedOut(now_us)) {
        // 命令失效时立即标记 command_valid=false，并把目标保持在中心角。
        // UV 安全任务也会因为命令超时而关闭紫光。
        sysData.link.command_valid = false;
        faults |= FAULT_COMMAND_TIMEOUT;
    }

    // 默认目标是纸面中心。只有命令有效且处于协作绘图模式时，才接受主机 x_norm。
    float target_x_mm = 0.0f;
    if (sysData.link.command_valid && sysData.link.current_mode == MODE_COLLAB_DRAW) {
        target_x_mm = xNormToPaperMm(command.x_norm);
    }
    const uint32_t trajectory_start_us = micros();

    const SlaveTrajectorySmootherInput smoother_input = {
        target_x_mm,
        dt_s,
        command.pen_down ? kSlaveTrajectory.draw_speed_mm_s : kSlaveTrajectory.lift_speed_mm_s,
        kSlaveTrajectory.accel_mm_s2,
        kSlaveTrajectory.command_deadband_mm,
    };
    const SlaveTrajectorySmootherOutput smoother_output =
        updateSlaveTrajectorySmoother(x_smoother, smoother_input);
    const float target_angle_rad = paperMmToGimbalAngleRad(smoother_output.x_mm);

    // 边界判断基于纸面目标是否达到硬边界；触边时 UV 联锁会阻止落笔。
    const bool at_edge = fabsf(target_x_mm) >= PLOT_X_HALF_RANGE_MM;
    if (at_edge) {
        faults |= FAULT_BOUNDARY_HIT;
    }

    sysData.slave.boundary_hit = at_edge;
    sysData.slave.target_x_mm = target_x_mm;
    sysData.slave.smooth_x_mm = smoother_output.x_mm;
    sysData.slave.target_angle_rad = target_angle_rad;
    const uint32_t motor_start_us = micros();

    // 硬件关闭时，用一阶跟随模拟实际角度逐步靠近目标。
    // 硬件打开时 applySlaveXMotorTarget() 会返回真实 shaft_angle。
    const float error_rad = target_angle_rad - sysData.slave.actual_angle_rad;
    const float simulated_actual_angle_rad =
        sysData.slave.actual_angle_rad + (error_rad * kSlaveXAxis.simulated_response_alpha);
    SlaveXMotorStepTiming motor_timing = {};
    sysData.slave.actual_angle_rad =
        applySlaveXMotorTarget(target_angle_rad, simulated_actual_angle_rad, &motor_timing);
    const uint32_t state_start_us = micros();

    // 更新供串口和遥测读取的实际状态。Y 轴尚未实现，保持 0。
    sysData.slave.angle_deg = radToDeg(sysData.slave.actual_angle_rad);
    sysData.slave.x_pos = normToPercent(gimbalAngleRadToXNorm(sysData.slave.actual_angle_rad));
    sysData.slave.y_pos = 0.0f;
    const uint32_t publish_start_us = micros();

    // 将本步故障和 UV 联锁状态发布出去，后续 sendSlaveTelemetry() 会带回主机。
    publishProtocolFaults(faults |
                          (sysData.slave.uv_interlock_blocked ? FAULT_UV_INTERLOCK : FAULT_NONE));
    const uint32_t done_us = micros();
    if (timing != nullptr) {
        timing->command_us = trajectory_start_us - command_start_us;
        timing->trajectory_us = motor_start_us - trajectory_start_us;
        timing->motor_us = state_start_us - motor_start_us;
        timing->foc_us = motor_timing.loop_foc_us;
        timing->foc_ran = motor_timing.loop_foc_ran;
        timing->move_us = motor_timing.move_us;
        timing->angle_read_us = motor_timing.read_us;
        timing->state_us = publish_start_us - state_start_us;
        timing->publish_us = done_us - publish_start_us;
    }
}

void runSlaveControlStep(float dt_s) {
    runSlaveControlStep(dt_s, nullptr);
}
