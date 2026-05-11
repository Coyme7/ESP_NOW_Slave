#include "slave/slave_motion.h"

#include <math.h>

#include "common/system_state.h"
#include "slave/slave_config.h"
#include "slave/slave_hardware.h"
#include "slave/slave_safety.h"
#include "slave/slave_transport.h"

// 从机运动模块。
// 数据流：MasterCommandPacket.x_norm -> 纸面毫米 -> 云台目标角 ->
// 真实电机或仿真跟随 -> 实际角/实际 x_norm 遥测。
// 这里位于 10 kHz 控制热路径，不做无线发送、串口输出或 UV GPIO 写。

float xNormToPaperMm(int16_t x_norm) {
    // 协议坐标表示纸面目标，不是电机电角度或编码器原始值。
    // normToUnit() 先把 int16 坐标归一到 -1..+1，再乘以当前安全绘图半幅。
    return normToUnit(x_norm) * PLOT_X_HALF_RANGE_MM;
}

float paperMmToGimbalAngleRad(float x_mm) {
    // A4 中心为零点。700 mm 投影距离下，+/-95 mm 约等于 +/-7.73 deg。
    // 先夹紧纸面目标，确保任何异常命令都不会让目标角超过当前安全绘图范围。
    const float limited_x_mm = clampFloat(x_mm, -PLOT_X_HALF_RANGE_MM, PLOT_X_HALF_RANGE_MM);

    // direction 负责适配机械安装方向；center_angle_rad 是光斑落在 A4 中心时的云台角。
    return kSlaveXAxis.center_angle_rad +
           (kSlaveXAxis.direction * atanf(limited_x_mm / kSlaveXAxis.throw_distance_mm));
}

int16_t gimbalAngleRadToXNorm(float angle_rad) {
    // 反向换算用于遥测：把当前云台角投影回纸面 X，再压回协议坐标。
    // 这不是控制目标来源，只是让主机知道从机“实际光斑大概在哪里”。
    const float axis_angle_rad = (angle_rad - kSlaveXAxis.center_angle_rad) * kSlaveXAxis.direction;
    const float x_mm = tanf(axis_angle_rad) * kSlaveXAxis.throw_distance_mm;
    return unitToNorm(x_mm / PLOT_X_HALF_RANGE_MM);
}

void runSlaveControlStep() {
    // now_us 用于命令超时判断；faults 从本机锁存故障开始叠加本步运行状态。
    const uint32_t now_us = micros();
    uint16_t faults = getLocalFaultFlags();

    // ESP-NOW 回调只维护最新命令快照；控制步通过短临界区复制后使用稳定副本。
    const MasterCommandPacket command = snapshotMasterCommand();

    if (isSlaveCommandTimedOut(now_us)) {
        // 命令失效时立即标记 command_valid=false，并把目标保持在中心角。
        // UV 安全任务也会因为命令超时而关闭紫光。
        sysData.command_valid = false;
        faults |= FAULT_COMMAND_TIMEOUT;
    }

    // 默认目标是中心角。只有命令有效且处于协作绘图模式时，才接受主机 x_norm。
    float target_angle_rad = kSlaveXAxis.center_angle_rad;
    if (sysData.command_valid && sysData.current_mode == MODE_COLLAB_DRAW) {
        target_angle_rad = paperMmToGimbalAngleRad(xNormToPaperMm(command.x_norm));
    }

    // 边界判断基于协议坐标是否接近满幅。当前阈值 0.999f 用于识别端点命令，
    // 一旦触边，UV 联锁会阻止落笔。
    const float x_unit = normToUnit(command.x_norm);
    const bool at_edge = fabsf(x_unit) >= 0.999f;
    if (at_edge) {
        faults |= FAULT_BOUNDARY_HIT;
    }

    sysData.boundary_hit = at_edge;
    sysData.slave_target_angle_rad = target_angle_rad;

    // 硬件关闭时，用一阶跟随模拟实际角度逐步靠近目标。
    // 硬件打开时 applySlaveXMotorTarget() 会返回真实 shaft_angle。
    const float error_rad = target_angle_rad - sysData.slave_actual_angle_rad;
    const float simulated_actual_angle_rad =
        sysData.slave_actual_angle_rad + (error_rad * kSlaveXAxis.simulated_response_alpha);
    sysData.slave_actual_angle_rad =
        applySlaveXMotorTarget(target_angle_rad, simulated_actual_angle_rad);

    // 更新供串口和遥测读取的实际状态。Y 轴尚未实现，保持 0。
    sysData.slave_angle_deg = radToDeg(sysData.slave_actual_angle_rad);
    sysData.slave_x_pos = normToPercent(gimbalAngleRadToXNorm(sysData.slave_actual_angle_rad));
    sysData.slave_y_pos = 0.0f;

    // 将本步故障和 UV 联锁状态发布出去，后续 sendSlaveTelemetry() 会带回主机。
    publishProtocolFaults(faults | (sysData.uv_interlock_blocked ? FAULT_UV_INTERLOCK : FAULT_NONE));
}
