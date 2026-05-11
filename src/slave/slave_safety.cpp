#include "slave/slave_safety.h"

#include <math.h>

#include "common/system_state.h"
#include "slave/slave_config.h"
#include "slave/slave_hardware.h"
#include "slave/slave_transport.h"

// 从机安全模块。
// 它独立于 10 kHz 运动控制运行在 Core 0，核心原则是：任何不确定状态都关 UV。
// 真正写 UV GPIO 的位置只有 runSlaveSafetyStep() -> setUvPen()，方便审查失效保护。

bool isSlaveCommandTimedOut(uint32_t now_us) {
    // command_valid 由 ESP-NOW 接收回调维护；last_rx_us 是最后一次接受有效命令的时间。
    // 只要从未收到有效包，或有效包超过 COMMAND_TIMEOUT_US 没更新，就视为超时。
    return !sysData.command_valid || now_us - sysData.last_rx_us > COMMAND_TIMEOUT_US;
}

bool isSlaveUvAllowed(uint32_t now_us) {
    // 取命令快照而不是直接读取 rxPacket，避免安全任务读到回调正在写入的半包。
    const MasterCommandPacket command = snapshotMasterCommand();

    if (isSlaveCommandTimedOut(now_us)) {
        // 没有新鲜命令时必须关灯。
        return false;
    }
    if (!command.pen_down) {
        // 主机没有明确要求落笔时必须关灯。
        return false;
    }
    if (sysData.boundary_hit) {
        // 触边表示光斑接近安全绘图边界，必须阻止 UV。
        return false;
    }

    // 只有实际角度足够接近目标角度，才认为光斑稳定到可落笔位置。
    const float tracking_error_rad = fabsf(sysData.slave_target_angle_rad - sysData.slave_actual_angle_rad);
    return tracking_error_rad <= kSlaveXAxis.settle_error_rad;
}

void runSlaveSafetyStep(uint32_t now_us) {
    // uv_allowed 是最终输出判定；command.pen_down 用于区分“主机没要求落笔”和
    // “主机要求落笔但联锁条件不满足”，后者会在串口中显示 uv_block=1。
    const bool uv_allowed = isSlaveUvAllowed(now_us);
    const MasterCommandPacket command = snapshotMasterCommand();

    sysData.uv_interlock_blocked = command.pen_down && !uv_allowed;
    if (sysData.uv_interlock_blocked) {
        // 联锁阻止落笔时锁存 fault，方便主机和从机串口都能看见。
        addLocalFault(FAULT_UV_INTERLOCK);
    }

    // 唯一的 UV GPIO 写出口。任何 false 都会关闭两路 UV MOS。
    setUvPen(uv_allowed);
}
