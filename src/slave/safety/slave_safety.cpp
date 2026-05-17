#include "slave/safety/slave_safety.h"

#include <math.h>

#include "common/system_state.h"
#include "slave/config/slave_config.h"
#include "slave/hardware/slave_hardware.h"
#include "slave/comm/slave_transport.h"
#include "slave/control/slave_motion.h"

// 从机安全模块。
// 它独立于运动控制运行在 Core 0，核心原则是：任何不确定状态都关 UV。
// 真正写 UV GPIO 的位置只有 runSlaveSafetyStep() -> setUvPen()，方便审查失效保护。

bool isSlaveRtCommandFresh(const SlaveRtCommand &command, uint32_t now_us) {
    if (command.valid == 0) {
        return false;
    }

    return static_cast<uint32_t>(now_us - command.last_rx_us) <= COMMAND_TIMEOUT_US;
}

bool isSlaveCommandTimedOut(uint32_t now_us) {
    // 统一使用 SlaveRtCommand fresh 语义。
    // sysData.link.command_valid 只作为低频状态显示，不再参与控制/安全判断。
    const SlaveRtCommand command = snapshotSlaveRtCommand();
    return !isSlaveRtCommandFresh(command, now_us);
}

bool isSlaveUvAllowed(uint32_t now_us) {
    // 取实时命令和运动快照，避免安全任务读取完整协议包或直接访问传感器。
    const SlaveRtCommand command = snapshotSlaveRtCommand();
    const SlaveMotionSnapshot motion = snapshotSlaveMotion();

    if (!isSlaveRtCommandFresh(command, now_us)) {
        // 没有新鲜命令时必须关灯。
        return false;
    }

    if (command.pen_down == 0) {
        // 主机没有明确要求落笔时必须关灯。
        return false;
    }

    if (motion.boundary_hit) {
        // 触边表示光斑接近安全绘图边界，必须阻止 UV。
        return false;
    }

    // 只有平滑目标接近命令目标，且实际角度足够接近目标角度，才认为光斑稳定到可落笔位置。
    const float smoothing_error_mm =
        fabsf(motion.target_x_mm - motion.smooth_x_mm);
    if (smoothing_error_mm > kSlaveTrajectory.settle_error_mm) {
        return false;
    }

    const float tracking_error_rad =
        fabsf(motion.target_angle_rad - motion.actual_angle_rad);
    return tracking_error_rad <= kSlaveXAxis.settle_error_rad;
}

void runSlaveSafetyStep(uint32_t now_us) {
    // uv_allowed 是最终输出判定；command.pen_down 用于区分“主机没要求落笔”和
    // “主机要求落笔但联锁条件不满足”。当前 UV 硬件默认关闭时不锁存 UV fault，
    // 避免 SingleX 电机性能阶段被 UV 逻辑污染。
    const bool uv_allowed = isSlaveUvAllowed(now_us);
    const SlaveRtCommand command = snapshotSlaveRtCommand();
    const bool command_fresh = isSlaveRtCommandFresh(command, now_us);

    sysData.slave.uv_interlock_blocked =
        SLAVE_UV_HW_ENABLED &&
        command_fresh &&
        command.pen_down &&
        !uv_allowed;

#if SLAVE_UV_HW_ENABLED
    if (sysData.slave.uv_interlock_blocked) {
        // 只有 UV 硬件阶段才锁存 UV_INTERLOCK，避免当前 SingleX 电机调试阶段被 UV fault 污染。
        addLocalFault(FAULT_UV_INTERLOCK);
    }
#endif

    // 唯一的 UV GPIO 写出口。任何 false 都会关闭两路 UV MOS。
    setUvPen(uv_allowed);
}
