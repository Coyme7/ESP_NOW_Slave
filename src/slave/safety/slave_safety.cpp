#include "slave/safety/slave_safety.h"

#include <math.h>

#include "common/system_state.h"
#include "common/timing/link_timing.h"
#include "slave/config/slave_config.h"
#include "slave/hardware/slave_hardware.h"
#include "slave/comm/slave_transport.h"
#include "slave/control/slave_motion.h"
#include "slave/modes/mode_guard.h"
#include "slave/modes/mode_table.h"
#include "slave/safety/pen_state_machine.h"

// 从机安全模块。
// 它独立于运动控制运行在 Core 0，核心原则是：任何不确定状态都关 UV。
// 真正写 UV GPIO 的位置只有 runSlaveSafetyStep() -> setUvOutput()，方便审查失效保护。

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

uint16_t evaluateSlaveUvBlockReasons(uint32_t now_us) {
    // 取实时命令和运动快照，避免安全任务读取完整协议包或直接访问传感器。
    const SlaveRtCommand command = snapshotSlaveRtCommand();
    const SlaveMotionSnapshot motion = snapshotSlaveMotion();
    const ModeCapability capability = slaveModeCapability();
    uint16_t reasons = UV_BLOCK_NONE;

    if (!isSlaveRtCommandFresh(command, now_us)) {
        // 没有新鲜命令时必须关灯。
        reasons |= UV_BLOCK_TIMEOUT;
    }

    if (motion.pen_req == 0) {
        // planner 没有发布有效落笔请求时必须关灯；AutoDraw 使用轨迹 effective_pen_req。
        reasons |= UV_BLOCK_PEN_UP;
    }

    if (motion.boundary_hit) {
        // 触边表示光斑接近安全绘图边界，必须阻止 UV。
        reasons |= UV_BLOCK_BOUNDARY;
    }

    const uint16_t non_uv_faults =
        static_cast<uint16_t>(getActiveFaultFlags() & ~static_cast<uint16_t>(FAULT_UV_INTERLOCK));
    if (non_uv_faults != FAULT_NONE) {
        reasons |= UV_BLOCK_FAULT;
    }

    if (!slaveModeAllowsUv(command)) {
        reasons |= UV_BLOCK_MODE;
    }

    if (slaveAppModeIsDryRun() || (command.command_flags & PACKET_FLAG_DRY_RUN)) {
        reasons |= UV_BLOCK_DRY_RUN;
    }

    if (!modeHasCapability(capability, MODE_CAP_X_SENSOR) ||
        !modeHasCapability(capability, MODE_CAP_X_MOTOR)) {
        reasons |= UV_BLOCK_X_INVALID;
    }

    if (!modeHasCapability(capability, MODE_CAP_Y_SENSOR) ||
        !modeHasCapability(capability, MODE_CAP_Y_MOTOR)) {
        reasons |= UV_BLOCK_Y_INVALID;
    }

    // 只有平滑目标接近命令目标，且实际角度足够接近目标角度，才认为光斑稳定到可落笔位置。
    const float smoothing_error_x_mm = fabsf(motion.target_x_mm - motion.smooth_x_mm);
    const float smoothing_error_y_mm = fabsf(motion.target_y_mm - motion.smooth_y_mm);
    if (smoothing_error_x_mm > kSlaveXTrajectory.settle_error_mm ||
        smoothing_error_y_mm > kSlaveYTrajectory.settle_error_mm) {
        reasons |= UV_BLOCK_NOT_SETTLED;
    }

    const float x_tracking_error_rad = fabsf(motion.target_angle_rad - motion.actual_angle_rad);
    const float y_tracking_error_rad =
        fabsf(motion.target_y_angle_rad - motion.actual_y_angle_rad);
    if (x_tracking_error_rad > kSlaveXAxis.tracking.settle_error_rad ||
        y_tracking_error_rad > kSlaveYAxis.tracking.settle_error_rad) {
        reasons |= UV_BLOCK_TRACKING;
    }

    return reasons;
}

bool isSlaveUvAllowed(uint32_t now_us) {
    return evaluateSlaveUvBlockReasons(now_us) == UV_BLOCK_NONE;
}

void runSlaveSafetyStep(uint32_t now_us) {
    // uv_allowed 是最终输出判定；motion.pen_req 是 planner 发布的 effective_pen_req，
    // 用于区分“没有要求落笔”和“要求落笔但联锁条件不满足”。当前 UV 硬件默认关闭时不锁存 UV fault，
    // 避免 SingleX 电机性能阶段被 UV 逻辑污染。
    static PenStateMachineState pen_state = {PEN_UP};
    const uint16_t block_reasons = evaluateSlaveUvBlockReasons(now_us);
    const bool uv_allowed = block_reasons == UV_BLOCK_NONE;
    const SlaveRtCommand command = snapshotSlaveRtCommand();
    const SlaveMotionSnapshot motion = snapshotSlaveMotion();
    const bool command_fresh = isSlaveRtCommandFresh(command, now_us);
    const bool effective_pen_req = command_fresh && motion.pen_req != 0;
    const uint8_t next_pen_state =
        updatePenStateMachine(pen_state,
                               {effective_pen_req,
                                 uv_allowed,
                                (getActiveFaultFlags() & ~static_cast<uint16_t>(FAULT_UV_INTERLOCK)) != 0});

    sysData.slave.uv_interlock_blocked =
        SLAVE_UV_HW_ENABLED &&
        command_fresh &&
        effective_pen_req &&
        !uv_allowed;
    sysData.slave.uv_block_reasons = block_reasons;
    sysData.slave.pen_state = next_pen_state;
    if (!command_fresh) {
        sysData.link.link_state = LINK_TIMEOUT;
    }

#if SLAVE_UV_HW_ENABLED
    if (sysData.slave.uv_interlock_blocked) {
        // 只有 UV 硬件阶段才锁存 UV_INTERLOCK，避免当前 SingleX 电机调试阶段被 UV fault 污染。
        addLocalFault(FAULT_UV_INTERLOCK);
    }
#endif

    // 唯一的 UV GPIO 写出口。任何 false 都会关闭两路 UV MOS。
    setUvOutput(uv_allowed && next_pen_state == PEN_DOWN);
}
