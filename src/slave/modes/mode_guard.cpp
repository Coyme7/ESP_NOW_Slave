#include "slave/modes/mode_guard.h"

#include "common/app/mode_capability.h"
#include "slave/config/build/slave_build_options.h"
#include "slave/modes/mode_manager.h"
#include "slave/modes/mode_table.h"

bool slaveAppModeIsDrawMode() {
    return currentSlaveAppMode() == SLAVE_APP_MODE_AUTO_DRAW;
}

bool slaveAppModeIsDryRun() {
    const ModeCapability capability = slaveModeCapability();
    return modeHasCapability(capability, MODE_CAP_DRY_RUN);
}

bool slaveCommandRequestsTrajectory(const SlaveRtCommand &command) {
    if (command.valid == 0 || !slaveAppModeIsDrawMode()) {
        return false;
    }
    if ((command.command_flags & PACKET_FLAG_TRAJECTORY_ACTIVE) == 0) {
        return false;
    }
    return command.mode == MODE_DUALXY_DRAW_DRY_RUN ||
           command.mode == MODE_DUALXY_DRAW_UV;
}

bool slaveModeAllowsUv(const SlaveRtCommand &command) {
    const ModeCapability capability = slaveModeCapability();
    return SLAVE_UV_HW_ENABLED &&
           currentSlaveAppMode() == SLAVE_APP_MODE_AUTO_DRAW &&
           modeHasCapability(capability, MODE_CAP_UV) &&
           command.mode == MODE_DUALXY_DRAW_UV &&
           (command.command_flags & PACKET_FLAG_DRY_RUN) == 0;
}
