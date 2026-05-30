#pragma once

#include <stdint.h>

#include "common/protocol/protocol_types.h"
#include "slave/control/slave_runtime_snapshot.h"
#include "slave/modes/mode_traits.h"

constexpr bool slaveProtocolModeAllowsRemoteTarget(uint8_t protocol_mode) {
    return protocol_mode == MODE_COLLAB_DRAW ||
           protocol_mode == MODE_DUALXY_DRY_RUN ||
           protocol_mode == MODE_DUALXY_DRAW_DRY_RUN ||
           protocol_mode == MODE_DUALXY_DRAW_UV;
}

bool slaveAppModeIsDrawMode();
bool slaveAppModeIsDryRun();
bool slaveCommandRequestsTrajectory(const SlaveRtCommand &command);
bool slaveModeAllowsUv(const SlaveRtCommand &command);
