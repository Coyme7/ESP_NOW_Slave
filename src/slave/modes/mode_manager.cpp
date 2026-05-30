#include "slave/modes/mode_manager.h"

#include <Arduino.h>

#include "common/protocol/protocol_types.h"
#include "slave/config/build/slave_build_options.h"
#include "slave/modes/mode_traits.h"

namespace {

portMUX_TYPE slaveRuntimeModeMux = portMUX_INITIALIZER_UNLOCKED;

constexpr bool startupModeSupportsAutoDraw() {
    return SLAVE_AUTO_DRAW_ENABLED && slaveRunModeIsDualXYLogic();
}

constexpr SlaveAppMode requestedAppModeFromStartupOption() {
    return (SLAVE_STARTUP_APP_MODE == SLAVE_STARTUP_APP_AUTO_DRAW_ID)
               ? SLAVE_APP_MODE_AUTO_DRAW
               : ((SLAVE_STARTUP_APP_MODE == SLAVE_STARTUP_APP_BLE_SAFE_ID)
                      ? SLAVE_APP_MODE_BLE_SAFE
                      : ((SLAVE_STARTUP_APP_MODE == SLAVE_STARTUP_APP_DIAGNOSTICS_ID)
                             ? SLAVE_APP_MODE_DIAGNOSTICS
                             : SLAVE_APP_MODE_MANUAL_DRAW));
}

constexpr SlaveAppMode defaultAppModeFromStartupProfile() {
    return (requestedAppModeFromStartupOption() == SLAVE_APP_MODE_AUTO_DRAW &&
            !startupModeSupportsAutoDraw())
               ? SLAVE_APP_MODE_MANUAL_DRAW
               : requestedAppModeFromStartupOption();
}

SlaveAppMode appModeFromProtocol(uint8_t protocol_mode, uint16_t command_flags) {
    switch (protocol_mode) {
        case MODE_DUALXY_DRAW_DRY_RUN:
        case MODE_DUALXY_DRAW_UV:
            return (command_flags & PACKET_FLAG_TRAJECTORY_ACTIVE)
                       ? SLAVE_APP_MODE_AUTO_DRAW
                       : SLAVE_APP_MODE_MANUAL_DRAW;
        case MODE_BLE_MEDIA:
            return SLAVE_APP_MODE_BLE_SAFE;
        case MODE_DUALXY_DRY_RUN:
        case MODE_COLLAB_DRAW:
        default:
            return SLAVE_APP_MODE_MANUAL_DRAW;
    }
}

bool appModeAvailableInternal(SlaveAppMode mode) {
    switch (mode) {
        case SLAVE_APP_MODE_MANUAL_DRAW:
        case SLAVE_APP_MODE_BLE_SAFE:
        case SLAVE_APP_MODE_DIAGNOSTICS:
            return true;
        case SLAVE_APP_MODE_AUTO_DRAW:
            return startupModeSupportsAutoDraw();
        default:
            return false;
    }
}

uint8_t protocolModeForActiveApp(SlaveAppMode active_mode,
                                 uint8_t requested_protocol_mode) {
    switch (active_mode) {
        case SLAVE_APP_MODE_AUTO_DRAW:
            return (requested_protocol_mode == MODE_DUALXY_DRAW_UV)
                       ? MODE_DUALXY_DRAW_UV
                       : MODE_DUALXY_DRAW_DRY_RUN;
        case SLAVE_APP_MODE_BLE_SAFE:
            return MODE_BLE_MEDIA;
        case SLAVE_APP_MODE_DIAGNOSTICS:
            return MODE_DUALXY_DRY_RUN;
        case SLAVE_APP_MODE_MANUAL_DRAW:
        default:
            return (requested_protocol_mode == MODE_DUALXY_DRY_RUN)
                       ? MODE_DUALXY_DRY_RUN
                       : MODE_COLLAB_DRAW;
    }
}

SlaveRuntimeModeSnapshot slaveRuntimeModeState = {
    static_cast<uint8_t>(defaultAppModeFromStartupProfile()),
    static_cast<uint8_t>(defaultAppModeFromStartupProfile()),
    1U,
    0U,
    MODE_COLLAB_DRAW,
    PACKET_FLAG_NONE,
    0UL,
    0UL,
};

}  // namespace

SlaveAppMode slaveDefaultAppMode() {
    return defaultAppModeFromStartupProfile();
}

bool slaveAppModeAvailable(SlaveAppMode mode) {
    return appModeAvailableInternal(mode);
}

uint8_t slaveAcceptedProtocolModeForCommand(uint8_t protocol_mode,
                                            uint16_t command_flags) {
    const SlaveAppMode requested = appModeFromProtocol(protocol_mode, command_flags);
    const SlaveAppMode active = appModeAvailableInternal(requested)
                                    ? requested
                                    : SLAVE_APP_MODE_MANUAL_DRAW;
    return protocolModeForActiveApp(active, protocol_mode);
}

void updateSlaveRuntimeModeFromCommand(uint8_t protocol_mode,
                                       uint16_t command_flags,
                                       uint32_t now_ms) {
    const SlaveAppMode requested = appModeFromProtocol(protocol_mode, command_flags);
    const bool accepted = appModeAvailableInternal(requested);
    const SlaveAppMode active = accepted ? requested : SLAVE_APP_MODE_MANUAL_DRAW;

    portENTER_CRITICAL(&slaveRuntimeModeMux);
    slaveRuntimeModeState.requested_mode = static_cast<uint8_t>(requested);
    slaveRuntimeModeState.active_mode = static_cast<uint8_t>(active);
    slaveRuntimeModeState.request_accepted = accepted ? 1U : 0U;
    slaveRuntimeModeState.request_rejected = accepted ? 0U : 1U;
    slaveRuntimeModeState.last_protocol_mode = protocol_mode;
    slaveRuntimeModeState.last_command_flags = command_flags;
    slaveRuntimeModeState.request_count++;
    slaveRuntimeModeState.last_change_ms = now_ms;
    portEXIT_CRITICAL(&slaveRuntimeModeMux);
}

SlaveRuntimeModeSnapshot getSlaveRuntimeModeSnapshot() {
    SlaveRuntimeModeSnapshot snapshot = {};
    portENTER_CRITICAL(&slaveRuntimeModeMux);
    snapshot = slaveRuntimeModeState;
    portEXIT_CRITICAL(&slaveRuntimeModeMux);
    return snapshot;
}

SlaveAppMode currentSlaveAppMode() {
    const SlaveRuntimeModeSnapshot runtime = getSlaveRuntimeModeSnapshot();
    return static_cast<SlaveAppMode>(runtime.active_mode);
}

const char *slaveAppModeName(SlaveAppMode mode) {
    switch (mode) {
        case SLAVE_APP_MODE_MANUAL_DRAW:
            return "ManualDraw";
        case SLAVE_APP_MODE_AUTO_DRAW:
            return "AutoDraw";
        case SLAVE_APP_MODE_BLE_SAFE:
            return "BleSafe";
        case SLAVE_APP_MODE_DIAGNOSTICS:
            return "Diagnostics";
        default:
            return "Unknown";
    }
}
