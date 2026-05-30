#include "slave/modes/mode_table.h"

#include "slave/config/build/slave_build_options.h"
#include "slave/config/core/slave_control_config.h"
#include "slave/modes/auto_draw/auto_draw_mode.h"
#include "slave/modes/ble/ble_mode.h"
#include "slave/modes/manual_draw/manual_draw_mode.h"
#include "slave/modes/mode_manager.h"

const char *slaveRunModeName() {
    switch (SLAVE_RUN_MODE) {
        case SLAVE_MODE_SINGLE_X_5KHZ_ID:
            return "SingleX_5kHz";
        case SLAVE_MODE_SINGLE_Y_5KHZ_ID:
            return "SingleY_5kHz";
        case SLAVE_MODE_DUAL_XY_2KHZ_ID:
            return "DualXY_2kHz";
        case SLAVE_MODE_DUAL_XY_DRY_RUN_ID:
            return "DualXY_DryRun";
        case SLAVE_MODE_YSENSOR_ONLY_ID:
            return "YSensorOnly";
        case SLAVE_MODE_Y_OPEN_LOOP_ID:
            return "YOpenLoop";
        case SLAVE_MODE_Y_CLOSED_LOOP_ID:
            return "YClosedLoop";
        default:
            return "Unknown";
    }
}

const char *slaveAppModeNameForCurrent() {
    return slaveAppModeName(currentSlaveAppMode());
}

ModeCapability slaveModeCapabilityForApp(SlaveAppMode app_mode) {
    switch (app_mode) {
        case SLAVE_APP_MODE_MANUAL_DRAW:
            return slaveManualDrawCapability(SLAVE_RUN_MODE);
        case SLAVE_APP_MODE_AUTO_DRAW:
            return slaveAppModeAvailable(SLAVE_APP_MODE_AUTO_DRAW)
                       ? slaveAutoDrawCapability(SLAVE_RUN_MODE)
                       : ModeCapability{};
        case SLAVE_APP_MODE_BLE_SAFE:
            return slaveBleSafeCapability();
        case SLAVE_APP_MODE_DIAGNOSTICS: {
            ModeCapability capability = {};
            capability.flags = MODE_CAP_NONE;
            capability.control_rate_hz =
                static_cast<uint16_t>(1000000UL / SLAVE_CONTROL_LOOP_PERIOD_US);
            capability.outer_rate_hz = 1000;
            return capability;
        }
        default:
            return ModeCapability{};
    }
}

ModeCapability slaveModeCapability() {
    return slaveModeCapabilityForApp(currentSlaveAppMode());
}
