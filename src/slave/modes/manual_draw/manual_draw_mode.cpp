#include "slave/modes/manual_draw/manual_draw_mode.h"

#include "slave/config/build/slave_build_options.h"
#include "slave/config/core/slave_comm_config.h"
#include "slave/config/core/slave_control_config.h"

namespace {

constexpr uint16_t slaveRemoteCommandFlags() {
    return SLAVE_ESPNOW_ENABLED ? MODE_CAP_REMOTE_COMMAND : MODE_CAP_NONE;
}

constexpr uint16_t slaveAxisHardwareFlags(bool sensor_enabled, bool motor_enabled,
                                          uint16_t sensor_flag, uint16_t motor_flag) {
    return (sensor_enabled ? sensor_flag : static_cast<uint16_t>(MODE_CAP_NONE)) |
           (motor_enabled ? motor_flag : static_cast<uint16_t>(MODE_CAP_NONE));
}

}  // namespace

ModeCapability slaveManualDrawCapability(uint8_t run_mode) {
    ModeCapability capability = {};
    capability.control_rate_hz =
        static_cast<uint16_t>(1000000UL / SLAVE_CONTROL_LOOP_PERIOD_US);
    capability.outer_rate_hz = 1000;

    const uint16_t x_hw = slaveAxisHardwareFlags(SLAVE_X_SENSOR_HW_ENABLED,
                                                 SLAVE_X_MOTOR_HW_ENABLED,
                                                 MODE_CAP_X_SENSOR,
                                                 MODE_CAP_X_MOTOR);
    const uint16_t y_hw = slaveAxisHardwareFlags(SLAVE_Y_SENSOR_HW_ENABLED,
                                                 SLAVE_Y_MOTOR_HW_ENABLED,
                                                 MODE_CAP_Y_SENSOR,
                                                 MODE_CAP_Y_MOTOR);

    switch (run_mode) {
        case SLAVE_MODE_SINGLE_X_5KHZ_ID:
            capability.flags = x_hw | slaveRemoteCommandFlags();
            break;
        case SLAVE_MODE_SINGLE_Y_5KHZ_ID:
            capability.flags = y_hw | slaveRemoteCommandFlags();
            break;
        case SLAVE_MODE_DUAL_XY_2KHZ_ID:
            capability.flags = x_hw | y_hw | slaveRemoteCommandFlags() | MODE_CAP_PEN;
            if (SLAVE_UV_HW_ENABLED) {
                capability.flags |= MODE_CAP_UV;
            }
            break;
        case SLAVE_MODE_DUAL_XY_DRY_RUN_ID:
            capability.flags = slaveRemoteCommandFlags() | MODE_CAP_PEN | MODE_CAP_DRY_RUN;
            break;
        case SLAVE_MODE_YSENSOR_ONLY_ID:
            capability.flags = (SLAVE_Y_SENSOR_HW_ENABLED ? MODE_CAP_Y_SENSOR : MODE_CAP_NONE) |
                               MODE_CAP_DRY_RUN;
            break;
        case SLAVE_MODE_Y_OPEN_LOOP_ID:
            capability.flags = (SLAVE_Y_MOTOR_HW_ENABLED ? MODE_CAP_Y_MOTOR : MODE_CAP_NONE) |
                               MODE_CAP_DRY_RUN;
            break;
        case SLAVE_MODE_Y_CLOSED_LOOP_ID:
            capability.flags = y_hw | MODE_CAP_DRY_RUN;
            break;
        default:
            capability.flags = MODE_CAP_NONE;
            break;
    }
    return capability;
}
