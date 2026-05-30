#include "slave/modes/auto_draw/auto_draw_mode.h"

#include "slave/config/build/slave_build_options.h"
#include "slave/config/core/slave_comm_config.h"
#include "slave/config/core/slave_control_config.h"

namespace {

constexpr uint16_t slaveRemoteTrajectoryFlags() {
    return SLAVE_ESPNOW_ENABLED
               ? (MODE_CAP_REMOTE_COMMAND | MODE_CAP_PEN | MODE_CAP_TRAJECTORY)
               : MODE_CAP_NONE;
}

}  // namespace

ModeCapability slaveAutoDrawCapability(uint8_t run_mode) {
    ModeCapability capability = {};
    capability.control_rate_hz =
        static_cast<uint16_t>(1000000UL / SLAVE_CONTROL_LOOP_PERIOD_US);
    capability.outer_rate_hz = 1000;

    switch (run_mode) {
        case SLAVE_MODE_DUAL_XY_DRY_RUN_ID:
            capability.flags = slaveRemoteTrajectoryFlags() | MODE_CAP_DRY_RUN;
            break;
        case SLAVE_MODE_DUAL_XY_2KHZ_ID:
            capability.flags = slaveRemoteTrajectoryFlags() |
                               (SLAVE_X_SENSOR_HW_ENABLED ? MODE_CAP_X_SENSOR : MODE_CAP_NONE) |
                               (SLAVE_Y_SENSOR_HW_ENABLED ? MODE_CAP_Y_SENSOR : MODE_CAP_NONE) |
                               (SLAVE_X_MOTOR_HW_ENABLED ? MODE_CAP_X_MOTOR : MODE_CAP_NONE) |
                               (SLAVE_Y_MOTOR_HW_ENABLED ? MODE_CAP_Y_MOTOR : MODE_CAP_NONE);
            if (SLAVE_UV_HW_ENABLED) {
                capability.flags |= MODE_CAP_UV;
            }
            break;
        default:
            capability.flags = MODE_CAP_NONE;
            break;
    }
    return capability;
}
