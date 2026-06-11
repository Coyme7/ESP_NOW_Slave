#pragma once

#include <stdint.h>

#include "common/math/axis_math.h"
#include "slave/config/build/slave_build_options.h"

constexpr bool slaveRunModeHasLogicalAxis(AxisId axis) {
    return axis == AXIS_X
               ? (SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_X_4KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_4KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_DRY_RUN_ID)
               : (SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_Y_4KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_4KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_DRY_RUN_ID);
}

constexpr bool slaveRunModeNeedsSensorHardware(AxisId axis) {
    return axis == AXIS_X
               ? (SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_X_4KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_4KHZ_ID)
               : (SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_Y_4KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_4KHZ_ID);
}

constexpr bool slaveRunModeNeedsMotorHardware(AxisId axis) {
    return axis == AXIS_X
               ? (SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_X_4KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_4KHZ_ID)
               : (SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_Y_4KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_4KHZ_ID);
}

constexpr bool slaveRunModeIsDualXYLogic() {
    return SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_4KHZ_ID ||
           SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_DRY_RUN_ID;
}

constexpr bool slaveRunModeAllowsUvHardware() {
    return SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_4KHZ_ID;
}

constexpr bool slaveRunModeRunsAxis(AxisId axis) {
    return slaveRunModeHasLogicalAxis(axis);
}

constexpr bool slaveRunModeDrivesAxis(AxisId axis) {
    return slaveRunModeNeedsMotorHardware(axis);
}

constexpr uint32_t slaveRunModeNominalPeriodUs() {
    return (SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_DRY_RUN_ID) ? 500UL : 250UL;
}
