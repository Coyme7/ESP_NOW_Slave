#pragma once

#include <stdint.h>

#include "common/math/axis_math.h"
#include "slave/config/build/slave_build_options.h"

// run mode 是从机唯一硬件路径选择来源。
// *_HW_ENABLED 只表示编译期是否允许对象存在，不能单独决定初始化路径。
constexpr bool slaveRunModeHasLogicalAxis(AxisId axis) {
    return axis == AXIS_X
               ? (SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_X_5KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_4KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_DRY_RUN_ID)
               : (SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_Y_5KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_4KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_DRY_RUN_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_YSENSOR_ONLY_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_Y_OPEN_LOOP_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_Y_CLOSED_LOOP_ID);
}

constexpr bool slaveRunModeNeedsSensorHardware(AxisId axis) {
    return axis == AXIS_X
               ? (SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_X_5KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_4KHZ_ID)
               : (SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_Y_5KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_4KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_YSENSOR_ONLY_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_Y_CLOSED_LOOP_ID);
}

constexpr bool slaveRunModeNeedsMotorHardware(AxisId axis) {
    return axis == AXIS_X
               ? (SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_X_5KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_4KHZ_ID)
               : (SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_Y_5KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_4KHZ_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_Y_OPEN_LOOP_ID ||
                  SLAVE_RUN_MODE == SLAVE_MODE_Y_CLOSED_LOOP_ID);
}

constexpr bool slaveRunModeUsesOpenLoopMotor(AxisId axis) {
    return axis == AXIS_Y && SLAVE_RUN_MODE == SLAVE_MODE_Y_OPEN_LOOP_ID;
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
    return (SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_X_5KHZ_ID ||
            SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_Y_5KHZ_ID)
               ? 200UL
               : (SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_4KHZ_ID ? 250UL : 500UL);
}
