#pragma once

#include <stdint.h>

#include "sdkconfig.h"

// == 构建保护 =================================================================

#if defined(BOARD_HAS_PSRAM) || \
    (defined(CONFIG_ESP32S3_SPIRAM_SUPPORT) && CONFIG_ESP32S3_SPIRAM_SUPPORT) || \
    (defined(CONFIG_SPIRAM_SUPPORT) && CONFIG_SPIRAM_SUPPORT) || \
    (defined(CONFIG_SPIRAM) && CONFIG_SPIRAM)
#error "Slave firmware must not declare or initialize PSRAM."
#endif

// == 运行能力模式 =============================================================
//
// SLAVE_RUN_MODE 是从机唯一硬件路径选择入口，同时派生默认控制周期。
// 所有真实闭环模式统一 4kHz / 250us；独立 Y 轴 bring-up 路径已删除。

#define SLAVE_MODE_SINGLE_X_4KHZ_ID 0
#define SLAVE_MODE_SINGLE_Y_4KHZ_ID 1
#define SLAVE_MODE_DUAL_XY_4KHZ_ID 2
#define SLAVE_MODE_DUAL_XY_DRY_RUN_ID 3

enum SlaveRunMode : uint8_t {
    SLAVE_MODE_SINGLE_X_4KHZ = SLAVE_MODE_SINGLE_X_4KHZ_ID,
    SLAVE_MODE_SINGLE_Y_4KHZ = SLAVE_MODE_SINGLE_Y_4KHZ_ID,
    SLAVE_MODE_DUAL_XY_4KHZ = SLAVE_MODE_DUAL_XY_4KHZ_ID,
    SLAVE_MODE_DUAL_XY_DRY_RUN = SLAVE_MODE_DUAL_XY_DRY_RUN_ID,
};

#ifndef SLAVE_RUN_MODE
#define SLAVE_RUN_MODE SLAVE_MODE_DUAL_XY_4KHZ_ID
#endif

// == 启动应用模式 =============================================================
//
// SLAVE_STARTUP_APP_MODE 只决定上电后的默认业务入口，不决定硬件路径。

#define SLAVE_STARTUP_APP_MANUAL_DRAW_ID 0
#define SLAVE_STARTUP_APP_AUTO_DRAW_ID 1
#define SLAVE_STARTUP_APP_BLE_SAFE_ID 2
#define SLAVE_STARTUP_APP_DIAGNOSTICS_ID 3

#ifndef SLAVE_STARTUP_APP_MODE
#define SLAVE_STARTUP_APP_MODE SLAVE_STARTUP_APP_MANUAL_DRAW_ID
#endif

// == 硬件编译开关 ==============================================================

#ifndef SLAVE_X_MOTOR_HW_ENABLED
#define SLAVE_X_MOTOR_HW_ENABLED 1
#endif

#ifndef SLAVE_X_SENSOR_HW_ENABLED
#define SLAVE_X_SENSOR_HW_ENABLED 1
#endif

#ifndef SLAVE_Y_MOTOR_HW_ENABLED
#define SLAVE_Y_MOTOR_HW_ENABLED 0
#endif

#ifndef SLAVE_Y_SENSOR_HW_ENABLED
#define SLAVE_Y_SENSOR_HW_ENABLED 0
#endif

#ifndef SLAVE_UV_HW_ENABLED
#define SLAVE_UV_HW_ENABLED 0
#endif

#ifndef SLAVE_DUAL_XY_HARDWARE_ENABLED
#define SLAVE_DUAL_XY_HARDWARE_ENABLED 1
#endif

#ifndef SLAVE_FAST_SENSOR_READER_ENABLED
#define SLAVE_FAST_SENSOR_READER_ENABLED 1
#endif

#ifndef SLAVE_ENABLE_CURRENT_SENSE
#define SLAVE_ENABLE_CURRENT_SENSE 1
#endif

#ifndef SLAVE_ENABLE_CURRENT_SENSE_DIAG_TEST
#define SLAVE_ENABLE_CURRENT_SENSE_DIAG_TEST 0
#endif

#ifndef SLAVE_ENABLE_ZERO_CURRENT_TEST
#define SLAVE_ENABLE_ZERO_CURRENT_TEST 0
#endif

// == FOC 快速启动 ==============================================================

#ifndef SLAVE_SKIP_FOC_ALIGNMENT_ON_STARTUP
#define SLAVE_SKIP_FOC_ALIGNMENT_ON_STARTUP 0
#endif

#ifndef SLAVE_X_FOC_SENSOR_DIRECTION
#define SLAVE_X_FOC_SENSOR_DIRECTION 1
#endif

#ifndef SLAVE_Y_FOC_SENSOR_DIRECTION
#define SLAVE_Y_FOC_SENSOR_DIRECTION 1
#endif

#ifndef SLAVE_X_ZERO_ELECTRIC_ANGLE_RAD
#define SLAVE_X_ZERO_ELECTRIC_ANGLE_RAD 0.0f
#endif

#ifndef SLAVE_Y_ZERO_ELECTRIC_ANGLE_RAD
#define SLAVE_Y_ZERO_ELECTRIC_ANGLE_RAD 0.0f
#endif

// == 自动绘图 / 仿真 ==========================================================

#ifndef SLAVE_AUTO_DRAW_ENABLED
#define SLAVE_AUTO_DRAW_ENABLED 0
#endif

#ifndef SLAVE_Y_SIMULATION_ENABLED
#define SLAVE_Y_SIMULATION_ENABLED 1
#endif

// == 性能隔离 =================================================================

#define SLAVE_PERF_MODE_TIMER_EMPTY 0
#define SLAVE_PERF_MODE_SENSOR_ONLY 1
#define SLAVE_PERF_MODE_LOOPFOC_ONLY 2
#define SLAVE_PERF_MODE_MOVE_ONLY 3
#define SLAVE_PERF_MODE_LOOPFOC_MOVE 4
#define SLAVE_PERF_MODE_FULL_CONTROL 5

enum SlaveControlPerfMode : uint8_t {
    SLAVE_PERF_TIMER_EMPTY = SLAVE_PERF_MODE_TIMER_EMPTY,
    SLAVE_PERF_SENSOR_ONLY = SLAVE_PERF_MODE_SENSOR_ONLY,
    SLAVE_PERF_LOOPFOC_ONLY = SLAVE_PERF_MODE_LOOPFOC_ONLY,
    SLAVE_PERF_MOVE_ONLY = SLAVE_PERF_MODE_MOVE_ONLY,
    SLAVE_PERF_LOOPFOC_MOVE = SLAVE_PERF_MODE_LOOPFOC_MOVE,
    SLAVE_PERF_FULL_CONTROL = SLAVE_PERF_MODE_FULL_CONTROL,
};

#ifndef SLAVE_CONTROL_PERF_MODE
#define SLAVE_CONTROL_PERF_MODE SLAVE_PERF_MODE_FULL_CONTROL
#endif

inline const char *slaveControlPerfModeName() {
    switch (SLAVE_CONTROL_PERF_MODE) {
        case SLAVE_PERF_TIMER_EMPTY:
            return "TIMER_EMPTY";
        case SLAVE_PERF_SENSOR_ONLY:
            return "SENSOR_ONLY";
        case SLAVE_PERF_LOOPFOC_ONLY:
            return "LOOPFOC_ONLY";
        case SLAVE_PERF_MOVE_ONLY:
            return "MOVE_ONLY";
        case SLAVE_PERF_LOOPFOC_MOVE:
            return "LOOPFOC_MOVE";
        case SLAVE_PERF_FULL_CONTROL:
            return "FULL_CONTROL";
        default:
            return "Unknown";
    }
}
