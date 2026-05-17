#pragma once

#include <stdint.h>

// 从机运行模式。默认 SingleXSync 只运行当前已验证的 X 单轴同步链路。
enum SlaveRunMode : uint8_t {
    SLAVE_MODE_SINGLE_X_SYNC = 0,
    SLAVE_MODE_SINGLE_Y_SYNC = 1,
    SLAVE_MODE_DUAL_XY_FRAME = 2,
    SLAVE_MODE_DUAL_XY_HW = 3,
};

// Y 轴 bring-up 入口。默认关闭；每一步都必须由 build_flags 或本文件显式打开。
enum SlaveYBringupMode : uint8_t {
    SLAVE_Y_BRINGUP_DISABLED = 0,
    SLAVE_Y_BRINGUP_SENSOR_ONLY = 1,
    SLAVE_Y_BRINGUP_MOTOR_OPEN_LOOP = 2,
    SLAVE_Y_BRINGUP_CLOSED_LOOP = 3,
};

// 从机 5kHz 性能隔离模式数值。使用独立宏是为了让 #if 能在编译期裁剪隔离路径。
#define SLAVE_PERF_MODE_TIMER_EMPTY 0
#define SLAVE_PERF_MODE_SENSOR_ONLY 1
#define SLAVE_PERF_MODE_LOOPFOC_ONLY 2
#define SLAVE_PERF_MODE_MOVE_ONLY 3
#define SLAVE_PERF_MODE_LOOPFOC_MOVE 4
#define SLAVE_PERF_MODE_FULL_CONTROL 5

// 从机 5kHz 性能隔离模式。默认 FULL_CONTROL 运行完整工程路径；其它模式只用于定位 timer、
// 传感器、loopFOC、move 的独立开销，不能作为正常同步配置长期使用。
enum SlaveControlPerfMode : uint8_t {
    SLAVE_PERF_TIMER_EMPTY = SLAVE_PERF_MODE_TIMER_EMPTY,
    SLAVE_PERF_SENSOR_ONLY = SLAVE_PERF_MODE_SENSOR_ONLY,
    SLAVE_PERF_LOOPFOC_ONLY = SLAVE_PERF_MODE_LOOPFOC_ONLY,
    SLAVE_PERF_MOVE_ONLY = SLAVE_PERF_MODE_MOVE_ONLY,
    SLAVE_PERF_LOOPFOC_MOVE = SLAVE_PERF_MODE_LOOPFOC_MOVE,
    SLAVE_PERF_FULL_CONTROL = SLAVE_PERF_MODE_FULL_CONTROL,
};

// 分类配置依赖入口。
// build_options 定义枚举和跨分类静态检查；具体默认值仍在 hardware/log/control 等分类文件中说明。
#include "slave/config/slave_hardware_config.h"
#include "slave/config/slave_log_config.h"
#include "slave/config/slave_control_config.h"

// 从机控制性能隔离模式。默认 FULL_CONTROL 保持 X 单轴同步路径；临时压测时可在 build_flags 中覆盖。
#ifndef SLAVE_CONTROL_PERF_MODE
#define SLAVE_CONTROL_PERF_MODE SLAVE_PERF_MODE_FULL_CONTROL
#endif

static_assert(SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_X_SYNC ||
                  SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_Y_SYNC ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_FRAME ||
                  SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_HW,
              "invalid SLAVE_RUN_MODE");

static_assert(SLAVE_Y_BRINGUP_MODE == SLAVE_Y_BRINGUP_DISABLED ||
                  SLAVE_Y_BRINGUP_MODE == SLAVE_Y_BRINGUP_SENSOR_ONLY ||
                  SLAVE_Y_BRINGUP_MODE == SLAVE_Y_BRINGUP_MOTOR_OPEN_LOOP ||
                  SLAVE_Y_BRINGUP_MODE == SLAVE_Y_BRINGUP_CLOSED_LOOP,
              "invalid SLAVE_Y_BRINGUP_MODE");
static_assert(SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_TIMER_EMPTY ||
                  SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_SENSOR_ONLY ||
                  SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_LOOPFOC_ONLY ||
                  SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_MOVE_ONLY ||
                  SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_LOOPFOC_MOVE ||
                  SLAVE_CONTROL_PERF_MODE == SLAVE_PERF_FULL_CONTROL,
              "invalid SLAVE_CONTROL_PERF_MODE");
static_assert(SLAVE_PLANNER_EVERY_N_STEPS > 0,
              "SLAVE_PLANNER_EVERY_N_STEPS must be greater than 0");
static_assert(SLAVE_RUNTIME_PUBLISH_EVERY_N_STEPS > 0,
              "SLAVE_RUNTIME_PUBLISH_EVERY_N_STEPS must be greater than 0");
static_assert(SLAVE_MOTION_SNAPSHOT_EVERY_N_STEPS > 0,
              "SLAVE_MOTION_SNAPSHOT_EVERY_N_STEPS must be greater than 0");
static_assert(SLAVE_X_MOVE_EVERY_N_STEPS > 0,
              "SLAVE_X_MOVE_EVERY_N_STEPS must be greater than 0");

static_assert(!(SLAVE_X_MOTOR_HW_ENABLED && !SLAVE_X_SENSOR_HW_ENABLED),
              "SLAVE_X_MOTOR_HW_ENABLED requires SLAVE_X_SENSOR_HW_ENABLED");
static_assert(!(SLAVE_Y_MOTOR_HW_ENABLED && !SLAVE_Y_SENSOR_HW_ENABLED),
              "SLAVE_Y_MOTOR_HW_ENABLED requires SLAVE_Y_SENSOR_HW_ENABLED");
static_assert(!(SLAVE_Y_BRINGUP_MODE == SLAVE_Y_BRINGUP_SENSOR_ONLY && !SLAVE_Y_SENSOR_HW_ENABLED),
              "YSensorOnly requires SLAVE_Y_SENSOR_HW_ENABLED");
static_assert(!(SLAVE_Y_BRINGUP_MODE == SLAVE_Y_BRINGUP_MOTOR_OPEN_LOOP &&
                !(SLAVE_Y_SENSOR_HW_ENABLED && SLAVE_Y_MOTOR_HW_ENABLED)),
              "YMotorOpenLoop requires Y sensor and Y motor hardware");
static_assert(!(SLAVE_Y_BRINGUP_MODE == SLAVE_Y_BRINGUP_CLOSED_LOOP &&
                !(SLAVE_Y_SENSOR_HW_ENABLED && SLAVE_Y_MOTOR_HW_ENABLED)),
              "YClosedLoop requires Y sensor and Y motor hardware");
static_assert(!(SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_HW && !SLAVE_DUAL_XY_HARDWARE_ENABLED),
              "SLAVE_MODE_DUAL_XY_HW requires SLAVE_DUAL_XY_HARDWARE_ENABLED");
static_assert(!(SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_HW &&
                !(SLAVE_X_MOTOR_HW_ENABLED && SLAVE_X_SENSOR_HW_ENABLED &&
                  SLAVE_Y_MOTOR_HW_ENABLED && SLAVE_Y_SENSOR_HW_ENABLED)),
              "DualXYHardware requires X/Y motor and sensor hardware");
static_assert(!(SLAVE_Y_BRINGUP_MODE == SLAVE_Y_BRINGUP_SENSOR_ONLY &&
                (SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_Y_SYNC ||
                 SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_HW ||
                 SLAVE_Y_BRINGUP_MODE == SLAVE_Y_BRINGUP_CLOSED_LOOP)),
              "YSensorOnly must stay out of Y closed-loop or DualXYHardware modes");
static_assert(!(SLAVE_Y_BRINGUP_MODE == SLAVE_Y_BRINGUP_SENSOR_ONLY &&
                SLAVE_X_MOTOR_HW_ENABLED),
              "YSensorOnly shares the SPI bus with X closed loop; disable X motor or add explicit SPI arbitration");
static_assert(!(SLAVE_AUTO_DRAW_ENABLED && SLAVE_RUN_MODE != SLAVE_MODE_DUAL_XY_HW),
              "SLAVE_AUTO_DRAW_ENABLED requires DualXYHardware mode");
static_assert(SLAVE_FAST_SENSOR_READER_ENABLED,
              "从机控制热路径必须使用 MT6701 fast reader，禁止回退慢速读取器");

inline const char *slaveRunModeName() {
    switch (SLAVE_RUN_MODE) {
        case SLAVE_MODE_SINGLE_X_SYNC:
            return "SingleXSync";
        case SLAVE_MODE_SINGLE_Y_SYNC:
            return "SingleYSync";
        case SLAVE_MODE_DUAL_XY_FRAME:
            return "DualXYFramework";
        case SLAVE_MODE_DUAL_XY_HW:
            return "DualXYHardware";
        default:
            return "Unknown";
    }
}

inline const char *slaveYBringupModeName() {
    switch (SLAVE_Y_BRINGUP_MODE) {
        case SLAVE_Y_BRINGUP_DISABLED:
            return "Disabled";
        case SLAVE_Y_BRINGUP_SENSOR_ONLY:
            return "YSensorOnly";
        case SLAVE_Y_BRINGUP_MOTOR_OPEN_LOOP:
            return "YMotorOpenLoop";
        case SLAVE_Y_BRINGUP_CLOSED_LOOP:
            return "YClosedLoop";
        default:
            return "Unknown";
    }
}

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
