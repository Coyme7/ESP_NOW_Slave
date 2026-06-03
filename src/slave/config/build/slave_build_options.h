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
// 说明：
// - SLAVE_RUN_MODE 是从机唯一硬件路径选择入口，同时派生默认控制周期。
// - *_HW_ENABLED 只表示编译期允许对象存在，不能单独决定初始化哪条硬件路径。
// - 模式名中的频率必须与 SLAVE_CONTROL_LOOP_PERIOD_US 保持一致。

// SingleX_5kHz：单 X 轴闭环压测，初始化 X sensor + X motor，控制周期 200us。
#define SLAVE_MODE_SINGLE_X_5KHZ_ID 0
// SingleY_5kHz：单 Y 轴闭环压测，初始化 Y sensor + Y motor，控制周期 200us。
#define SLAVE_MODE_SINGLE_Y_5KHZ_ID 1
// DualXY_2kHz：双轴真实硬件验收，初始化 X/Y sensor + motor，控制周期 500us。
#define SLAVE_MODE_DUAL_XY_2KHZ_ID 2
// DualXY_DryRun：双轴逻辑干跑，不初始化真实 motor，适合 AutoDraw 联调。
#define SLAVE_MODE_DUAL_XY_DRY_RUN_ID 3
// YSensorOnly：只初始化 Y sensor，用于编码器方向、零点和读数稳定性验证。
#define SLAVE_MODE_YSENSOR_ONLY_ID 4
// YOpenLoop：只初始化 Y motor 开环低压路径，不要求 Y sensor。
#define SLAVE_MODE_Y_OPEN_LOOP_ID 5
// YClosedLoop：初始化 Y sensor + Y motor，用于 Y 轴闭环 bring-up。
#define SLAVE_MODE_Y_CLOSED_LOOP_ID 6

enum SlaveRunMode : uint8_t {
    SLAVE_MODE_SINGLE_X_5KHZ = SLAVE_MODE_SINGLE_X_5KHZ_ID,
    SLAVE_MODE_SINGLE_Y_5KHZ = SLAVE_MODE_SINGLE_Y_5KHZ_ID,
    SLAVE_MODE_DUAL_XY_2KHZ = SLAVE_MODE_DUAL_XY_2KHZ_ID,
    SLAVE_MODE_DUAL_XY_DRY_RUN = SLAVE_MODE_DUAL_XY_DRY_RUN_ID,
    SLAVE_MODE_YSENSOR_ONLY = SLAVE_MODE_YSENSOR_ONLY_ID,
    SLAVE_MODE_Y_OPEN_LOOP = SLAVE_MODE_Y_OPEN_LOOP_ID,
    SLAVE_MODE_Y_CLOSED_LOOP = SLAVE_MODE_Y_CLOSED_LOOP_ID,
};

// 功能说明：选择从机 run mode、硬件初始化路径和默认控制周期。
// 默认：SingleX_5kHz；切 Y/DualXY 时需同步打开对应 Y 轴和双轴保险开关。
#ifndef SLAVE_RUN_MODE
#define SLAVE_RUN_MODE SLAVE_MODE_DUAL_XY_2KHZ_ID
#endif

// == 硬件编译开关 ===============================================================
//
// 说明：
// - 这些宏只表示真实外设对象是否参与编译、是否允许被 run mode 初始化。
// - 实际硬件路径只由 SLAVE_RUN_MODE 选择。

// 功能说明：启用 X 轴真实电机输出。
// 0：不编译 X 电机驱动对象；1：允许 run mode 初始化 X 电机。
#ifndef SLAVE_X_MOTOR_HW_ENABLED
#define SLAVE_X_MOTOR_HW_ENABLED 1
#endif

// 功能说明：启用 X 轴 MT6701 编码器。
// 0：不编译 X 编码器对象；1：允许 run mode 初始化 X 编码器。
#ifndef SLAVE_X_SENSOR_HW_ENABLED
#define SLAVE_X_SENSOR_HW_ENABLED 1
#endif

// 功能说明：启用 Y 轴真实电机输出。
// 0：不编译 Y 电机驱动对象；1：允许 run mode 初始化 Y 电机。
#ifndef SLAVE_Y_MOTOR_HW_ENABLED
#define SLAVE_Y_MOTOR_HW_ENABLED 1
#endif

// 功能说明：启用 Y 轴 MT6701 编码器。
// 0：不编译 Y 编码器对象；1：允许 run mode 初始化 Y 编码器。
#ifndef SLAVE_Y_SENSOR_HW_ENABLED
#define SLAVE_Y_SENSOR_HW_ENABLED 1
#endif

// 功能说明：启用真实 UV MOS 输出。
// 0：UV GPIO 保持安全关闭；1：允许安全联锁后驱动 UV MOS。
#ifndef SLAVE_UV_HW_ENABLED
#define SLAVE_UV_HW_ENABLED 0
#endif

// 功能说明：启用双轴真实硬件总保险。
// 0：禁止 DualXY 真实硬件模式；1：允许 X/Y 双轴真实硬件模式。
#ifndef SLAVE_DUAL_XY_HARDWARE_ENABLED
#define SLAVE_DUAL_XY_HARDWARE_ENABLED 1
#endif

// 功能说明：启用 MT6701 fast reader。
// 0：禁用 fast reader；1：控制热路径使用 fast reader。
#ifndef SLAVE_FAST_SENSOR_READER_ENABLED
#define SLAVE_FAST_SENSOR_READER_ENABLED 1
#endif

// == 启动应用模式 =============================================================
//
// 说明：
// - SLAVE_STARTUP_APP_MODE 只决定上电后的默认业务入口，不决定硬件路径。
// - 请求的 app 不可用时会回落到 ManualDraw，真实能力由 run mode 和 ENABLE 开关决定。

// ManualDraw：接收主机归一化坐标，按当前 run mode 可用轴执行。
#define SLAVE_STARTUP_APP_MANUAL_DRAW_ID 0
// AutoDraw：接收并执行主机轨迹段，只在 DualXY-capable run mode 下可用。
#define SLAVE_STARTUP_APP_AUTO_DRAW_ID 1
// BleSafe：BLE 占位安全模式，不接收远程绘图目标。
#define SLAVE_STARTUP_APP_BLE_SAFE_ID 2
// Diagnostics：诊断入口，占位给后续低频诊断流程。
#define SLAVE_STARTUP_APP_DIAGNOSTICS_ID 3

enum SlaveStartupAppMode : uint8_t {
    SLAVE_STARTUP_APP_MANUAL_DRAW = SLAVE_STARTUP_APP_MANUAL_DRAW_ID,
    SLAVE_STARTUP_APP_AUTO_DRAW = SLAVE_STARTUP_APP_AUTO_DRAW_ID,
    SLAVE_STARTUP_APP_BLE_SAFE = SLAVE_STARTUP_APP_BLE_SAFE_ID,
    SLAVE_STARTUP_APP_DIAGNOSTICS = SLAVE_STARTUP_APP_DIAGNOSTICS_ID,
};

// 功能说明：选择上电后的默认应用模式。
// 0：手动绘图；1：自动绘图；2：BLE 安全占位；3：诊断。
#ifndef SLAVE_STARTUP_APP_MODE
#define SLAVE_STARTUP_APP_MODE SLAVE_STARTUP_APP_MANUAL_DRAW_ID
#endif

// == 自动绘图 / 仿真 ==========================================================

// 功能说明：启用从机自动绘图轨迹执行。
// 0：不接收轨迹段、不进入 AutoDraw；1：允许接收主机轨迹并执行。
#ifndef SLAVE_AUTO_DRAW_ENABLED
#define SLAVE_AUTO_DRAW_ENABLED 0
#endif

// 功能说明：启用 Y 轴仿真跟随。
// 0：不模拟 Y 轴响应；1：Y 硬件关闭时用仿真位置参与状态和安全逻辑。
#ifndef SLAVE_Y_SIMULATION_ENABLED
#define SLAVE_Y_SIMULATION_ENABLED 1
#endif

// == 性能隔离 =================================================================
//
// 说明：
// - 这些模式只用于控制环耗时拆分，不改变 SLAVE_RUN_MODE 的硬件路径语义。
// - 常规固件应保持 FULL_CONTROL；其他值只用于台架定位热路径瓶颈。

// TimerEmpty：只运行定时器壳层，测量调度基线。
#define SLAVE_PERF_MODE_TIMER_EMPTY 0
// SensorOnly：只读传感器路径，测量编码器读取成本。
#define SLAVE_PERF_MODE_SENSOR_ONLY 1
// LoopFocOnly：只运行 loopFOC 路径，测量 FOC 计算成本。
#define SLAVE_PERF_MODE_LOOPFOC_ONLY 2
// MoveOnly：只运行 move 路径，测量目标更新成本。
#define SLAVE_PERF_MODE_MOVE_ONLY 3
// LoopFocMove：运行 loopFOC + move，不运行完整 planner。
#define SLAVE_PERF_MODE_LOOPFOC_MOVE 4
// FullControl：运行完整控制路径，默认测试和验收使用。
#define SLAVE_PERF_MODE_FULL_CONTROL 5

enum SlaveControlPerfMode : uint8_t {
    SLAVE_PERF_TIMER_EMPTY = SLAVE_PERF_MODE_TIMER_EMPTY,
    SLAVE_PERF_SENSOR_ONLY = SLAVE_PERF_MODE_SENSOR_ONLY,
    SLAVE_PERF_LOOPFOC_ONLY = SLAVE_PERF_MODE_LOOPFOC_ONLY,
    SLAVE_PERF_MOVE_ONLY = SLAVE_PERF_MODE_MOVE_ONLY,
    SLAVE_PERF_LOOPFOC_MOVE = SLAVE_PERF_MODE_LOOPFOC_MOVE,
    SLAVE_PERF_FULL_CONTROL = SLAVE_PERF_MODE_FULL_CONTROL,
};

// 功能说明：选择控制热路径性能隔离测量模式。
// 0-4：只运行指定子路径；5：运行完整控制路径。
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
