#pragma once

#include <stdint.h>

#include "slave/config/build/slave_build_options.h"

// 从机控制周期与热路径分频配置。
// SingleX/SingleY 5kHz 默认 200us；真实 DualXY 4kHz 默认 250us；DryRun 和 Y-only 默认 500us。

// 从机本地控制周期，单位 us。
// 手动覆盖后由 slave_config_validate.h 检查是否和 run mode 名称一致。
#ifndef SLAVE_CONTROL_LOOP_PERIOD_US_CONFIG
#if SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_X_5KHZ_ID || \
    SLAVE_RUN_MODE == SLAVE_MODE_SINGLE_Y_5KHZ_ID
#define SLAVE_CONTROL_LOOP_PERIOD_US_CONFIG 200UL
#elif SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_4KHZ_ID
#define SLAVE_CONTROL_LOOP_PERIOD_US_CONFIG 250UL
#else
#define SLAVE_CONTROL_LOOP_PERIOD_US_CONFIG 500UL
#endif
#endif
static constexpr uint32_t SLAVE_CONTROL_LOOP_PERIOD_US = SLAVE_CONTROL_LOOP_PERIOD_US_CONFIG;
static constexpr uint32_t CONTROL_LOOP_PERIOD_US = SLAVE_CONTROL_LOOP_PERIOD_US;

// planner 执行分频。
// SLAVE_FAST_PLANNER_ENABLED=1 时每个 motor tick 运行一次；置 0 可回退到原 2 tick 分频。
#ifndef SLAVE_FAST_PLANNER_ENABLED
#if SLAVE_RUN_MODE == SLAVE_MODE_DUAL_XY_4KHZ_ID
#define SLAVE_FAST_PLANNER_ENABLED 0
#else
#define SLAVE_FAST_PLANNER_ENABLED 1
#endif
#endif

// 每 N 个 motor tick 运行一次。
#ifndef SLAVE_PLANNER_EVERY_N_STEPS
#if SLAVE_FAST_PLANNER_ENABLED
#define SLAVE_PLANNER_EVERY_N_STEPS 1UL
#else
#define SLAVE_PLANNER_EVERY_N_STEPS 2UL
#endif
#endif

// runtime 状态发布分频。
// 每 N 个 motor tick 更新一次 sysData 运行态字段。
#ifndef SLAVE_RUNTIME_PUBLISH_EVERY_N_STEPS
#define SLAVE_RUNTIME_PUBLISH_EVERY_N_STEPS 4UL
#endif

// motion snapshot 发布分频。
// 每 N 个 motor tick 更新一次安全和遥测快照。
#ifndef SLAVE_MOTION_SNAPSHOT_EVERY_N_STEPS
#define SLAVE_MOTION_SNAPSHOT_EVERY_N_STEPS 4UL
#endif

// X 轴 loopFOC 分频。
// 默认每个 motor tick 都运行。
#ifndef SLAVE_X_FOC_EVERY_N_STEPS
#define SLAVE_X_FOC_EVERY_N_STEPS 1UL
#endif

// X 轴 move 分频。
// 默认每个 motor tick 都写入最近目标。
#ifndef SLAVE_X_MOVE_EVERY_N_STEPS
#define SLAVE_X_MOVE_EVERY_N_STEPS 1UL
#endif

// Y 轴 loopFOC 分频。
// 仅在 run mode 选择 Y 闭环硬件路径时生效。
#ifndef SLAVE_Y_FOC_EVERY_N_STEPS
#define SLAVE_Y_FOC_EVERY_N_STEPS 1UL
#endif

// Y 轴 move 分频。
// 仅在 run mode 选择 Y 电机硬件路径时生效。
#ifndef SLAVE_Y_MOVE_EVERY_N_STEPS
#define SLAVE_Y_MOVE_EVERY_N_STEPS 1UL
#endif
