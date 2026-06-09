#pragma once

#include <stdint.h>

#include "slave/config/build/slave_build_options.h"

// 从机日志与 timing 诊断配置。
// 日志只允许在启动路径和 Core 0 状态任务输出，禁止进入控制热路径。

// 功能说明：启用启动配置日志。
// 0：启动时不打印配置摘要；1：打印模式、硬件开关、信道和分频。
#ifndef SLAVE_BOOT_LOG_ENABLED
#define SLAVE_BOOT_LOG_ENABLED (SLAVE_VOFA_TUNER_ENABLED ? 0 : 1)
#endif

// 功能说明：启用低频状态任务日志。
// 0：不创建状态日志输出；1：按周期输出摘要、XY 和 timing 状态。
#ifndef SLAVE_STATUS_LOG_ENABLED
#define SLAVE_STATUS_LOG_ENABLED (SLAVE_VOFA_TUNER_ENABLED ? 0 : 1)
#endif

// 功能说明：启用控制定时器启动日志。
// 0：不打印定时器启动提示；1：打印控制周期和 dispatch 模式。
#ifndef SLAVE_CONTROL_TIMER_LOG_ENABLED
#define SLAVE_CONTROL_TIMER_LOG_ENABLED SLAVE_BOOT_LOG_ENABLED
#endif

// 状态任务周期，单位 ms。
// 默认 500ms，避免串口输出干扰 Core 0 通信和安全任务。
#ifndef SLAVE_STATUS_LOOP_PERIOD_MS
#define SLAVE_STATUS_LOOP_PERIOD_MS 500UL
#endif

// timing 诊断等级。
// 0：关闭；1：整步统计；2：完整分段统计。
#ifndef SLAVE_TIMING_DIAG_LEVEL
#define SLAVE_TIMING_DIAG_LEVEL 2
#endif

#define SLAVE_TIMING_STEP_DIAG_ENABLED (SLAVE_TIMING_DIAG_LEVEL >= 1)
#define SLAVE_TIMING_DETAIL_DIAG_ENABLED (SLAVE_TIMING_DIAG_LEVEL >= 2)

// 功能说明：启用从机摘要状态行。
// 0：不打印摘要；1：打印模式、收包、fault、UV 和命令年龄。
#ifndef SLAVE_STATUS_SUMMARY_LOG_ENABLED
#define SLAVE_STATUS_SUMMARY_LOG_ENABLED 1
#endif

// 功能说明：启用从机 XY 状态行。
// 0：不打印 XY 细节；1：打印命令、目标、实际角、误差和限位。
#ifndef SLAVE_STATUS_XY_LOG_ENABLED
#define SLAVE_STATUS_XY_LOG_ENABLED 1
#endif

// 功能说明：启用 timing 状态行。
// 0：不打印 timing；1：打印已采样的 timing 统计。
#ifndef SLAVE_STATUS_TIMING_LOG_ENABLED
#define SLAVE_STATUS_TIMING_LOG_ENABLED SLAVE_TIMING_STEP_DIAG_ENABLED
#endif

// 功能说明：启用 YSensorOnly 低频读数日志。
// 0：不在状态任务读取 Y 编码器；1：YSensorOnly bring-up 时输出 Y 编码器读数。
#ifndef SLAVE_STATUS_Y_SENSOR_BRINGUP_LOG_ENABLED
#define SLAVE_STATUS_Y_SENSOR_BRINGUP_LOG_ENABLED 1
#endif

// 功能说明：启用 DengV3 电流环低频状态行。
// 0：不打印 current 行；1：打印 raw ADC、offset、q/d current 和 q/d voltage。
#ifndef SLAVE_STATUS_CURRENT_LOG_ENABLED
#define SLAVE_STATUS_CURRENT_LOG_ENABLED SLAVE_ENABLE_CURRENT_SENSE
#endif
