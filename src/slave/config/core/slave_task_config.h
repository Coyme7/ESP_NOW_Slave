#pragma once

#include <Arduino.h>

#include "slave/config/core/slave_control_config.h"

// 从机 FreeRTOS 任务与定时器配置。
// Core 1 保留控制热路径，Core 0 运行通信、安全和状态输出。

static constexpr BaseType_t SLAVE_CONTROL_CORE = 1; // 控制任务绑核。
static constexpr BaseType_t SLAVE_IO_CORE = 0;      // IO/通信/状态任务绑核。

static constexpr UBaseType_t SLAVE_CONTROL_TASK_PRIORITY = configMAX_PRIORITIES - 1; // 控制任务优先级。
static constexpr UBaseType_t SLAVE_COMM_TASK_PRIORITY = 3;                           // 通信任务优先级。
static constexpr UBaseType_t SLAVE_SAFETY_TASK_PRIORITY = 2;                         // 安全任务优先级。
static constexpr UBaseType_t SLAVE_STATUS_TASK_PRIORITY = 1;                         // 状态任务优先级。
static constexpr UBaseType_t SLAVE_VOFA_TUNER_TASK_PRIORITY = 1;                     // VOFA 串口任务优先级。

static constexpr uint32_t SLAVE_CONTROL_TASK_STACK_BYTES = 8192; // 控制任务栈，单位 byte。
static constexpr uint32_t SLAVE_COMM_TASK_STACK_BYTES = 4096;    // 通信任务栈，单位 byte。
static constexpr uint32_t SLAVE_SAFETY_TASK_STACK_BYTES = 4096;  // 安全任务栈，单位 byte。
static constexpr uint32_t SLAVE_STATUS_TASK_STACK_BYTES = 4096;  // 状态任务栈，单位 byte。
static constexpr uint32_t SLAVE_VOFA_TUNER_TASK_STACK_BYTES = 4096; // VOFA 串口任务栈，单位 byte。

static constexpr uint32_t SLAVE_CONTROL_STEPS_PER_TICK = 1000UL / CONTROL_LOOP_PERIOD_US; // 每个 FreeRTOS tick 的控制步数。
static constexpr uint32_t SLAVE_CONTROL_TIMER_PERIOD_US = CONTROL_LOOP_PERIOD_US;          // 控制定时器周期，单位 us。
static constexpr uint32_t SLAVE_CONTROL_TIMER_TIMEOUT_MS = 10UL;                           // 控制定时器等待超时，单位 ms。
