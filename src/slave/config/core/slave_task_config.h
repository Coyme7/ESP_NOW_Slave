#pragma once

#include <Arduino.h>

#include "slave/config/core/slave_control_config.h"

// 从机 FreeRTOS 任务与定时器配置。
// Core 1 保留控制热路径和 ADC DMA consumer；Core 0 运行通信、安全和状态输出。
static constexpr BaseType_t SLAVE_CONTROL_CORE = 1;
static constexpr BaseType_t SLAVE_IO_CORE = 0;

static constexpr UBaseType_t SLAVE_CONTROL_TASK_PRIORITY = configMAX_PRIORITIES - 1;
static constexpr UBaseType_t SLAVE_ADC_DMA_TASK_PRIORITY = configMAX_PRIORITIES - 2;
static constexpr UBaseType_t SLAVE_COMM_TASK_PRIORITY = 3;
static constexpr UBaseType_t SLAVE_SAFETY_TASK_PRIORITY = 2;
static constexpr UBaseType_t SLAVE_STATUS_TASK_PRIORITY = 1;
static constexpr UBaseType_t SLAVE_VOFA_TUNER_TASK_PRIORITY = 1;

static constexpr uint32_t SLAVE_CONTROL_TASK_STACK_BYTES = 8192;
static constexpr uint32_t SLAVE_ADC_DMA_TASK_STACK_BYTES = 4096;
static constexpr uint32_t SLAVE_COMM_TASK_STACK_BYTES = 4096;
static constexpr uint32_t SLAVE_SAFETY_TASK_STACK_BYTES = 4096;
static constexpr uint32_t SLAVE_STATUS_TASK_STACK_BYTES = 4096;
static constexpr uint32_t SLAVE_VOFA_TUNER_TASK_STACK_BYTES = 4096;

static constexpr uint32_t SLAVE_CONTROL_STEPS_PER_TICK =
    1000UL / CONTROL_LOOP_PERIOD_US;
static constexpr uint32_t SLAVE_CONTROL_TIMER_PERIOD_US =
    CONTROL_LOOP_PERIOD_US;
static constexpr uint32_t SLAVE_CONTROL_TIMER_TIMEOUT_MS = 10UL;
