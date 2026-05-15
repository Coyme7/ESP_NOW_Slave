#pragma once

#include <stdint.h>

// 主从一致的通信和状态周期。控制周期属于各节点本地 task config。
static constexpr uint32_t COMMAND_TIMEOUT_US = 50000UL;
static constexpr uint32_t TELEMETRY_TIMEOUT_US = 50000UL;
static constexpr uint32_t COMM_LOOP_PERIOD_MS = 10UL;
static constexpr uint32_t STATUS_LOOP_PERIOD_MS = 100UL;

