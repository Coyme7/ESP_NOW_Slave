#pragma once

#include <stdint.h>

// 主从共享的低频链路节拍。
// 控制环周期属于各节点本地配置，不放入 common，避免把 FOC 频率和无线节拍耦合。
static constexpr uint32_t COMMAND_TIMEOUT_US = 50000UL;
static constexpr uint32_t TELEMETRY_TIMEOUT_US = 50000UL;

// 主机命令发送：约 333 Hz，用于 X/Y/pen 最新命令发布。
static constexpr uint32_t MASTER_COMMAND_PERIOD_MS = 3UL;
// 从机遥测回传：50 Hz 起步，先降低无线和串口观测干扰，稳定后可提升到 100 Hz。
static constexpr uint32_t SLAVE_TELEMETRY_PERIOD_MS = 20UL;
