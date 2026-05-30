#pragma once

#include <stdint.h>

#include "slave/control/slave_runtime_snapshot.h"

// slave_safety
// 职责：从机命令超时和紫光安全联锁。
// 运行约束：isSlaveUvAllowed() 只读快照和标量状态；runSlaveSafetyStep() 是 1 kHz
// 低频安全任务入口，负责唯一的 UV GPIO 写操作。

// 判断实时命令缓存是否新鲜。
// SlaveRtCommand 表示“最后一次通过校验的命令”；fresh 表示“当前是否未超时”。
bool isSlaveRtCommandFresh(const SlaveRtCommand &command, uint32_t now_us);

// 判断主机命令是否已经失效。无命令或超过 COMMAND_TIMEOUT_US 都视为超时。
bool isSlaveCommandTimedOut(uint32_t now_us);

// 判断当前是否允许打开紫光：命令有效、pen_req、未触边且跟踪误差足够小。
bool isSlaveUvAllowed(uint32_t now_us);

// 返回本周期 UV 被禁止的具体原因位，取 UvBlockReason。
uint16_t evaluateSlaveUvBlockReasons(uint32_t now_us);

// 1 kHz 安全任务入口，集中更新 uv_interlock_blocked 并写 UV MOS。
void runSlaveSafetyStep(uint32_t now_us);
