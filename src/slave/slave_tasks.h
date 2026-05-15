#pragma once

#include <stdint.h>

// slave_tasks
// 职责：集中从机 FreeRTOS task loop 和任务创建。
// Core 1 只跑控制热路径；Core 0 处理通信、紫光安全和状态输出。

// 创建并启动从机所有任务。调用前必须已经完成安全输出、硬件初始化和 ESP-NOW 初始化。
void startSlaveTasks();

// 控制定时诊断。只在低频状态任务读取，用于确认 ISR timer 是否生效以及是否丢 tick。
uint32_t getSlaveControlTimerMissedTicks();
uint32_t getSlaveControlLastDtUs();
