#pragma once

#include "shared_types.h"

// common/system_state
// 职责：集中定义跨任务可见的监视状态和本机故障锁存位。
//
// 这里的设计刻意很朴素：状态字段放在 shared_types.h 的 SharedData 中，
// 本文件只提供故障合并和发布的小函数。这样 ESP-NOW 回调、Core 0 任务和
// Core 1 控制步都能用同一套状态出口，不需要互相依赖具体模块。
//
// 运行约束：这些函数只做标量读写，可被 ESP-NOW 回调、低频任务和控制步调用；
// 不做日志、不分配内存、不访问外设，避免污染 10 kHz 热路径。

// 锁存本机故障。调用后故障位会保留，直到复位或后续新增明确的清故障机制。
void addLocalFault(uint16_t fault);

// 读取本机已经锁存的故障位。控制和通信路径会用它合并运行期故障。
uint16_t getLocalFaultFlags();

// 将对端回传或当前步骤产生的故障，与本机锁存故障合并。
uint16_t combineWithLocalFaults(uint16_t remote_or_runtime_faults);

// 把合并后的故障发布到 sysData.protocol_fault_flags，供串口和遥测读取。
void publishProtocolFaults(uint16_t remote_or_runtime_faults);
