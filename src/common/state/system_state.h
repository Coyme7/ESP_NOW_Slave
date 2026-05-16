#pragma once

#include <stdint.h>

#include "common/state/shared_data.h"

// 锁存本机故障。故障位会保留，直到复位或后续显式清故障机制接入。
void addLocalFault(uint16_t fault);

// 读取本机已经锁存的故障位。
uint16_t getLocalFaultFlags();

// 读取当前周期正在生效的故障位，不包含历史锁存位。
uint16_t getActiveFaultFlags();

// 读取历史锁存故障位，语义等同于 getLocalFaultFlags()。
uint16_t getLatchedFaultFlags();

// 将对端回传或当前步骤产生的故障与本机锁存故障合并。
uint16_t combineWithLocalFaults(uint16_t remote_or_runtime_faults);

// 把合并后的故障发布到 sysData.link.protocol_fault_flags。
void publishProtocolFaults(uint16_t remote_or_runtime_faults);
