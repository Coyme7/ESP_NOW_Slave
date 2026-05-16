#include "common/state/system_state.h"

#include <Arduino.h>

// 全局监视状态放在内部 RAM，避免跨任务频繁读写落到外部存储。
// common 只承载链路/故障状态，主机和从机调试字段分别放在各自子结构中。
DRAM_ATTR // 全局状态实例，集中保存主机、从机、链路和故障信息。
SharedData sysData = {
    makeDefaultCommonLinkState(),
    {},
    {},
};

namespace {

// 本机故障锁存位。后续如果加入清故障命令，应在这里集中处理锁存策略。
volatile uint16_t local_fault_flags = FAULT_NONE;
// 当前周期正在生效的故障位，状态行用它区分实时故障和历史锁存故障。
volatile uint16_t active_fault_flags = FAULT_NONE;

}  // namespace

// 锁存本地故障：一旦置位，不会自动清零，便于保留故障现场。
void addLocalFault(uint16_t fault) {
    // 故障采用 OR 锁存，FAULT_NONE 不会清除已经发生过的故障。
    local_fault_flags = static_cast<uint16_t>(local_fault_flags | fault);
    sysData.link.protocol_fault_flags =
        combineWithLocalFaults(active_fault_flags);
}

// 读取本地故障锁存值。
uint16_t getLocalFaultFlags() {
    return local_fault_flags;
}

uint16_t getActiveFaultFlags() {
    return active_fault_flags;
}

uint16_t getLatchedFaultFlags() {
    return local_fault_flags;
}

// 把远端或运行时故障与本地锁存故障合并。
uint16_t combineWithLocalFaults(uint16_t remote_or_runtime_faults) {
    return static_cast<uint16_t>(local_fault_flags | remote_or_runtime_faults);
}

// 发布协议/遥测相关故障到 sysData，状态输出会读取该字段。
void publishProtocolFaults(uint16_t remote_or_runtime_faults) {
    active_fault_flags = remote_or_runtime_faults;
    sysData.link.protocol_fault_flags = combineWithLocalFaults(remote_or_runtime_faults);
}
