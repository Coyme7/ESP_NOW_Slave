#include "common/state/system_state.h"

#include <Arduino.h>

// 全局监视状态放在内部 RAM，避免跨任务频繁读写落到外部存储。
// common 只承载链路/故障状态，主机和从机调试字段分别放在各自子结构中。
DRAM_ATTR SharedData sysData = {
    makeDefaultCommonLinkState(),
    {},
    {},
};

namespace {

// 本机故障锁存位。后续如果加入清故障命令，应在这里集中处理锁存策略。
volatile uint16_t local_fault_flags = FAULT_NONE;

}  // namespace

void addLocalFault(uint16_t fault) {
    // 故障采用 OR 锁存，FAULT_NONE 不会清除已经发生过的故障。
    local_fault_flags = static_cast<uint16_t>(local_fault_flags | fault);
    sysData.link.protocol_fault_flags =
        static_cast<uint16_t>(sysData.link.protocol_fault_flags | fault);
}

uint16_t getLocalFaultFlags() {
    return local_fault_flags;
}

uint16_t combineWithLocalFaults(uint16_t remote_or_runtime_faults) {
    return static_cast<uint16_t>(local_fault_flags | remote_or_runtime_faults);
}

void publishProtocolFaults(uint16_t remote_or_runtime_faults) {
    sysData.link.protocol_fault_flags = combineWithLocalFaults(remote_or_runtime_faults);
}
