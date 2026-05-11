#include "common/system_state.h"

// 全局监视状态。
// DRAM_ATTR 明确把它放在内部 RAM，降低跨任务频繁读写时落到外部存储的风险。
// 字段顺序必须和 SharedData 定义保持一致；这里只给出上电默认值：
// 坐标/角度归零、pen/边界/超时均为 false、模式为协作绘图、故障为 FAULT_NONE。
DRAM_ATTR SharedData sysData = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    false,
    false,
    false,
    MODE_COLLAB_DRAW,
    FAULT_NONE,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    false,
};

namespace {

// 本机故障锁存位。
// volatile 表示它可能被不同任务或回调读写；这里只进行 16 bit 标量合并，
// 不负责复杂状态机。后续若加入“清故障命令”，应在这里集中处理锁存策略。
volatile uint16_t local_fault_flags = FAULT_NONE;

}  // namespace

void addLocalFault(uint16_t fault) {
    // 故障采用 OR 锁存：任何模块报告过的故障都不会被后续 FAULT_NONE 覆盖。
    // 同步写入 sysData，是为了状态打印和遥测能立刻看到本机故障。
    local_fault_flags = static_cast<uint16_t>(local_fault_flags | fault);
    sysData.protocol_fault_flags = static_cast<uint16_t>(sysData.protocol_fault_flags | fault);
}

uint16_t getLocalFaultFlags() {
    // 只返回本机锁存故障，不混入对端命令或当前控制步临时故障。
    return local_fault_flags;
}

uint16_t combineWithLocalFaults(uint16_t remote_or_runtime_faults) {
    // 对端故障、运行期故障和本机锁存故障在串口/遥测层统一展示。
    return static_cast<uint16_t>(local_fault_flags | remote_or_runtime_faults);
}

void publishProtocolFaults(uint16_t remote_or_runtime_faults) {
    // 这是所有模块更新对外故障视图的统一出口，避免各处重复拼 fault flags。
    sysData.protocol_fault_flags = combineWithLocalFaults(remote_or_runtime_faults);
}
