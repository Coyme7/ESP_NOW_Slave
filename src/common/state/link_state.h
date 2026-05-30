#pragma once

#include <stdint.h>

#include "common/protocol/protocol_types.h"
#include "common/state/fault_flags.h"

// 链路状态只记录通信健康信息，不包含主机电流环或从机位置环内部细节。
struct CommonLinkState {
    // 主机落笔请求。实际 UV 输出必须看 uv_out，不能把请求当作输出。
    volatile bool pen_req;
    // 从机实际 UV 输出状态。主机侧由 telemetry 回填，从机侧由硬件输出层写入。
    volatile bool uv_out;
    // 最近命令/链路状态是否有效。
    volatile bool command_valid;
    // 当前系统模式。
    volatile uint8_t current_mode;
    // 链路状态：启动、连接、退化、超时或故障。
    volatile uint8_t link_state;
    // 协议层故障位，如超时、校验失败等。
    volatile uint16_t protocol_fault_flags;
    // 最近发送的主机命令序号。
    volatile uint32_t last_command_seq;
    // 最近接收的有效从机遥测序号。
    volatile uint32_t last_telemetry_seq;
    // 最近一次有效接收时间，单位 us。
    volatile uint32_t last_rx_us;
    // ESP-NOW 发送成功计数。
    volatile uint32_t espnow_send_ok_count;
    // ESP-NOW 发送失败计数。
    volatile uint32_t espnow_send_fail_count;
    // ESP-NOW 接收并通过校验计数。
    volatile uint32_t espnow_recv_ok_count;
    // ESP-NOW 接收后被拒收计数。
    // 包含协议错误、stale、duplicate 等所有未接受包。
    volatile uint32_t espnow_recv_reject_count;
    // ESP-NOW 接收到旧序号包计数。
    // stale 只作为链路统计，不锁存 fault，不清 command_valid。
    volatile uint32_t espnow_recv_stale_count;
    // ESP-NOW 接收到重复序号包计数。
    // duplicate 只作为链路统计，不锁存 fault，不清 command_valid。
    volatile uint32_t espnow_recv_duplicate_count;
    // 最近一次发送结果，1 表示成功，0 表示失败。
    volatile uint8_t last_send_ok;
};

// 创建默认链路状态，启动时各计数和序号都从 0 开始。
inline CommonLinkState makeDefaultCommonLinkState() {
    CommonLinkState state = {};
    state.current_mode = MODE_COLLAB_DRAW;
    state.link_state = LINK_BOOT;
    state.protocol_fault_flags = FAULT_NONE;
    return state;
}
