#pragma once

#include <stdint.h>

// 故障位会跨主从回传，顺序和含义改变时要同步协议文档。
// 故障位按 bit 分配，可以通过按位或组合多个异常来源。
enum FaultFlags : uint16_t {
    FAULT_NONE = 0,
    FAULT_COMMAND_TIMEOUT = 1u << 0,
    FAULT_CHECKSUM_ERROR = 1u << 1,
    FAULT_VERSION_MISMATCH = 1u << 2,
    FAULT_PACKET_SIZE = 1u << 3,
    FAULT_STALE_SEQUENCE = 1u << 4,
    FAULT_BOUNDARY_HIT = 1u << 5,
    FAULT_MOTOR_OUTPUT_DISABLED = 1u << 6,
    FAULT_TARGET_LIMITED = 1u << 7,
    FAULT_UV_INTERLOCK = 1u << 8,
    FAULT_TELEMETRY_TIMEOUT = 1u << 9,
    FAULT_ENCODER_INVALID = 1u << 10,
    FAULT_LINK_STATE = 1u << 11,
    FAULT_PEN_STATE = 1u << 12,
    FAULT_TRAJECTORY_INVALID = 1u << 13,
};
