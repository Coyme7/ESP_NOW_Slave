#pragma once

#include <stddef.h>
#include <stdint.h>

#include "common/protocol/common_contract_version.h"
#include "common/state/fault_flags.h"

static_assert(COMMON_CONTRACT_VERSION == 1, "common contract version mismatch");

// 协议版本用于拒绝不兼容包。只要包字段含义或布局发生破坏性变化，就应提升版本。
static constexpr uint8_t PROTOCOL_VERSION = 1;

// ESP-NOW 包类型。新增类型时必须同步主机、从机和协议文档。
enum PacketType : uint8_t {
    PACKET_TYPE_MASTER_COMMAND = 1,
    PACKET_TYPE_SLAVE_TELEMETRY = 2,
    PACKET_TYPE_FAULT = 4,
};

// 系统模式只描述行为语义，不承载电机控制参数。
enum SystemMode : uint8_t {
    MODE_COLLAB_DRAW = 0,
    MODE_AUTO_DRAW = 1,
    MODE_BLE_MEDIA = 2,
};

// 命令包标志位。pen_down 同时保留为独立字段，方便串口观察和兼容旧调试脚本。
enum PacketFlags : uint16_t {
    PACKET_FLAG_NONE = 0,
    PACKET_FLAG_PEN_DOWN = 1u << 0,
};

// 固定长度 ESP-NOW 二进制包。两端结构体布局必须完全一致。
struct __attribute__((packed)) MasterCommandPacket {
    uint8_t version;
    uint8_t type;
    uint16_t flags;
    uint32_t seq;
    uint32_t timestamp_us;
    int16_t x_norm;
    int16_t y_norm;
    uint8_t pen_down;
    uint8_t mode;
    uint16_t checksum;
};

struct __attribute__((packed)) SlaveTelemetryPacket {
    uint8_t version;
    uint8_t type;
    uint16_t fault_flags;
    uint32_t seq;
    uint32_t ack_seq;
    uint32_t timestamp_us;
    int16_t x_actual_norm;
    int16_t y_actual_norm;
    uint8_t pen_state;
    uint8_t mode;
    uint16_t checksum;
};

static_assert(sizeof(MasterCommandPacket) == 20, "MasterCommandPacket layout drifted");
static_assert(sizeof(SlaveTelemetryPacket) == 24, "SlaveTelemetryPacket layout drifted");
static_assert(sizeof(MasterCommandPacket) <= 250, "ESP-NOW v1 payload limit exceeded");
static_assert(sizeof(SlaveTelemetryPacket) <= 250, "ESP-NOW v1 payload limit exceeded");

