#pragma once

#include <stddef.h>
#include <stdint.h>

#include "common/protocol/common_contract_version.h"
#include "common/state/fault_flags.h"

static_assert(COMMON_CONTRACT_VERSION == 1, "common contract version mismatch");

// 协议版本用于拒绝不兼容包。只要包字段含义或布局发生破坏性变化，就应提升版本。
// 协议版本必须主从一致；包结构或字段含义改变时应同步升级。
static constexpr uint8_t PROTOCOL_VERSION = 1;

// ESP-NOW 包类型。新增类型时必须同步主机、从机和协议文档。
// 包类型用于区分主机命令和从机遥测，避免误把不同结构体按同一种方式解析。
enum PacketType : uint8_t {
    PACKET_TYPE_MASTER_COMMAND = 1,
    PACKET_TYPE_SLAVE_TELEMETRY = 2,
    PACKET_TYPE_FAULT = 4,
};

// 系统模式只描述行为语义，不承载电机控制参数。
// 系统模式用于标记当前链路语义；下一阶段单轴联调主要使用绘图手动模式。
enum SystemMode : uint8_t {
    MODE_COLLAB_DRAW = 0,
    MODE_AUTO_DRAW = 1,
    MODE_BLE_MEDIA = 2,
};

// 命令包标志位。pen_down 同时保留为独立字段，方便串口观察和兼容旧调试脚本。
// flags 是位标志，可同时表达落笔、边界命中等状态。
enum PacketFlags : uint16_t {
    PACKET_FLAG_NONE = 0,
    PACKET_FLAG_PEN_DOWN = 1u << 0,
};

// 固定长度 ESP-NOW 二进制包。两端结构体布局必须完全一致。
// 主机发给从机的命令包。packed 用于固定二进制布局，避免编译器插入 padding。
struct __attribute__((packed)) MasterCommandPacket {
    // 协议版本，用于主从兼容性检查。
    uint8_t version;
    // 包类型，接收端据此判断应按哪种结构体解析。
    uint8_t type;
    // 命令标志位，当前主要用于 pen down 等开关量。
    uint16_t flags;
    // 发送序号，主机命令和从机遥测各自递增。
    uint32_t seq;
    // 发送端时间戳，便于估算包年龄和调试通信延迟。
    uint32_t timestamp_us;
    // X 轴目标归一化值，当前单轴联调优先使用该字段。
    int16_t x_norm;
    // Y 轴目标归一化值，当前可保留为 0，为后续 XY 扩展预留。
    int16_t y_norm;
    // 落笔状态，独立字段便于从机直接判断执行状态。
    uint8_t pen_down;
    // 系统模式，标识协同绘图、自动绘图或 BLE 控制语义。
    uint8_t mode;
    // 16-bit 校验和，发送前计算，接收后验证。
    uint16_t checksum;
};

// 从机发回主机的遥测包。包含 ack_seq，主机可判断从机处理到了哪个命令。
struct __attribute__((packed)) SlaveTelemetryPacket {
    uint8_t version;
    uint8_t type;
    // 从机故障位，主机收到后合并到状态输出。
    uint16_t fault_flags;
    uint32_t seq;
    // 从机已处理的主机命令序号，用于判断命令是否被执行到。
    uint32_t ack_seq;
    uint32_t timestamp_us;
    // 从机实际 X 轴位置归一化值。
    int16_t x_actual_norm;
    // 从机实际 Y 轴位置归一化值，单轴阶段可固定为 0。
    int16_t y_actual_norm;
    // 从机实际落笔/UV 状态。
    uint8_t pen_state;
    uint8_t mode;
    uint16_t checksum;
};

static_assert(sizeof(MasterCommandPacket) == 20, "MasterCommandPacket layout drifted");
static_assert(sizeof(SlaveTelemetryPacket) == 24, "SlaveTelemetryPacket layout drifted");
static_assert(sizeof(MasterCommandPacket) <= 250, "ESP-NOW v1 payload limit exceeded");
static_assert(sizeof(SlaveTelemetryPacket) <= 250, "ESP-NOW v1 payload limit exceeded");

