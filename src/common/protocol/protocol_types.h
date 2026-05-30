#pragma once

#include <stddef.h>
#include <stdint.h>

#include "common/protocol/common_contract_version.h"
#include "common/state/fault_flags.h"


// 协议版本用于拒绝不兼容包。只要包字段含义或布局发生破坏性变化，就应提升版本。
// 协议版本必须主从一致；包结构或字段含义改变时应同步升级。
static constexpr uint16_t PROTOCOL_MAGIC = 0x5859;  // "XY"
static constexpr uint8_t PROTOCOL_VERSION = 5;

// ESP-NOW 包类型。新增类型时必须同步主机、从机和协议文档。
// 包类型用于区分主机命令和从机遥测，避免误把不同结构体按同一种方式解析。
enum PacketType : uint8_t {
    PACKET_TYPE_MASTER_COMMAND = 1,
    PACKET_TYPE_SLAVE_TELEMETRY = 2,
    PACKET_TYPE_TRAJECTORY_SEGMENT = 3,
    PACKET_TYPE_FAULT = 4,
};

// 系统模式只描述行为语义，不承载电机控制参数。
// 系统模式用于标记当前链路语义；下一阶段单轴联调主要使用绘图手动模式。
enum SystemMode : uint8_t {
    MODE_COLLAB_DRAW = 0,
    // 旧自动绘图占位值；当前跨节点自动绘图使用 MODE_DUALXY_DRAW_* 表达 dry-run/UV 语义。
    MODE_AUTO_DRAW = 1,
    // 主机本地 BLE 媒体模式；从机收到时只进入 safe/idle，不接受远程绘图目标。
    MODE_BLE_MEDIA = 2,
    MODE_DUALXY_DRY_RUN = 3,
    MODE_DUALXY_DRAW_DRY_RUN = 4,
    MODE_DUALXY_DRAW_UV = 5,
};

// 命令包标志位。pen_req 同时保留为独立字段，方便串口观察和兼容旧调试脚本。
// flags 是位标志，可同时表达落笔请求、边界命中等状态。
enum PacketFlags : uint16_t {
    PACKET_FLAG_NONE = 0,
    PACKET_FLAG_PEN_REQ = 1u << 0,
    PACKET_FLAG_DRY_RUN = 1u << 1,
    PACKET_FLAG_TRAJECTORY_ACTIVE = 1u << 2,
};

enum LinkState : uint8_t {
    LINK_BOOT = 0,
    LINK_PAIRING = 1,
    LINK_CONNECTED = 2,
    LINK_DEGRADED = 3,
    LINK_TIMEOUT = 4,
    LINK_FAULT = 5,
};

enum PenState : uint8_t {
    PEN_UP = 0,
    PEN_ARMING = 1,
    PEN_DOWN = 2,
    PEN_FAULT = 3,
};

enum DrawState : uint8_t {
    DRAW_STATE_IDLE = 0,
    DRAW_STATE_RUNNING = 1,
    DRAW_STATE_FINISHED = 2,
    DRAW_STATE_BLOCKED = 3,
    DRAW_STATE_LOADING = 4,
};

enum TrajectoryStatusFlags : uint8_t {
    TRAJECTORY_STATUS_NONE = 0,
    TRAJECTORY_STATUS_TASK_KNOWN = 1u << 0,
    TRAJECTORY_STATUS_LOADING = 1u << 1,
    TRAJECTORY_STATUS_READY = 1u << 2,
    TRAJECTORY_STATUS_RUNNING = 1u << 3,
    TRAJECTORY_STATUS_COMPLETE = 1u << 4,
    TRAJECTORY_STATUS_SEGMENT_MISSING = 1u << 5,
    TRAJECTORY_STATUS_BLOCKED = 1u << 6,
};

enum UvBlockReason : uint16_t {
    UV_BLOCK_NONE = 0,
    UV_BLOCK_MODE = 1u << 0,
    UV_BLOCK_DRY_RUN = 1u << 1,
    UV_BLOCK_PEN_UP = 1u << 2,
    UV_BLOCK_FAULT = 1u << 3,
    UV_BLOCK_TIMEOUT = 1u << 4,
    UV_BLOCK_BOUNDARY = 1u << 5,
    UV_BLOCK_TRACKING = 1u << 6,
    UV_BLOCK_NOT_SETTLED = 1u << 7,
    UV_BLOCK_X_INVALID = 1u << 8,
    UV_BLOCK_Y_INVALID = 1u << 9,
    UV_BLOCK_INTERLOCK = 1u << 10,
};

// 固定长度 ESP-NOW 二进制包。两端结构体布局必须完全一致。
// 主机发给从机的命令包。packed 用于固定二进制布局，避免编译器插入 padding。
struct __attribute__((packed)) MasterCommandPacket {
    uint16_t magic;
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
    // 落笔请求，独立字段便于从机直接判断请求状态。实际 UV 输出由遥测 uv_out 表示。
    uint8_t pen_req;
    // 系统模式，标识协同绘图、自动绘图或 BLE 控制语义。
    uint8_t mode;
    // 命令扩展标志，当前用于 dry-run / trajectory 等模式提示。
    uint16_t command_flags;
    // CRC16-CCITT，发送前计算，接收后验证。
    uint16_t crc16;
};

// 主机分包下发给从机的自动绘图轨迹段。
// 坐标和速度使用 0.1mm / 0.1mm/s 定点值，避免跨节点发送 float。
struct __attribute__((packed)) TrajectorySegmentPacket {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t flags;
    uint32_t seq;
    uint32_t timestamp_us;
    uint16_t task_id;
    uint8_t segment_index;
    uint8_t segment_count;
    int16_t start_x_mm_q10;
    int16_t start_y_mm_q10;
    int16_t end_x_mm_q10;
    int16_t end_y_mm_q10;
    uint16_t feed_mm_s_q10;
    uint8_t pen_req;
    uint8_t reserved;
    uint16_t crc16;
};

// 从机发回主机的遥测包。包含 ack_seq，主机可判断从机处理到了哪个命令。
struct __attribute__((packed)) SlaveTelemetryPacket {
    uint16_t magic;
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
    uint16_t uv_block_reasons;
    uint8_t draw_state;
    uint8_t draw_progress_pct;
    uint8_t link_state;
    uint8_t uv_out;
    uint16_t trajectory_task_id;
    uint8_t trajectory_segment_count;
    uint8_t trajectory_segment_cursor;
    uint8_t trajectory_received_count;
    uint8_t trajectory_status_flags;
    uint32_t trajectory_received_mask_low;
    uint16_t trajectory_received_mask_high;
    uint16_t crc16;
};
