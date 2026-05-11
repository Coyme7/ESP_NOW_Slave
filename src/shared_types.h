#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// shared_types.h 是主从两套独立工程之间的协议契约。
// 两边文件内容必须保持一致；任何字段、枚举值、常量语义变化，都要同步修改主机、
// 从机和 docs/packet_protocol_draft.md，并分别编译两个工程。

// 协议版本用于拒绝不兼容包。只要包字段含义或布局发生破坏性变化，就应提升版本。
static constexpr uint8_t PROTOCOL_VERSION = 1;

// 运行节拍约定：sdkconfig.defaults 将 FreeRTOS tick 固定为 1000 Hz；
// 控制任务在每个 1 ms tick 内执行 10 个 100 us 子步，目标控制频率约 10 kHz。
// 命令/遥测超时 50 ms：足够覆盖少量无线抖动，但能快速让 UV 和控制状态进入安全模式。
static constexpr uint32_t COMMAND_TIMEOUT_US = 50000UL;
static constexpr uint32_t TELEMETRY_TIMEOUT_US = 50000UL;
static constexpr uint32_t CONTROL_LOOP_PERIOD_US = 100UL;
static constexpr uint32_t COMM_LOOP_PERIOD_MS = 10UL;
static constexpr uint32_t STATUS_LOOP_PERIOD_MS = 100UL;
static constexpr int16_t NORM_MIN = static_cast<int16_t>(-32767 - 1);
static constexpr int16_t NORM_MAX = 32767;
static constexpr float PI_F = 3.14159265358979323846f;

// 物理测试几何：协议包只传归一化坐标，不传电机型号、编码器原始值或电角度。
// 当前临时联调用 MT6701 编码器 + 2804 电机测试主从两端；
// 后续从机正式云台仍可切回 2208，硬件差异留在各节点本地配置中。
// A4 和安全边距用于从机把 x_norm/y_norm 解释成纸面坐标。
static constexpr float MASTER_KNOB_HALF_RANGE_DEG = 180.0f;
static constexpr float A4_WIDTH_MM = 210.0f;
static constexpr float A4_HEIGHT_MM = 297.0f;
static constexpr float PAPER_EDGE_MARGIN_MM = 10.0f;
static constexpr float PLOT_X_HALF_RANGE_MM = (A4_WIDTH_MM * 0.5f) - PAPER_EDGE_MARGIN_MM;
static constexpr float PLOT_Y_HALF_RANGE_MM = (A4_HEIGHT_MM * 0.5f) - PAPER_EDGE_MARGIN_MM;
static constexpr float DEFAULT_THROW_DISTANCE_MM = 700.0f;

// ESP-NOW 包类型。新增类型时必须同步主机、从机和协议文档。
enum PacketType : uint8_t {
    PACKET_TYPE_MASTER_COMMAND = 1,
    PACKET_TYPE_SLAVE_TELEMETRY = 2,
    PACKET_TYPE_FAULT = 4,
};

// 系统模式只描述行为语义，不承载电机控制参数。
// 模式值会通过协议传输，因此新增模式时要保证旧固件的默认行为仍安全。
enum SystemMode : uint8_t {
    MODE_COLLAB_DRAW = 0,
    MODE_AUTO_DRAW = 1,
    MODE_BLE_MEDIA = 2,
};

// 命令包标志位。pen_down 同时保留为独立字段，方便串口观察和兼容旧调试脚本。
// flags 适合以后扩展更多布尔命令；独立 pen_down 字段方便人读和调试脚本直接解析。
enum PacketFlags : uint16_t {
    PACKET_FLAG_NONE = 0,
    PACKET_FLAG_PEN_DOWN = 1u << 0,
};

// 故障位会跨主从回传，顺序和含义改变时要同步 docs/packet_protocol_draft.md。
// 故障位只表达“发生了什么”，不在协议层规定如何清除；锁存策略由本机 system_state 管理。
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
};

// 固定长度 ESP-NOW 二进制包。
// 两个独立工程中的结构体布局必须完全一致；改字段时同步主机、从机和协议文档。
// __attribute__((packed)) 防止编译器插入对齐填充；static_assert 用于在编译期发现漂移。
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

// 简单反码累加校验。checksum 字段本身不参与计算。
// 这个校验只用于发现短链路调试中的常见损坏，不替代加密或强完整性校验。
inline uint16_t packetChecksum(const void *data, size_t len) {
    const uint8_t *bytes = static_cast<const uint8_t *>(data);
    uint16_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum = static_cast<uint16_t>(sum + bytes[i]);
    }
    return static_cast<uint16_t>(~sum);
}

inline void finalizeMasterCommand(MasterCommandPacket &packet) {
    // 打包前统一写入 version/type/checksum，避免调用方忘记维护协议头。
    packet.version = PROTOCOL_VERSION;
    packet.type = PACKET_TYPE_MASTER_COMMAND;
    packet.checksum = packetChecksum(&packet, offsetof(MasterCommandPacket, checksum));
}

inline void finalizeSlaveTelemetry(SlaveTelemetryPacket &packet) {
    // 遥测包同样由统一出口补齐协议头和校验。
    packet.version = PROTOCOL_VERSION;
    packet.type = PACKET_TYPE_SLAVE_TELEMETRY;
    packet.checksum = packetChecksum(&packet, offsetof(SlaveTelemetryPacket, checksum));
}

inline bool validateMasterCommand(const MasterCommandPacket &packet) {
    // 从机接收主机命令时使用：版本、类型和 checksum 全部通过才允许进入命令快照。
    return packet.version == PROTOCOL_VERSION &&
           packet.type == PACKET_TYPE_MASTER_COMMAND &&
           packet.checksum == packetChecksum(&packet, offsetof(MasterCommandPacket, checksum));
}

inline bool validateSlaveTelemetry(const SlaveTelemetryPacket &packet) {
    // 主机接收从机遥测时使用：拒绝错版本、错类型或校验失败的包。
    return packet.version == PROTOCOL_VERSION &&
           packet.type == PACKET_TYPE_SLAVE_TELEMETRY &&
           packet.checksum == packetChecksum(&packet, offsetof(SlaveTelemetryPacket, checksum));
}

inline bool isNewerSeq(uint32_t seq, uint32_t previous) {
    // 使用有符号差值处理 uint32_t 回绕，常见于递增序号的新旧判断。
    return static_cast<int32_t>(seq - previous) > 0;
}

inline float clampFloat(float value, float min_value, float max_value) {
    // 控制和坐标换算共用的夹紧函数，避免异常输入传播到电机目标或协议范围外。
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

inline float degToRad(float deg) {
    return deg * PI_F / 180.0f;
}

inline float radToDeg(float rad) {
    return rad * 180.0f / PI_F;
}

inline float wrapAngleDegToSigned180(float angle_deg) {
    // MT6701 原始机械角是 0..360 deg；控制/协议侧统一使用 [-180, +180] deg。
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

inline float absoluteAngleDegToSignedAngleDeg(float absolute_angle_deg, float center_deg) {
    // 先减机械中心得到相对角，再 wrap，避免把 0..360 deg 绝对角直接夹到端点。
    return wrapAngleDegToSigned180(absolute_angle_deg - center_deg);
}

inline float signedAngleDeltaDeg(float current_deg, float previous_deg) {
    // 角速度估算要跨过 +/-180 deg 回绕点，不能直接相减得到接近 360 deg 的假跳变。
    return wrapAngleDegToSigned180(current_deg - previous_deg);
}

// 单位坐标和 int16 归一化坐标互转。
// int16_t 负数比正数多一个取值，所以 NORM_MIN 需要单独处理。
inline int16_t unitToNorm(float unit) {
    unit = clampFloat(unit, -1.0f, 1.0f);
    if (unit <= -1.0f) {
        return NORM_MIN;
    }
    return static_cast<int16_t>(unit * static_cast<float>(NORM_MAX));
}

inline float normToUnit(int16_t norm) {
    if (norm == NORM_MIN) {
        return -1.0f;
    }
    return clampFloat(static_cast<float>(norm) / static_cast<float>(NORM_MAX), -1.0f, 1.0f);
}

inline int16_t signedAngleDegToNorm(float angle_deg, float half_range_deg) {
    // 将任意角度先限制到机械半幅，再映射到协议归一化坐标。
    const float limited = clampFloat(angle_deg, -half_range_deg, half_range_deg);
    return unitToNorm(limited / half_range_deg);
}

inline float normToSignedAngleDeg(int16_t norm, float half_range_deg) {
    // 将协议归一化坐标按指定半幅还原成带符号角度。
    return normToUnit(norm) * half_range_deg;
}

// 当前联调把主机旋钮 +/-180 deg 机械行程映射到完整归一化 X 范围。
// 从机纸面几何和云台角度换算仍是本地配置，不进入协议包。
inline int16_t angleDegToNorm(float angle_deg) {
    return signedAngleDegToNorm(angle_deg, MASTER_KNOB_HALF_RANGE_DEG);
}

inline float normToAngleDeg(int16_t norm) {
    return normToSignedAngleDeg(norm, MASTER_KNOB_HALF_RANGE_DEG);
}

inline int16_t percentToNorm(float percent) {
    // 百分比主要用于串口/调试接口，先限制到 +/-100%，再映射到协议坐标。
    if (percent < -100.0f) {
        percent = -100.0f;
    } else if (percent > 100.0f) {
        percent = 100.0f;
    }
    return static_cast<int16_t>((percent / 100.0f) * static_cast<float>(NORM_MAX));
}

inline float normToPercent(int16_t norm) {
    // 将协议坐标转换成易读百分比，供串口状态行使用。
    return (static_cast<float>(norm) / static_cast<float>(NORM_MAX)) * 100.0f;
}

// 串口监视共享状态。
// 这里故意保持为简单 volatile 标量，方便状态任务读取；固定长度包仍用短临界区快照，
// 避免 Core 1 控制路径读到半包。
struct SharedData {
    // 主机侧监视：旋钮角度、力反馈目标电流和主机映射出的纸面百分比。
    volatile float master_angle_deg;
    volatile float master_target_current_a;

    // 从机侧监视：实际角度、主从各自认为的 X/Y 百分比和目标/实际云台角。
    volatile float slave_angle_deg;
    volatile float master_x_pos;
    volatile float master_y_pos;
    volatile float slave_x_pos;
    volatile float slave_y_pos;
    volatile float slave_target_angle_rad;
    volatile float slave_actual_angle_rad;

    // 通用状态：pen_down 表示当前输出/命令落笔状态，boundary_hit 表示触边或接近边界。
    volatile bool pen_down;
    volatile bool boundary_hit;

    // command_valid 和 last_rx_us 用于超时判断；current_mode 是当前行为模式。
    volatile bool command_valid;
    volatile uint8_t current_mode;
    volatile uint16_t protocol_fault_flags;
    volatile uint32_t last_command_seq;
    volatile uint32_t last_telemetry_seq;
    volatile uint32_t last_rx_us;

    // ESP-NOW 健康计数只用于串口诊断，不作为控制输入；
    // 真正的闭环安全仍依赖序号、超时和故障位。
    volatile uint32_t espnow_send_ok_count;
    volatile uint32_t espnow_send_fail_count;
    volatile uint32_t espnow_recv_ok_count;
    volatile uint32_t espnow_recv_reject_count;
    volatile uint8_t last_send_ok;

    // 从机专用联锁状态：主机要求落笔但安全条件不满足时为 true。
    volatile bool uv_interlock_blocked;
};

extern SharedData sysData;
