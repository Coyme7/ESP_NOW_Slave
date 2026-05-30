#pragma once

#include <stddef.h>
#include <stdint.h>

#include "common/protocol/protocol_types.h"

// CRC16-CCITT，crc16 字段本身不参与计算。
inline uint16_t packetCrc16(const void *data, size_t len) {
    const uint8_t *bytes = static_cast<const uint8_t *>(data);
    uint16_t crc = 0xffffU;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(bytes[i]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                                  : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

// 主机命令包收尾：写入包头、协议版本和 CRC。
inline void finalizeMasterCommand(MasterCommandPacket &packet) {
    packet.magic = PROTOCOL_MAGIC;
    packet.version = PROTOCOL_VERSION;
    packet.type = PACKET_TYPE_MASTER_COMMAND;
    packet.crc16 = packetCrc16(&packet, offsetof(MasterCommandPacket, crc16));
}

// 从机遥测包收尾：写入包头、协议版本和 CRC。
inline void finalizeSlaveTelemetry(SlaveTelemetryPacket &packet) {
    packet.magic = PROTOCOL_MAGIC;
    packet.version = PROTOCOL_VERSION;
    packet.type = PACKET_TYPE_SLAVE_TELEMETRY;
    packet.crc16 = packetCrc16(&packet, offsetof(SlaveTelemetryPacket, crc16));
}

// 主机轨迹段分包收尾：写入包头、协议版本和 CRC。
inline void finalizeTrajectorySegment(TrajectorySegmentPacket &packet) {
    packet.magic = PROTOCOL_MAGIC;
    packet.version = PROTOCOL_VERSION;
    packet.type = PACKET_TYPE_TRAJECTORY_SEGMENT;
    packet.crc16 = packetCrc16(&packet, offsetof(TrajectorySegmentPacket, crc16));
}

// 收到主机命令时验证 magic、类型、版本和 CRC，防止旧包或损坏包进入控制层。
inline bool validateMasterCommand(const MasterCommandPacket &packet) {
    return packet.magic == PROTOCOL_MAGIC &&
           packet.version == PROTOCOL_VERSION &&
           packet.type == PACKET_TYPE_MASTER_COMMAND &&
           packet.crc16 == packetCrc16(&packet, offsetof(MasterCommandPacket, crc16));
}

// 收到从机遥测时验证 magic、类型、版本和 CRC，防止错误状态污染主机显示。
inline bool validateSlaveTelemetry(const SlaveTelemetryPacket &packet) {
    return packet.magic == PROTOCOL_MAGIC &&
           packet.version == PROTOCOL_VERSION &&
           packet.type == PACKET_TYPE_SLAVE_TELEMETRY &&
           packet.crc16 == packetCrc16(&packet, offsetof(SlaveTelemetryPacket, crc16));
}

// 收到轨迹段分包时验证 magic、类型、版本和 CRC。
inline bool validateTrajectorySegment(const TrajectorySegmentPacket &packet) {
    return packet.magic == PROTOCOL_MAGIC &&
           packet.version == PROTOCOL_VERSION &&
           packet.type == PACKET_TYPE_TRAJECTORY_SEGMENT &&
           packet.crc16 == packetCrc16(&packet, offsetof(TrajectorySegmentPacket, crc16));
}

// 序号比较使用有符号差值，允许 uint32_t 溢出后仍能判断新旧。
inline bool isNewerSeq(uint32_t seq, uint32_t previous) {
    return static_cast<int32_t>(seq - previous) > 0;
}
