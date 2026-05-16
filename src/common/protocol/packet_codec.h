#pragma once

#include <stddef.h>
#include <stdint.h>

#include "common/protocol/protocol_types.h"

// 简单反码累加校验。checksum 字段本身不参与计算。
// 简单 16-bit 累加校验：发送前把 checksum 字段清零再计算。
inline uint16_t packetChecksum(const void *data, size_t len) {
    const uint8_t *bytes = static_cast<const uint8_t *>(data);
    uint16_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum = static_cast<uint16_t>(sum + bytes[i]);
    }
    return static_cast<uint16_t>(~sum);
}

// 主机命令包收尾：写入包类型、协议版本和 checksum。
inline void finalizeMasterCommand(MasterCommandPacket &packet) {
    packet.version = PROTOCOL_VERSION;
    packet.type = PACKET_TYPE_MASTER_COMMAND;
    packet.checksum = packetChecksum(&packet, offsetof(MasterCommandPacket, checksum));
}

// 从机遥测包收尾：写入包类型、协议版本和 checksum。
inline void finalizeSlaveTelemetry(SlaveTelemetryPacket &packet) {
    packet.version = PROTOCOL_VERSION;
    packet.type = PACKET_TYPE_SLAVE_TELEMETRY;
    packet.checksum = packetChecksum(&packet, offsetof(SlaveTelemetryPacket, checksum));
}

// 收到主机命令时验证类型、版本和校验和，防止旧包或损坏包进入控制层。
inline bool validateMasterCommand(const MasterCommandPacket &packet) {
    return packet.version == PROTOCOL_VERSION &&
           packet.type == PACKET_TYPE_MASTER_COMMAND &&
           packet.checksum == packetChecksum(&packet, offsetof(MasterCommandPacket, checksum));
}

// 收到从机遥测时验证类型、版本和校验和，防止错误状态污染主机显示。
inline bool validateSlaveTelemetry(const SlaveTelemetryPacket &packet) {
    return packet.version == PROTOCOL_VERSION &&
           packet.type == PACKET_TYPE_SLAVE_TELEMETRY &&
           packet.checksum == packetChecksum(&packet, offsetof(SlaveTelemetryPacket, checksum));
}

// 序号比较使用有符号差值，允许 uint32_t 溢出后仍能判断新旧。
inline bool isNewerSeq(uint32_t seq, uint32_t previous) {
    return static_cast<int32_t>(seq - previous) > 0;
}

