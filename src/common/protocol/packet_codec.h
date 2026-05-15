#pragma once

#include <stddef.h>
#include <stdint.h>

#include "common/protocol/protocol_types.h"

// 简单反码累加校验。checksum 字段本身不参与计算。
inline uint16_t packetChecksum(const void *data, size_t len) {
    const uint8_t *bytes = static_cast<const uint8_t *>(data);
    uint16_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum = static_cast<uint16_t>(sum + bytes[i]);
    }
    return static_cast<uint16_t>(~sum);
}

inline void finalizeMasterCommand(MasterCommandPacket &packet) {
    packet.version = PROTOCOL_VERSION;
    packet.type = PACKET_TYPE_MASTER_COMMAND;
    packet.checksum = packetChecksum(&packet, offsetof(MasterCommandPacket, checksum));
}

inline void finalizeSlaveTelemetry(SlaveTelemetryPacket &packet) {
    packet.version = PROTOCOL_VERSION;
    packet.type = PACKET_TYPE_SLAVE_TELEMETRY;
    packet.checksum = packetChecksum(&packet, offsetof(SlaveTelemetryPacket, checksum));
}

inline bool validateMasterCommand(const MasterCommandPacket &packet) {
    return packet.version == PROTOCOL_VERSION &&
           packet.type == PACKET_TYPE_MASTER_COMMAND &&
           packet.checksum == packetChecksum(&packet, offsetof(MasterCommandPacket, checksum));
}

inline bool validateSlaveTelemetry(const SlaveTelemetryPacket &packet) {
    return packet.version == PROTOCOL_VERSION &&
           packet.type == PACKET_TYPE_SLAVE_TELEMETRY &&
           packet.checksum == packetChecksum(&packet, offsetof(SlaveTelemetryPacket, checksum));
}

inline bool isNewerSeq(uint32_t seq, uint32_t previous) {
    return static_cast<int32_t>(seq - previous) > 0;
}

