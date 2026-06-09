#include "slave/comm/slave_transport.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>

#include "common/protocol/packet_codec.h"
#include "common/system_state.h"
#include "slave/config/slave_config.h"
#include "slave/hardware/slave_hardware.h"
#include "slave/control/slave_motion.h"
#include "slave/control/slave_coordinate_mapper.h"
#include "slave/modes/mode_guard.h"
#include "slave/modes/mode_manager.h"

// 从机 ESP-NOW 传输模块。
// 这里是无线包进出的唯一位置：从机接收 MasterCommandPacket / TrajectorySegmentPacket，
// 发送 SlaveTelemetryPacket。
// 运动控制和 UV 安全只读取已校验的命令快照，不直接依赖 ESP-NOW 回调参数。

namespace {

// rxPacket 保留最近一次完整协议包，供低频状态兼容读取。
// rtCommand 是控制 planner 读取的紧凑实时命令缓存，避免热路径复制完整包。
MasterCommandPacket rxPacket = {};
MasterCommandPacket rxPendingPacket = {};
TrajectorySegmentPacket rxPendingTrajectoryPacket = {};
SlaveRtCommand rtCommand = {};
SlaveTelemetryPacket txPacket = {};
portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE commandPendingMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool rxPending = false;
volatile int rxPacketLen = 0;
volatile bool rxTrajectoryPending = false;
volatile int rxTrajectoryPacketLen = 0;
bool hasAcceptedCommand = false;
uint32_t lastAcceptedCommandSeq = 0;
bool hasAcceptedTrajectorySegment = false;
uint32_t lastAcceptedTrajectorySeq = 0;

void handleDataSent(esp_now_send_status_t status) {
    // ESP-NOW 发送回调只记录结果，不打印、不重发、不控制 UV 或电机。
    if (status == ESP_NOW_SEND_SUCCESS) {
        sysData.link.espnow_send_ok_count++;
        sysData.link.last_send_ok = 1;
    } else {
        sysData.link.espnow_send_fail_count++;
        sysData.link.last_send_ok = 0;
    }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void onDataSent(const esp_now_send_info_t *txInfo, esp_now_send_status_t status) {
    (void)txInfo;
    handleDataSent(status);
}
#else
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
    (void)mac;
    handleDataSent(status);
}
#endif

void recordRejectedCommandFault(uint16_t fault) {
    // 协议错误包只计数并锁存为本地故障。
    // 不直接 publishProtocolFaults(fault)，避免通信任务覆盖 planner/safety 发布的 runtime active fault。
    sysData.link.espnow_recv_reject_count++;
    addLocalFault(fault);
}

void recordStaleCommandOnly(uint32_t seq) {
    // stale 属于链路乱序或旧包，只计数，不覆盖命令、不清 valid、不锁存 fault。
    (void)seq;
    sysData.link.espnow_recv_reject_count++;
    sysData.link.espnow_recv_stale_count++;
}

void recordDuplicateCommandOnly(uint32_t seq) {
    // duplicate 属于链路重复包，只计数，不覆盖命令、不清 valid、不锁存 fault。
    (void)seq;
    sysData.link.espnow_recv_reject_count++;
    sysData.link.espnow_recv_duplicate_count++;
}

void handleDataRecv(const uint8_t *incomingData, int len) {
    // 回调只复制最新 raw packet，不做协议校验、seq 判断、fault 更新或 sysData 写入。
    portENTER_CRITICAL(&commandPendingMux);
    const int packet_len = (incomingData != nullptr) ? len : 0;
    const uint8_t packet_type =
        (incomingData != nullptr && packet_len >= 4) ? incomingData[3] : 0U;

    if (packet_type == PACKET_TYPE_TRAJECTORY_SEGMENT) {
        rxTrajectoryPacketLen = packet_len;
        if (packet_len > 0 && packet_len <= static_cast<int>(sizeof(rxPendingTrajectoryPacket))) {
            memcpy(&rxPendingTrajectoryPacket, incomingData, static_cast<size_t>(packet_len));
        }
        rxTrajectoryPending = true;
    } else {
        rxPacketLen = packet_len;
        if (packet_len > 0 && packet_len <= static_cast<int>(sizeof(rxPendingPacket))) {
            memcpy(&rxPendingPacket, incomingData, static_cast<size_t>(packet_len));
        }
        rxPending = true;
    }
    portEXIT_CRITICAL(&commandPendingMux);
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
    (void)recvInfo;
    handleDataRecv(incomingData, len);
}
#else
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    (void)mac;
    handleDataRecv(incomingData, len);
}
#endif

}  // namespace

void setupSlaveEspNow() {
    // ESP-NOW 要求 Wi-Fi 处于 STA 或 AP/STA 模式；本项目固定使用 STA。
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    (void)esp_wifi_set_ps(WIFI_PS_NONE);
    (void)esp_wifi_set_channel(SLAVE_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (esp_now_init() != ESP_OK) {
        // 无线初始化失败时锁存命令超时类故障；UV 输出只由启动安全态和 safety 任务负责。
        sysData.link.link_state = LINK_FAULT;
        addLocalFault(FAULT_COMMAND_TIMEOUT);
        return;
    }
    // 回调只复制 raw packet；校验、序号判断、故障和状态更新都在通信任务中完成。
    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);

    // 当前阶段使用硬编码主机 MAC；固定信道必须与主机一致，不启用加密。
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, kSlavePeerMasterAddress, 6);
    peerInfo.channel = SLAVE_ESPNOW_CHANNEL;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        sysData.link.link_state = LINK_PAIRING;
        addLocalFault(FAULT_COMMAND_TIMEOUT);
    } else {
        sysData.link.link_state = LINK_PAIRING;
    }
}

void printSlaveEspNowIdentity() {
    // 打印本机 STA MAC 和目标主机 MAC，方便现场确认两块板是否烧录反了或 MAC 写错。
    uint8_t local_mac[6] = {};
    const esp_err_t result = esp_wifi_get_mac(WIFI_IF_STA, local_mac);
    if (result == ESP_OK) {
        Serial.printf("[Slave] sta_mac=%02x:%02x:%02x:%02x:%02x:%02x peer_master=%02x:%02x:%02x:%02x:%02x:%02x channel=%u\n",
                      local_mac[0],
                      local_mac[1],
                      local_mac[2],
                      local_mac[3],
                      local_mac[4],
                      local_mac[5],
                      kSlavePeerMasterAddress[0],
                      kSlavePeerMasterAddress[1],
                      kSlavePeerMasterAddress[2],
                      kSlavePeerMasterAddress[3],
                      kSlavePeerMasterAddress[4],
                      kSlavePeerMasterAddress[5],
                      static_cast<unsigned int>(SLAVE_ESPNOW_CHANNEL));
    }
}

MasterCommandPacket snapshotMasterCommand() {
    // 控制、安全、状态任务读取命令快照。临界区尽量短，只复制固定长度结构体。
    MasterCommandPacket packet = {};
    portENTER_CRITICAL(&commandMux);
    packet = rxPacket;
    portEXIT_CRITICAL(&commandMux);
    return packet;
}

SlaveRtCommand snapshotSlaveRtCommand() {
    SlaveRtCommand snapshot = {};
    portENTER_CRITICAL(&commandMux);
    snapshot = rtCommand;
    portEXIT_CRITICAL(&commandMux);
    return snapshot;
}

void processTrajectorySegmentPacket(const TrajectorySegmentPacket &packet, int packet_len) {
    if (packet_len != static_cast<int>(sizeof(TrajectorySegmentPacket))) {
        recordRejectedCommandFault(FAULT_PACKET_SIZE);
        return;
    }

    if (!validateTrajectorySegment(packet)) {
        recordRejectedCommandFault((packet.version == PROTOCOL_VERSION) ? FAULT_CHECKSUM_ERROR : FAULT_VERSION_MISMATCH);
        return;
    }

    const SlaveRtCommand command = snapshotSlaveRtCommand();
    if (!slaveCommandRequestsTrajectory(command)) {
        recordRejectedCommandFault(FAULT_TRAJECTORY_INVALID);
        return;
    }

    if (hasAcceptedTrajectorySegment) {
        if (packet.seq == lastAcceptedTrajectorySeq) {
            recordDuplicateCommandOnly(packet.seq);
            return;
        }
        if (!isNewerSeq(packet.seq, lastAcceptedTrajectorySeq)) {
            recordStaleCommandOnly(packet.seq);
            return;
        }
    }

    if (!acceptSlaveTrajectorySegment(packet, micros())) {
        recordRejectedCommandFault(FAULT_TRAJECTORY_INVALID);
        return;
    }

    sysData.link.espnow_recv_ok_count++;
    hasAcceptedTrajectorySegment = true;
    lastAcceptedTrajectorySeq = packet.seq;
}

void processSlaveCommand() {
    MasterCommandPacket packet = {};
    int packet_len = 0;
    bool has_packet = false;
    TrajectorySegmentPacket trajectory_packet = {};
    int trajectory_packet_len = 0;
    bool has_trajectory_packet = false;

    portENTER_CRITICAL(&commandPendingMux);
    if (rxPending) {
        packet_len = rxPacketLen;
        if (packet_len == static_cast<int>(sizeof(packet))) {
            packet = rxPendingPacket;
        }
        rxPending = false;
        has_packet = true;
    }
    if (rxTrajectoryPending) {
        trajectory_packet_len = rxTrajectoryPacketLen;
        if (trajectory_packet_len == static_cast<int>(sizeof(trajectory_packet))) {
            trajectory_packet = rxPendingTrajectoryPacket;
        }
        rxTrajectoryPending = false;
        has_trajectory_packet = true;
    }
    portEXIT_CRITICAL(&commandPendingMux);

    if (has_trajectory_packet) {
        processTrajectorySegmentPacket(trajectory_packet, trajectory_packet_len);
    }

    if (!has_packet) {
        return;
    }

    // 长度先验校验：固定长度命令包不允许短包或长包进入业务解析。
    if (packet_len != static_cast<int>(sizeof(MasterCommandPacket))) {
        recordRejectedCommandFault(FAULT_PACKET_SIZE);
        return;
    }

    // magic/version/type/CRC 任一不匹配都会拒收。版本不一致通常表示两端代码或协议文档漂移。
    if (!validateMasterCommand(packet)) {
        recordRejectedCommandFault((packet.version == PROTOCOL_VERSION) ? FAULT_CHECKSUM_ERROR : FAULT_VERSION_MISMATCH);
        return;
    }

    // 序号必须向前推进。
    // duplicate/stale 只丢弃并计数，不清 command_valid，不覆盖实时命令缓存，不锁存 fault。
    if (hasAcceptedCommand) {
        if (packet.seq == lastAcceptedCommandSeq) {
            recordDuplicateCommandOnly(packet.seq);
            return;
        }

        if (!isNewerSeq(packet.seq, lastAcceptedCommandSeq)) {
            recordStaleCommandOnly(packet.seq);
            return;
        }
    }

    const uint32_t now_us = micros();
    const uint8_t accepted_protocol_mode =
        slaveAcceptedProtocolModeForCommand(packet.mode, packet.command_flags);
    const SlaveRtCommand next_command = {
        packet.x_norm,
        packet.y_norm,
        packet.seq,
        now_us,
        accepted_protocol_mode,
        packet.pen_req,
        packet.command_flags,
        1U,
    };

    // runtime app mode 由已校验协议语义驱动；SLAVE_RUN_MODE 只限制编译期能力。
    updateSlaveRuntimeModeFromCommand(packet.mode, packet.command_flags, now_us / 1000UL);

    // 通过校验后才发布命令快照。临界区只包住结构体赋值，保持通信任务可预期。
    portENTER_CRITICAL(&commandMux);
    rxPacket = packet;
    rtCommand = next_command;
    portEXIT_CRITICAL(&commandMux);

    // 更新给控制、安全和状态任务使用的标量状态。
    sysData.master.x_pos = normToPercent(packet.x_norm);
    sysData.master.y_pos = normToPercent(packet.y_norm);
    sysData.link.current_mode = accepted_protocol_mode;
    sysData.link.last_command_seq = packet.seq;
    sysData.link.last_rx_us = now_us;
    sysData.link.command_valid = true;
    sysData.link.link_state = LINK_CONNECTED;
    sysData.link.espnow_recv_ok_count++;
    hasAcceptedCommand = true;
    lastAcceptedCommandSeq = packet.seq;
}

void sendSlaveTelemetry(uint32_t seq) {
    const SlaveMotionSnapshot motion = snapshotSlaveMotion();
    const SlaveRtCommand command = snapshotSlaveRtCommand();

    // 遥测优先回传当前实时故障；历史锁存值仍保留在本机状态行中。
    uint16_t faults = getActiveFaultFlags();
    if (motion.boundary_hit) {
        faults |= FAULT_BOUNDARY_HIT;
    }
    if (sysData.slave.uv_interlock_blocked) {
        faults |= FAULT_UV_INTERLOCK;
    }

    // 遥测包回传实际 X/Y、pen 状态机、UV 实际输出、最近接受的链路语义和 ack_seq。
    // ack_seq 是从机最近接受的主机命令序号，主机可用它判断链路闭环是否通畅。
    SlaveTelemetryPacket packet = {};
    packet.fault_flags = faults;
    packet.seq = seq;
    packet.ack_seq = command.seq;
    packet.timestamp_us = micros();
    packet.x_actual_norm = slaveAxisGimbalAngleRadToNorm(AXIS_X, motion.actual_angle_rad);
    packet.y_actual_norm = slaveAxisGimbalAngleRadToNorm(AXIS_Y, motion.actual_y_angle_rad);
    packet.pen_state = sysData.slave.pen_state;
    packet.mode = sysData.link.current_mode;
    packet.uv_block_reasons = sysData.slave.uv_block_reasons;
    packet.draw_state = motion.draw_state;
    packet.draw_progress_pct = motion.draw_progress_pct;
    packet.link_state = sysData.link.link_state;
    packet.uv_out = sysData.link.uv_out ? 1 : 0;
    packet.trajectory_task_id = motion.trajectory_task_id;
    packet.trajectory_segment_count = motion.trajectory_segment_count;
    packet.trajectory_segment_cursor = motion.trajectory_segment_cursor;
    packet.trajectory_received_count = motion.trajectory_received_count;
    packet.trajectory_status_flags = motion.trajectory_status_flags;
    packet.trajectory_received_mask_low = motion.trajectory_received_mask_low;
    packet.trajectory_received_mask_high = motion.trajectory_received_mask_high;
    finalizeSlaveTelemetry(packet);

    // 保存最近一次遥测包，便于后续扩展状态读取或调试。
    portENTER_CRITICAL(&telemetryMux);
    txPacket = packet;
    portEXIT_CRITICAL(&telemetryMux);

    // esp_now_send 只表示提交到 ESP-NOW 栈；真正发送结果会在 onDataSent 回调中统计。
    const esp_err_t send_result =
        esp_now_send(kSlavePeerMasterAddress, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    if (send_result != ESP_OK) {
        sysData.link.espnow_send_fail_count++;
        sysData.link.last_send_ok = 0;
    }
}
