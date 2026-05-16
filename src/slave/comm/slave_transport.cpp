#include "slave/comm/slave_transport.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "common/system_state.h"
#include "slave/config/slave_config.h"
#include "slave/hardware/slave_hardware.h"
#include "slave/control/slave_motion.h"

// 从机 ESP-NOW 传输模块。
// 这里是无线包进出的唯一位置：从机接收 MasterCommandPacket，发送 SlaveTelemetryPacket。
// 运动控制和 UV 安全只读取已校验的命令快照，不直接依赖 ESP-NOW 回调参数。

namespace {

// rxPacket/txPacket 保存最近一次完整包，供控制、安全和状态任务读取。
// 固定长度结构体读写用 portMUX 短临界区保护，避免读到半包。
MasterCommandPacket rxPacket = {};
SlaveTelemetryPacket txPacket = {};
portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;

void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
    (void)mac;
    // ESP-NOW 发送回调只记录结果，不打印、不重发、不控制 UV 或电机。
    if (status == ESP_NOW_SEND_SUCCESS) {
        sysData.link.espnow_send_ok_count++;
        sysData.link.last_send_ok = 1;
    } else {
        sysData.link.espnow_send_fail_count++;
        sysData.link.last_send_ok = 0;
    }
}

void rejectCommand(uint16_t fault) {
    // 拒收命令时立即标记 command_valid=false，控制任务会回中心，安全任务会关 UV。
    sysData.link.command_valid = false;
    sysData.link.espnow_recv_reject_count++;
    addLocalFault(fault);
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    (void)mac;
    // 长度先验校验：固定长度命令包不允许短包或长包进入业务解析。
    if (len != static_cast<int>(sizeof(MasterCommandPacket))) {
        rejectCommand(FAULT_PACKET_SIZE);
        return;
    }

    // 复制到栈上临时对象后再校验，避免直接在回调参数内多处读取。
    MasterCommandPacket packet = {};
    memcpy(&packet, incomingData, sizeof(packet));
    // 版本/type/checksum 任一不匹配都会拒收。版本不一致通常表示两端代码或协议文档漂移。
    if (!validateMasterCommand(packet)) {
        rejectCommand((packet.version == PROTOCOL_VERSION) ? FAULT_CHECKSUM_ERROR : FAULT_VERSION_MISMATCH);
        return;
    }
    // 序号必须向前推进，防止旧命令覆盖新命令。
    if (sysData.link.command_valid && !isNewerSeq(packet.seq, sysData.link.last_command_seq)) {
        rejectCommand(FAULT_STALE_SEQUENCE);
        return;
    }

    // 通过校验后才发布命令快照。临界区只包住结构体赋值，保持回调很短。
    portENTER_CRITICAL(&commandMux);
    rxPacket = packet;
    portEXIT_CRITICAL(&commandMux);

    // 更新给控制、安全和状态任务使用的标量状态。
    sysData.master.x_pos = normToPercent(packet.x_norm);
    sysData.master.y_pos = normToPercent(packet.y_norm);
    sysData.link.current_mode = packet.mode;
    sysData.link.last_command_seq = packet.seq;
    sysData.link.last_rx_us = micros();
    sysData.link.command_valid = true;
    sysData.link.espnow_recv_ok_count++;
    publishProtocolFaults(FAULT_NONE);
}

}  // namespace

void setupSlaveEspNow() {
    // ESP-NOW 要求 Wi-Fi 处于 STA 或 AP/STA 模式；本项目固定使用 STA。
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        // 无线初始化失败时强制关 UV，并锁存命令超时类故障，方便串口暴露问题。
        setUvPen(false);
        addLocalFault(FAULT_COMMAND_TIMEOUT);
        return;
    }
    // 回调只做校验、快照和计数，不承担控制输出。
    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);

    // 当前阶段使用硬编码主机 MAC；channel=0 表示跟随当前 Wi-Fi 信道，不启用加密。
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, kSlavePeerMasterAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        addLocalFault(FAULT_COMMAND_TIMEOUT);
    }
}

void printSlaveEspNowIdentity() {
    // 打印本机 STA MAC 和目标主机 MAC，方便现场确认两块板是否烧录反了或 MAC 写错。
    uint8_t local_mac[6] = {};
    const esp_err_t result = esp_wifi_get_mac(WIFI_IF_STA, local_mac);
    if (result == ESP_OK) {
        Serial.printf("[Slave] sta_mac=%02x:%02x:%02x:%02x:%02x:%02x peer_master=%02x:%02x:%02x:%02x:%02x:%02x\n",
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
                      kSlavePeerMasterAddress[5]);
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

void sendSlaveTelemetry(uint32_t seq) {
    // 遥测优先回传当前实时故障；历史锁存值仍保留在本机状态行中。
    uint16_t faults = getActiveFaultFlags();
    if (sysData.slave.boundary_hit) {
        faults |= FAULT_BOUNDARY_HIT;
    }
    if (sysData.slave.uv_interlock_blocked) {
        faults |= FAULT_UV_INTERLOCK;
    }

    // 遥测包回传实际 X/Y、pen 输出状态、模式和 ack_seq。
    // ack_seq 是从机最近接受的主机命令序号，主机可用它判断链路闭环是否通畅。
    SlaveTelemetryPacket packet = {};
    packet.fault_flags = faults;
    packet.seq = seq;
    packet.ack_seq = sysData.link.last_command_seq;
    packet.timestamp_us = micros();
    packet.x_actual_norm = gimbalAngleRadToXNorm(sysData.slave.actual_angle_rad);
    packet.y_actual_norm = percentToNorm(sysData.slave.y_pos);
    packet.pen_state = sysData.link.pen_down ? 1 : 0;
    packet.mode = sysData.link.current_mode;
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
