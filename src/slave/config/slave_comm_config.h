#pragma once

#include <stdint.h>

// 从机 ESP-NOW 通信配置。
//
// 本文件只放无线链路本地配置，不定义协议包结构；协议结构仍在 common/protocol。

// 从机 ESP-NOW 通信开关。
// 默认值：1，主从联调默认启用无线命令接收和遥测发送。
// 用途：单机隔离运动、安全或硬件问题时可设为 0。
// 风险：关闭后不会接收主机命令，控制路径只能按超时/默认目标运行。
// 依赖：不影响控制任务创建；只裁剪通信任务和 ESP-NOW 初始化。
#ifndef SLAVE_ESPNOW_ENABLED
#define SLAVE_ESPNOW_ENABLED 1
#endif

// 从机 ESP-NOW 固定信道。
// 默认值：1，必须与 MASTER_ESPNOW_CHANNEL 保持一致。
// 用途：避免 channel=0 跟随当前 Wi-Fi 信道导致链路不确定。
// 风险：主从信道不一致会导致无法通信。
// 依赖：setupSlaveEspNow() 会调用 esp_wifi_set_channel()，peerInfo.channel 也使用该值。
#ifndef SLAVE_ESPNOW_CHANNEL
#define SLAVE_ESPNOW_CHANNEL 1
#endif

static_assert(SLAVE_ESPNOW_CHANNEL >= 1 && SLAVE_ESPNOW_CHANNEL <= 14,
              "SLAVE_ESPNOW_CHANNEL must be in 1..14");

// 固定主机 MAC。
// 默认值：在 slave_config.cpp 中定义当前主机 STA MAC。
// 用途：ESP-NOW peer 添加和遥测发送目标。
// 风险：更换主机开发板后必须更新该地址，否则从机无法发回遥测。
// 依赖：当前阶段不做自动配对或扫描。
extern const uint8_t kSlavePeerMasterAddress[6];
