#pragma once

#include <stdint.h>

// 从机 ESP-NOW 通信配置。
// 协议包结构仍在 common/protocol，本文件只放本地链路配置。

// 功能说明：启用 ESP-NOW 从机通信。
// 0：不初始化 ESP-NOW；1：接收主机命令并发送从机遥测。
#ifndef SLAVE_ESPNOW_ENABLED
#define SLAVE_ESPNOW_ENABLED 1
#endif

// 从机 ESP-NOW 固定信道。
// 必须与主机 `MASTER_ESPNOW_CHANNEL` 保持一致。
#ifndef SLAVE_ESPNOW_CHANNEL
#define SLAVE_ESPNOW_CHANNEL 1
#endif

// 当前固定主机 STA MAC。
// 更换主机板后必须更新该地址，自动配对加入后再替换。
extern const uint8_t kSlavePeerMasterAddress[6];
