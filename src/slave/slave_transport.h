#pragma once

#include "shared_types.h"

// slave_transport
// 职责：从机 ESP-NOW 初始化、接收主机命令、发送 SlaveTelemetry，并维护命令快照。
// 回调约束：只做长度/版本/checksum/序号校验、短临界区复制和标量状态更新；
// 不驱动电机、不打印、不分配内存、不阻塞。

// 初始化 Wi-Fi STA、ESP-NOW、回调和固定主机 peer。
void setupSlaveEspNow();

// 打印本机 STA MAC 和硬编码主机 MAC，用于烧录对象和配对地址核对。
void printSlaveEspNowIdentity();

// 读取最近一次接受的主机命令快照，供控制、安全和状态任务使用。
MasterCommandPacket snapshotMasterCommand();

// 由 Core 0 通信任务周期调用，发送当前从机遥测和 fault flags。
void sendSlaveTelemetry(uint32_t seq);
