#pragma once

#include "common/protocol/protocol_types.h"
#include "slave/control/slave_runtime_snapshot.h"

// slave_transport
// 职责：从机 ESP-NOW 初始化、接收主机命令/轨迹段、发送 SlaveTelemetry，并维护命令快照。
// 回调约束：只复制 raw packet、记录长度并设置 pending flag；
// 校验、序号判断、fault 和 sysData 更新全部放在 SlaveComm 任务中执行。

// 初始化 Wi-Fi STA、ESP-NOW、回调和固定主机 peer。
void setupSlaveEspNow();

// 打印本机 STA MAC 和硬编码主机 MAC，用于烧录对象和配对地址核对。
void printSlaveEspNowIdentity();

// 读取最近一次接受的完整主机命令快照，只供低频兼容状态路径使用。
MasterCommandPacket snapshotMasterCommand();

// 读取通信任务校验后的实时命令缓存，供 planner、安全和状态任务使用。
SlaveRtCommand snapshotSlaveRtCommand();

// 由 Core 0 通信任务调用，处理 ESP-NOW 回调留下的最新 pending 命令。
void processSlaveCommand();

// 由 Core 0 通信任务周期调用，发送当前从机遥测和 fault flags。
void sendSlaveTelemetry(uint32_t seq);
