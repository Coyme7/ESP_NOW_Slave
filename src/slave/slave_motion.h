#pragma once

#include <stdint.h>

// slave_motion
// 职责：隔离纸面坐标到云台角度的映射，以及 X 轴本地控制单步。
// 热路径说明：runSlaveControlStep() 不做串口、ESP-NOW 发送、UV GPIO 写或动态内存；
// 只消费最新命令快照并发布实际角度、纸面百分比和故障位。

// 将协议 x_norm 转成纸面毫米坐标，范围约为 -95..+95 mm。
float xNormToPaperMm(int16_t x_norm);

// 将纸面 X 毫米位置转成云台 X 轴目标角，使用 atan(x / 投影距离)。
float paperMmToGimbalAngleRad(float x_mm);

// 将云台实际角反推回协议归一化坐标，供遥测 x_actual_norm 使用。
int16_t gimbalAngleRadToXNorm(float angle_rad);

// 从机 X 轴控制单步：读取命令快照、计算目标、更新实际角和 fault。
void runSlaveControlStep();
