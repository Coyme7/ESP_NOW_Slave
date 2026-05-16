#pragma once

#include <stdint.h>

// 从机 X 单轴坐标映射纯函数。
// 只处理协议坐标、纸面毫米和云台机械角之间的换算，不访问电机、UV 或 ESP-NOW。
float slaveXNormToPaperMm(int16_t x_norm);
float slavePaperMmToGimbalAngleRad(float x_mm);
int16_t slaveGimbalAngleRadToXNorm(float angle_rad);
