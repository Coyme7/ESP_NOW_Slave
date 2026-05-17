#pragma once

#include <stdint.h>

#include "common/math/axis_math.h"

// 从机 X/Y 共用坐标映射纯函数。
// 这里只处理协议坐标、纸面毫米和云台机械角之间的换算，不访问电机、UV 或 ESP-NOW。
float slaveAxisHalfRangeMm(AxisId axis);
float slaveAxisNormToPaperMm(AxisId axis, int16_t norm);
float slaveClampAxisPaperMm(AxisId axis, float mm, bool *clamped);
float slaveAxisPaperMmToGimbalAngleRad(AxisId axis, float mm);
int16_t slaveAxisGimbalAngleRadToNorm(AxisId axis, float angle_rad);

// 兼容旧 X 单轴入口，保持现有 X 同步链路不退化。
float slaveXNormToPaperMm(int16_t x_norm);
float slavePaperMmToGimbalAngleRad(float x_mm);
int16_t slaveGimbalAngleRadToXNorm(float angle_rad);
