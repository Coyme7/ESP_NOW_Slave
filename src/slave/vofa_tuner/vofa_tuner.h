#pragma once

#include <stdint.h>

#include "common/math/axis_math.h"
#include "slave/vofa_tuner/vofa_tuner_config.h"

struct SlaveControlStepTiming;

enum SlaveVofaTunerMode : uint8_t {
    SLAVE_VOFA_MODE_OFF = 0,
    SLAVE_VOFA_MODE_CURRENT_Q = 1,
    SLAVE_VOFA_MODE_VELOCITY = 2,
    SLAVE_VOFA_MODE_ANGLE = 3,
    SLAVE_VOFA_MODE_CURRENT_RAW = 4,
};

// Core 0 入口：解析 ASCII 命令，并按 20~50 ms 周期输出 FireWater CSV。
void runSlaveVofaTunerIoStep();

// 硬件初始化完成前，VOFA 串口仍可响应，但拒绝访问电机快照和控制目标。
void setSlaveVofaHardwareReady(bool ready);

// Core 1 入口：消费 pending 配置，应用 SimpleFOC 参数并运行单轴慢速三角波。
// 返回 true 表示本控制周期由 tuner 接管，常规 planner/ESP-NOW 目标必须跳过。
bool runSlaveVofaTunerControlStep(float dt_s, SlaveControlStepTiming *timing);

// safety 任务使用。请求进入后立即阻断 UV，不等待 Core 1 完成 mode 切换。
bool isSlaveVofaTunerRequestedOrActive();
