#pragma once

#include <stdint.h>

#include "common/protocol/protocol_types.h"
#include "slave/control/slave_runtime_snapshot.h"

struct SlaveControlStepTiming {
    uint32_t command_us;
    uint32_t trajectory_us;
    uint32_t motor_us;
    uint32_t x_sensor_us;
    uint32_t x_foc_us;
    uint32_t x_foc_ran;
    uint32_t x_move_us;
    uint32_t y_sensor_us;
    uint32_t y_foc_us;
    uint32_t y_foc_ran;
    uint32_t y_move_us;
    uint32_t state_us;
    uint32_t publish_us;
};

void runSlaveControlStep(float dt_s, SlaveControlStepTiming *timing);
void runSlaveControlPerfIsolationStep(float dt_s, SlaveControlStepTiming *timing);
void runSlavePlannerStep(float dt_s, SlaveControlStepTiming *timing);
void runSlaveMotorStep(SlaveControlStepTiming *timing);
SlaveMotionSnapshot snapshotSlaveMotion();
bool acceptSlaveTrajectorySegment(const TrajectorySegmentPacket &packet, uint32_t now_us);

// slave_motion
// 职责：调度 planner、motor tick 和运动状态发布。
// 热路径说明：runSlaveControlStep() 不做串口、ESP-NOW 发送、UV GPIO 写或动态内存；
// 只消费最新命令快照并发布实际角度、纸面百分比和故障位。

// 从机控制单步：调度低频 planner 和当前 run mode 的 motor tick；真实 DualXY 基准为 4kHz。
void runSlaveControlStep(float dt_s);
void runSlaveControlPerfIsolationStep(float dt_s);
