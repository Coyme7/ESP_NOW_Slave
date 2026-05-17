#pragma once

#include <stdint.h>

// 通信任务校验后发布给 planner 的实时命令缓存。
// 它只保留同步所需字段，避免控制路径复制完整协议包和无关业务状态。
struct SlaveRtCommand {
    int16_t x_norm;
    int16_t y_norm;
    uint32_t seq;
    uint32_t last_rx_us;
    uint8_t mode;
    uint8_t pen_down;
    uint8_t valid;
};

// 兼容旧名称。新代码优先使用 SlaveRtCommand。
using SlaveControlInputSnapshot = SlaveRtCommand;

// 控制任务发布给 safety / telemetry / status 的运动快照。
// 该结构只表达运动与命令状态；timing 诊断统一走 SlaveControlHealthSnapshot，
// 避免把控制任务耗时字段混入运动快照语义。
// 该结构由控制任务短临界区写入，低频任务只读它，不直接读取传感器。
struct SlaveMotionSnapshot {
    float target_x_mm;
    float smooth_x_mm;
    float target_angle_rad;
    float actual_angle_rad;
    float x_track_err_mrad;
    float target_y_mm;
    float smooth_y_mm;
    float target_y_angle_rad;
    float actual_y_angle_rad;
    float y_track_err_mrad;
    bool x_limit;
    bool y_limit;
    bool x_clamped;
    bool y_clamped;
    bool boundary_hit;
    uint8_t command_valid;
    uint8_t pen_down;
};

// 兼容旧名称。后续旧引用迁移完后可删除。
using SlaveControlOutputSnapshot = SlaveMotionSnapshot;
