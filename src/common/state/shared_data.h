#pragma once

#include "common/state/link_state.h"

// 主机调试状态：由控制热路径低频发布，状态任务负责打印。
struct MasterDebugState {
    // 兼容旧字段：主机 X 旋钮控制角，单位 deg。
    volatile float angle_deg;
    // 主机力反馈最终目标电流，单位 A。
    volatile float target_current_a;
    // SimpleFOC q 轴实际电流，单位 A。
    volatile float current_q_a;
    // SimpleFOC d 轴实际电流，单位 A。
    volatile float current_d_a;
    // q 轴输出电压，单位 V。
    volatile float voltage_q_v;
    // d 轴输出电压，单位 V。
    volatile float voltage_d_v;
    // X 轴位置百分比，用于状态显示。
    volatile float x_pos;
    // Y 轴位置百分比，单轴阶段通常保持默认值。
    volatile float y_pos;
    // 是否命中边界或处于边界相关状态。
    volatile bool boundary_hit;
};

// 从机遥测状态：由主机通信回调解析从机包后更新。
struct SlaveDebugState {
    // 兼容旧字段：从机 X 轴实际角度，单位 deg。
    volatile float angle_deg;
    volatile float x_pos;
    volatile float y_pos;
    // 从机 X 轴目标角度，单位 rad。保留旧字段名，兼容当前 X 单轴遥测。
    volatile float target_angle_rad;
    // 从机 X 轴实际角度，单位 rad。控制步优先使用 motor.shaft_angle，不额外读取编码器。
    volatile float actual_angle_rad;
    // 从机命令纸面 X，单位 mm。
    volatile float target_x_mm;
    // 从机平滑后的纸面 X，单位 mm。
    volatile float smooth_x_mm;
    // 从机 Y 轴目标角度，单位 rad；Y 硬件关闭时由软件框架或仿真更新。
    volatile float target_y_angle_rad;
    // 从机 Y 轴实际角度，单位 rad；Y 硬件关闭时保持仿真或 0。
    volatile float actual_y_angle_rad;
    // 从机命令纸面 Y，单位 mm。
    volatile float target_y_mm;
    // 从机平滑后的纸面 Y，单位 mm。
    volatile float smooth_y_mm;
    // X/Y 目标是否碰到软件限位。
    volatile bool x_limit_hit;
    volatile bool y_limit_hit;
    // X/Y 命令是否被 clamp 到安全范围。
    volatile bool x_clamped;
    volatile bool y_clamped;
    volatile bool boundary_hit;
    // 从机 UV/落笔互锁是否阻止输出。
    volatile bool uv_interlock_blocked;
    // 从机 pen 状态机状态，取 PenState。
    volatile uint8_t pen_state;
    // 从机自动绘图状态，取 DrawState。
    volatile uint8_t draw_state;
    // 自动绘图进度百分比，范围 0..100。
    volatile uint8_t draw_progress_pct;
    // 自动绘图轨迹传输/执行状态，来自从机本地轨迹状态。
    volatile uint16_t trajectory_task_id;
    volatile uint8_t trajectory_segment_count;
    volatile uint8_t trajectory_segment_cursor;
    volatile uint8_t trajectory_received_count;
    volatile uint8_t trajectory_status_flags;
    volatile uint32_t trajectory_received_mask_low;
    volatile uint16_t trajectory_received_mask_high;
    // UV 被禁止的具体原因位，取 UvBlockReason。
    volatile uint16_t uv_block_reasons;
};

// 全局共享状态容器。当前工程用轻量方式访问，热路径只写少量字段。
struct SharedData {
    // 主从链路统计和超时状态。
    CommonLinkState link;
    // 主机本地调试状态。
    MasterDebugState master;
    // 从机遥测调试状态。
    SlaveDebugState slave;
};

// sysData 在 system_state.cpp 中定义，各模块通过 extern 访问同一个实例。
extern SharedData sysData;
