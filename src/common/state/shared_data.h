#pragma once

#include "common/state/link_state.h"

// 主机调试状态：由控制热路径低频发布，状态任务负责打印。
struct MasterDebugState {
    // 角度，单位由所属结构语义决定：主机为控制角，从机为遥测角。
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
    volatile float angle_deg;
    volatile float x_pos;
    volatile float y_pos;
    // 从机目标角度，单位 rad。
    volatile float target_angle_rad;
    // 从机实际角度，单位 rad。
    volatile float actual_angle_rad;
    // 从机命令纸面 X，单位 mm。
    volatile float target_x_mm;
    // 从机平滑后的纸面 X，单位 mm。
    volatile float smooth_x_mm;
    volatile bool boundary_hit;
    // 从机 UV/落笔互锁是否阻止输出。
    volatile bool uv_interlock_blocked;
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
