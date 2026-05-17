#pragma once

#include <stdint.h>

// slave_tasks
// 职责：集中从机 FreeRTOS task loop 和任务创建。
// Core 1 只跑控制热路径；Core 0 处理通信、紫光安全和状态输出。

// 创建并启动从机所有任务。调用前必须已经完成安全输出、硬件初始化和 ESP-NOW 初始化。
void startSlaveTasks();

struct SlaveControlHealthSnapshot {
    uint32_t last_dt_us;
    uint32_t max_dt_us;
    uint32_t missed_delta;
    uint32_t missed_total;
    uint32_t step_us;
    uint32_t step_max_us;
    uint32_t command_us;
    uint32_t command_max_us;
    uint32_t trajectory_us;
    uint32_t trajectory_max_us;
    uint32_t motor_us;
    uint32_t motor_max_us;
    uint32_t x_sensor_us;
    uint32_t x_sensor_max_us;
    uint32_t x_foc_us;
    uint32_t x_foc_max_us;
    uint32_t x_move_us;
    uint32_t x_move_max_us;
    uint32_t y_sensor_us;
    uint32_t y_sensor_max_us;
    uint32_t y_foc_us;
    uint32_t y_foc_max_us;
    uint32_t y_move_us;
    uint32_t y_move_max_us;
    uint32_t state_us;
    uint32_t state_max_us;
    uint32_t publish_us;
    uint32_t publish_max_us;
    uint32_t x_foc_run_delta;
    uint32_t x_foc_skip_delta;
    uint32_t x_foc_divisor;
    uint32_t y_foc_run_delta;
    uint32_t y_foc_skip_delta;
    uint32_t y_foc_divisor;
    uint32_t step_over_period_delta;
    uint32_t step_over_75pct_delta;
    uint32_t step_over_50pct_delta;
    uint32_t step_over_200_delta;
    uint32_t step_over_300_delta;
};

// 控制定时诊断。只在低频状态任务读取，用于确认 ISR timer 是否生效以及是否丢 tick。
uint32_t getSlaveControlTimerMissedTicks();
uint32_t getSlaveControlLastDtUs();
SlaveControlHealthSnapshot getSlaveControlHealthSnapshot();
