#pragma once

#include <stdint.h>

// 从机控制周期与热路径配置。
//
// 默认值面向 SingleX 5kHz 验收：电机 tick 保持 200us，planner 和状态发布降频。
// 本文件不访问硬件，只提供控制任务和硬件适配层读取的编译期常量。

// 从机本地控制周期。
// 默认值：200us，对应 5kHz motor tick。
// 用途：驱动 esp_timer 控制任务节拍，并作为 over_period 诊断基准。
// 风险：降低周期会压缩 SimpleFOC 和 SPI 余量；提高周期会降低跟随带宽。
// 依赖：FreeRTOS tick 固定 1ms，SLAVE_CONTROL_STEPS_PER_TICK 依赖该值能整除 1000us。
static constexpr uint32_t SLAVE_CONTROL_LOOP_PERIOD_US = 200UL;
static constexpr uint32_t CONTROL_LOOP_PERIOD_US = SLAVE_CONTROL_LOOP_PERIOD_US;

// 从机 planner 执行分频。
// 默认值：5，5kHz 控制下约 1kHz 更新命令解析、平滑、限幅和坐标映射。
// 用途：把 atanf、轨迹平滑和命令快照读取移出每个 motor tick。
// 风险：值过大会增加目标更新延迟；值为 1 会退回每 200us 规划一次。
// 依赖：motor tick 仍每周期消费最近一次 planner 预计算目标。
#ifndef SLAVE_PLANNER_EVERY_N_STEPS
#define SLAVE_PLANNER_EVERY_N_STEPS 5UL
#endif

// 从机 runtime 发布分频。
// 默认值：10，5kHz 控制下约 500Hz 更新 sysData 显示/统计字段。
// 用途：降低跨任务 volatile 写入和 actual->norm/mm 反算开销。
// 风险：值过大会让串口显示和遥测辅助字段变慢；不影响电机闭环目标。
// 依赖：安全和遥测应读取 SlaveMotionSnapshot，不依赖 sysData 高频刷新。
#ifndef SLAVE_RUNTIME_PUBLISH_EVERY_N_STEPS
#define SLAVE_RUNTIME_PUBLISH_EVERY_N_STEPS 10UL
#endif

// 从机运动快照发布分频。
// 默认值：5，5kHz 控制下约 1kHz 更新 SlaveMotionSnapshot。
// 用途：降低 5kHz 热路径中的 float 拷贝和临界区写入开销，状态、安全和遥测读取该快照。
// 风险：状态、安全和遥测读取到的实际位置最多滞后约 1ms；不影响电机闭环目标和 SimpleFOC 执行。
// 依赖：只用于 FULL_CONTROL；性能隔离模式默认不发布快照，避免污染 sensor/loopFOC/move 单项测量。
#ifndef SLAVE_MOTION_SNAPSHOT_EVERY_N_STEPS
#define SLAVE_MOTION_SNAPSHOT_EVERY_N_STEPS 5UL
#endif

// 兼容旧状态发布分频宏。
// 默认值：跟随 SLAVE_RUNTIME_PUBLISH_EVERY_N_STEPS。
// 用途：避免旧代码或 build_flags 仍使用旧宏时失效。
// 风险：两个宏同时覆盖且值不一致时，以旧宏实际值为准。
// 依赖：后续新代码优先使用 SLAVE_RUNTIME_PUBLISH_EVERY_N_STEPS。
#ifndef SLAVE_CONTROL_STATUS_PUBLISH_DIV
#define SLAVE_CONTROL_STATUS_PUBLISH_DIV SLAVE_RUNTIME_PUBLISH_EVERY_N_STEPS
#endif

// X 轴 loopFOC 分频。
// 默认值：1，真实 5kHz 验收每个 200us 控制步都运行 loopFOC。
// 用途：定位 loopFOC 负载或临时降低 FOC 调用频率。
// 风险：大于 1 会改变闭环语义，可能增大跟踪误差或声音变化。
// 依赖：只影响 X 硬件闭环路径，Y 硬件关闭时不访问 Y。
#ifndef SLAVE_X_FOC_EVERY_N_STEPS
#define SLAVE_X_FOC_EVERY_N_STEPS 1UL
#endif

// X 轴 move 分频。
// 默认值：1，保持每个 motor tick 都调用 SimpleFOC move(target_angle)。
// 用途：临时设为 2 或 5 判断 move 是否是尖峰来源。
// 风险：大于 1 会让目标写入降采样，可能增加跟随延迟；不能作为默认同步配置。
// 依赖：loopFOC 分频独立，move 跳过时 SimpleFOC 保持上一目标。
#ifndef SLAVE_X_MOVE_EVERY_N_STEPS
#define SLAVE_X_MOVE_EVERY_N_STEPS 1UL
#endif

// Y 轴 loopFOC 分频。
// 默认值：1，仅在显式启用 Y 硬件闭环时生效。
// 用途：后续 Y bring-up 时定位 Y loopFOC 开销。
// 风险：Y 轴尚未默认启用，误改不会让 Y 硬件自动运行。
// 依赖：SLAVE_Y_MOTOR_HW_ENABLED=1 且运行模式允许 Y 闭环时才会访问。
#ifndef SLAVE_Y_FOC_EVERY_N_STEPS
#define SLAVE_Y_FOC_EVERY_N_STEPS 1UL
#endif
