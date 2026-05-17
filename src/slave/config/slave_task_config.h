#pragma once

#include <Arduino.h>

#include "slave/config/slave_control_config.h"

// 从机 FreeRTOS 任务与定时器配置。
//
// 默认值保持 Core 1 运行 5kHz 控制热路径，Core 0 运行通信、安全和状态输出。

// 控制任务绑核。
// 默认值：Core 1。
// 用途：隔离 SimpleFOC、编码器读取和 motor tick。
// 风险：改到 Core 0 会和 ESP-NOW、串口、安全任务竞争。
// 依赖：通信、安全、状态任务默认在 SLAVE_IO_CORE。
static constexpr BaseType_t SLAVE_CONTROL_CORE = 1;

// IO 任务绑核。
// 默认值：Core 0。
// 用途：承载 ESP-NOW、UV 安全和串口状态输出。
// 风险：把控制外的慢任务放到 Core 1 会扰动 5kHz 闭环。
// 依赖：SLAVE_CONTROL_CORE 默认独占控制热路径。
static constexpr BaseType_t SLAVE_IO_CORE = 0;

// 控制任务优先级。
// 默认值：configMAX_PRIORITIES - 1，系统内最高。
// 用途：尽量保证 200us timer 通知后及时执行 motor tick。
// 风险：控制任务内不能阻塞或做慢操作，否则会饿死低优先级任务。
// 依赖：通信/安全/状态任务优先级必须低于控制任务。
static constexpr UBaseType_t SLAVE_CONTROL_TASK_PRIORITY = configMAX_PRIORITIES - 1;

// 通信任务优先级。
// 默认值：3，低于控制任务。
// 用途：处理 ESP-NOW pending 包和低频遥测发送。
// 风险：过高会抢占安全/状态，过低会增加命令缓存更新延迟。
// 依赖：ESP-NOW 回调只 copy-only，解析在该任务完成。
static constexpr UBaseType_t SLAVE_COMM_TASK_PRIORITY = 3;

// 安全任务优先级。
// 默认值：2，高于状态任务。
// 用途：1kHz 检查命令超时和 UV 联锁，保证异常时关灯。
// 风险：过低会延迟 UV 关闭；过高不应抢占控制热路径。
// 依赖：安全任务只读取快照，不直接读传感器。
static constexpr UBaseType_t SLAVE_SAFETY_TASK_PRIORITY = 2;

// 状态任务优先级。
// 默认值：1，最低。
// 用途：低频串口打印和人工观测。
// 风险：串口格式化很慢，不能提高到接近控制或通信任务。
// 依赖：由日志开关和 SLAVE_STATUS_LOOP_PERIOD_MS 控制输出量。
static constexpr UBaseType_t SLAVE_STATUS_TASK_PRIORITY = 1;

// 控制任务栈大小。
// 默认值：8192 byte。
// 用途：容纳 SimpleFOC 调用、控制 runtime 和少量局部诊断结构。
// 风险：加入复杂逻辑后需复查栈水位；不能靠增大栈掩盖热路径膨胀。
// 依赖：控制任务不分配动态内存。
static constexpr uint32_t SLAVE_CONTROL_TASK_STACK_BYTES = 8192;

// 通信任务栈大小。
// 默认值：4096 byte。
// 用途：处理 ESP-NOW 包校验、实时命令缓存和遥测发送。
// 风险：不要在通信任务加入大对象或复杂格式化。
// 依赖：ESP-NOW 回调不在这里的栈上执行复杂逻辑。
static constexpr uint32_t SLAVE_COMM_TASK_STACK_BYTES = 4096;

// 安全任务栈大小。
// 默认值：4096 byte。
// 用途：运行 UV 联锁和超时检查。
// 风险：安全任务不应加入串口打印或硬件扫描。
// 依赖：只调用 setUvPen() 作为 UV GPIO 出口。
static constexpr uint32_t SLAVE_SAFETY_TASK_STACK_BYTES = 4096;

// 状态任务栈大小。
// 默认值：4096 byte。
// 用途：容纳串口格式化输出。
// 风险：状态行过长会增加栈和串口耗时。
// 依赖：状态任务可通过日志开关裁剪输出类别。
static constexpr uint32_t SLAVE_STATUS_TASK_STACK_BYTES = 4096;

// FreeRTOS tick 内控制步数量。
// 默认值：1000us / 200us = 5。
// 用途：辅助确认 5kHz timer 与 1kHz tick 的关系。
// 风险：控制周期不能整除 1000us 时该值只作近似，不应用于硬实时调度。
// 依赖：SLAVE_CONTROL_LOOP_PERIOD_US。
static constexpr uint32_t SLAVE_CONTROL_STEPS_PER_TICK = 1000UL / CONTROL_LOOP_PERIOD_US;

// 控制定时器周期。
// 默认值：等于 SLAVE_CONTROL_LOOP_PERIOD_US。
// 用途：驱动 5kHz 控制任务通知。
// 风险：修改后必须重新评估 step_us 和 over_period。
// 依赖：esp_timer 使用 ISR dispatch 时不能在回调中做复杂逻辑。
static constexpr uint32_t SLAVE_CONTROL_TIMER_PERIOD_US = CONTROL_LOOP_PERIOD_US;

// 控制定时器等待超时。
// 默认值：10ms。
// 用途：控制任务长时间未收到 timer 通知时锁存故障。
// 风险：过大会延迟发现 timer 异常，过小可能误报启动抖动。
// 依赖：超时后不直接驱动电机，只锁存故障并继续等待。
static constexpr uint32_t SLAVE_CONTROL_TIMER_TIMEOUT_MS = 10UL;
