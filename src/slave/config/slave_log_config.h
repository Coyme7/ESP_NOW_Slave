#pragma once

#include <stdint.h>

// slave_log_config
// 职责：集中管理从机启动日志、状态输出和 timing 诊断。
// 注意：日志只允许在启动路径和 Core 0 状态任务中输出，禁止进入控制热路径。
//
// 配置分层原则：
//   SLAVE_TIMING_DIAG_LEVEL 控制“采不采 timing、采多细”，会影响 5kHz 热路径开销。
//   SLAVE_STATUS_*_LOG_ENABLED 控制“打印哪些已有状态行”，主要影响 Core 0 串口输出。

// 启动配置日志开关。
// 默认值：1，上电时打印模式、硬件开关、信道和关键控制分频。
// 用途：现场确认烧录对象、默认配置和无线信道是否正确。
// 风险：只在启动时输出；关闭后排查配置漂移会更困难。
// 依赖：不影响运行期状态任务。
#ifndef SLAVE_BOOT_LOG_ENABLED
#define SLAVE_BOOT_LOG_ENABLED 1
#endif

// 低频状态任务总开关。
// 默认值：1，创建 SlaveStatus 任务。
// 用途：周期打印摘要、XY 和 timing 状态。
// 风险：串口输出会占用 Core 0 时间；性能压测时可关闭。
// 依赖：关闭后下面的细分日志开关都不会输出。
#ifndef SLAVE_STATUS_LOG_ENABLED
#define SLAVE_STATUS_LOG_ENABLED 1
#endif

// 从机控制定时器启动日志开关。
// 默认值：跟随 SLAVE_BOOT_LOG_ENABLED，成功启动时打印控制周期和 dispatch 模式。
// 用途：确认 200us 控制定时器是否按预期启用。
// 风险：只控制成功启动提示；创建失败和启动失败仍会打印错误。
// 依赖：不影响控制任务创建，也不进入 5kHz 控制循环。
#ifndef SLAVE_CONTROL_TIMER_LOG_ENABLED
#define SLAVE_CONTROL_TIMER_LOG_ENABLED SLAVE_BOOT_LOG_ENABLED
#endif

// 状态任务周期。
// 默认值：500ms，降低串口输出对无线和控制观测的反向干扰。
// 用途：控制运行期串口输出频率。
// 风险：值过小会增加 Core 0 压力，值过大不利于人工观察。
// 依赖：仅在 SLAVE_STATUS_LOG_ENABLED=1 时生效。
#ifndef SLAVE_STATUS_LOOP_PERIOD_MS
#define SLAVE_STATUS_LOOP_PERIOD_MS 500UL
#endif

static_assert(SLAVE_STATUS_LOOP_PERIOD_MS > 0,
              "SLAVE_STATUS_LOOP_PERIOD_MS must be greater than 0");

// 从机 timing 诊断等级。
// level 0：关闭 timing 采样和统计，用于观察最真实的 5kHz 控制负载。
// level 1：只保留整步统计：step_us / step_max / over_period / over_75pct / over_50pct。
// level 2：完整分段诊断：command / trajectory / motor / sensor / loopFOC / move / state / publish。
// 默认值：0，默认压测不采样；如果旧 build_flags 显式设置 SLAVE_TIMING_DIAG_ENABLED=1 且未设置本宏，则兼容映射为 level 2。
// 用途：区分 timer、sensor、loopFOC、move 和完整工程路径开销。
// 风险：level 1/2 会增加 micros() 采样和统计写入；正式性能验收应同时测试 level 0。
// 依赖：x_sensor_us / x_foc_us / x_move_us 只有 level 2 才真实更新。
#ifndef SLAVE_TIMING_DIAG_LEVEL
#ifdef SLAVE_TIMING_DIAG_ENABLED
#if SLAVE_TIMING_DIAG_ENABLED
#define SLAVE_TIMING_DIAG_LEVEL 2
#else
#define SLAVE_TIMING_DIAG_LEVEL 0
#endif
#else
#define SLAVE_TIMING_DIAG_LEVEL 0
#endif
#endif

static_assert(SLAVE_TIMING_DIAG_LEVEL >= 0 && SLAVE_TIMING_DIAG_LEVEL <= 2,
              "SLAVE_TIMING_DIAG_LEVEL must be 0, 1, or 2");

// 从机 timing 整步诊断派生开关。
// 默认值：由 SLAVE_TIMING_DIAG_LEVEL>=1 自动得出，默认 0。
// 用途：控制 control step 总耗时、miss 和 over 阈值统计是否编译进入热路径。
// 风险：开启后每个控制步会增加整步 micros() 采样和统计写入。
// 依赖：只通过 SLAVE_TIMING_DIAG_LEVEL 配置，不建议在 build_flags 中直接覆盖。
#define SLAVE_TIMING_STEP_DIAG_ENABLED (SLAVE_TIMING_DIAG_LEVEL >= 1)

// 从机 timing 分段诊断派生开关。
// 默认值：由 SLAVE_TIMING_DIAG_LEVEL>=2 自动得出，默认 0。
// 用途：控制 command / planner / motor / sensor / loopFOC / move 等分段统计是否编译进入热路径。
// 风险：开启后开销明显高于 level 1，只适合短时间定位瓶颈。
// 依赖：x_sensor_us / x_foc_us / x_move_us 等字段依赖该开关真实更新。
#define SLAVE_TIMING_DETAIL_DIAG_ENABLED (SLAVE_TIMING_DIAG_LEVEL >= 2)

// 从机 timing 旧兼容派生开关。
// 默认值：由 SLAVE_TIMING_DIAG_LEVEL>0 自动得出，默认 0。
// 用途：兼容旧代码中的 SLAVE_TIMING_DIAG_ENABLED 条件编译。
// 风险：新配置不应直接改该宏，否则容易和 level 语义不一致。
// 依赖：新代码统一使用 SLAVE_TIMING_DIAG_LEVEL / SLAVE_TIMING_STEP_DIAG_ENABLED / SLAVE_TIMING_DETAIL_DIAG_ENABLED。
#undef SLAVE_TIMING_DIAG_ENABLED
#define SLAVE_TIMING_DIAG_ENABLED SLAVE_TIMING_STEP_DIAG_ENABLED

// 从机摘要状态行开关。
// 默认值：1，输出模式、收包计数、fault、UV 和命令年龄。
// 用途：低成本观察链路和安全状态。
// 风险：摘要行仍有 Serial.printf 开销，只允许在状态任务中使用。
// 依赖：受 SLAVE_STATUS_LOG_ENABLED 总开关控制。
#ifndef SLAVE_STATUS_SUMMARY_LOG_ENABLED
#define SLAVE_STATUS_SUMMARY_LOG_ENABLED 1
#endif

// 从机 XY 状态行开关。
// 默认值：1，输出命令、目标、实际角、误差和限位。
// 用途：SingleX 同步验收和后续 Y 框架观察。
// 风险：包含浮点格式化，输出开销明显高于摘要行。
// 依赖：状态任务读取 SlaveMotionSnapshot，不直接读控制热路径或传感器。
#ifndef SLAVE_STATUS_XY_LOG_ENABLED
#define SLAVE_STATUS_XY_LOG_ENABLED 1
#endif

// 从机 timing 状态行开关。
// 默认值：跟随 SLAVE_TIMING_STEP_DIAG_ENABLED，level 0 时默认不打印 timing 行。
// 用途：对比 5kHz/2kHz 和定位 sensor/loopFOC/move 尖峰。
// 风险：状态行长，串口输出本身会增加 Core 0 负载。
// 依赖：设为 1 只打开串口输出；是否采样由 SLAVE_TIMING_DIAG_LEVEL 决定。
#ifndef SLAVE_STATUS_TIMING_LOG_ENABLED
#define SLAVE_STATUS_TIMING_LOG_ENABLED SLAVE_TIMING_STEP_DIAG_ENABLED
#endif

// YSensorOnly 状态任务读 Y 编码器开关。
// 默认值：1，但只有 SLAVE_Y_BRINGUP_MODE=SLAVE_Y_BRINGUP_SENSOR_ONLY 且 Y 传感器启用时才会真正读硬件。
// 用途：Y 传感器专用 bring-up 低频观测。
// 风险：如果 X/Y 共享 SPI，不能与 X 闭环同时使用；互斥由 build options 静态检查保护。
// 依赖：SLAVE_Y_SENSOR_HW_ENABLED=1 且 SLAVE_Y_BRINGUP_MODE=SLAVE_Y_BRINGUP_SENSOR_ONLY。
#ifndef SLAVE_STATUS_Y_SENSOR_BRINGUP_LOG_ENABLED
#define SLAVE_STATUS_Y_SENSOR_BRINGUP_LOG_ENABLED 1
#endif
