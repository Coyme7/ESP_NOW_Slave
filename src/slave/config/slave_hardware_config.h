#pragma once

// 从机硬件与运行模式配置。
//
// 本文件只放“会影响真实硬件是否初始化或输出”的编译期开关。
// 默认值面向当前 SingleX 实机同步阶段：X 轴真实闭环开启，Y/UV/自动绘图关闭。

// 启用从机 X 轴真实电机输出。
// 默认值：1，当前阶段正在验证 X 单轴真实同步，需要初始化 X 驱动、X 编码器和 SimpleFOC。
// 用途：设为 0 时只跑通信、规划、仿真和安全逻辑，不驱动真实 X 电机。
// 风险：设为 1 前必须确认 X 相线、编码器方向、供电和电压限制；错误接线可能导致抖动或发热。
// 依赖：启用时必须同时启用 SLAVE_X_SENSOR_HW_ENABLED。
#ifndef SLAVE_X_MOTOR_HW_ENABLED
#define SLAVE_X_MOTOR_HW_ENABLED 1
#endif

// 启用从机 X 轴真实 MT6701 编码器。
// 默认值：1，X 闭环必须读取真实角度。
// 用途：为 SimpleFOC X 轴 angle 模式提供机械角反馈。
// 风险：关闭后不能进入 X 闭环；错误读数会直接影响电机稳定性。
// 依赖：SLAVE_X_MOTOR_HW_ENABLED=1 时必须保持为 1。
#ifndef SLAVE_X_SENSOR_HW_ENABLED
#define SLAVE_X_SENSOR_HW_ENABLED 1
#endif

// 启用从机 Y 轴真实电机输出。
// 默认值：0，Y 轴方向、零点、限位和 PID 尚未实机确认。
// 用途：后续 YClosedLoop 或 DualXYHardware bring-up 时显式开启。
// 风险：误开会初始化并可能驱动未确认的 Y 电机。
// 依赖：启用时必须同时启用 SLAVE_Y_SENSOR_HW_ENABLED，并满足对应运行模式互斥检查。
#ifndef SLAVE_Y_MOTOR_HW_ENABLED
#define SLAVE_Y_MOTOR_HW_ENABLED 0
#endif

// 启用从机 Y 轴真实 MT6701 编码器。
// 默认值：0，SingleX 默认不访问 Y SPI、Y CS 或 Y 编码器。
// 用途：YSensorOnly 阶段可单独开启，用于低频确认 Y 编码器读数。
// 风险：如果 X/Y 共享 SPI 且没有总线仲裁，不能和 X 闭环同时读取。
// 依赖：YSensorOnly 必须与 X 电机闭环、Y 闭环和 DualXYHardware 互斥。
#ifndef SLAVE_Y_SENSOR_HW_ENABLED
#define SLAVE_Y_SENSOR_HW_ENABLED 0
#endif

// 启用真实 UV MOS 输出。
// 默认值：0，当前阶段不接紫光笔。
// 用途：后续落笔硬件接入后才允许安全任务控制 UV MOS。
// 风险：误开可能点亮 UV；任何无效命令、超时、故障或联锁失败都必须关灯。
// 依赖：不依赖绘图路径；即使开启也只能由安全任务调用 setUvPen()。
#ifndef SLAVE_UV_HW_ENABLED
#define SLAVE_UV_HW_ENABLED 0
#endif

// 从机运行模式。
// 默认值：SLAVE_MODE_SINGLE_X_SYNC，只运行已验证的 X 单轴同步链路。
// 用途：选择 SingleX、SingleY、双轴框架或双轴真实硬件路径。
// 风险：SingleY/DualXY 会改变命令映射和控制轴选择；DualXYHardware 会访问 Y 硬件。
// 依赖：DualXYHardware 必须显式开启 SLAVE_DUAL_XY_HARDWARE_ENABLED 和 X/Y 全部硬件。
#ifndef SLAVE_RUN_MODE
#define SLAVE_RUN_MODE SLAVE_MODE_SINGLE_X_SYNC
#endif

// 双轴真实硬件总保险。
// 默认值：0，禁止默认进入真实双轴硬件控制。
// 用途：只有确认 Y 轴方向、零点、限位、SPI 边界和闭环稳定后才设为 1。
// 风险：误开会让 DualXYHardware 访问并驱动 X/Y 两轴。
// 依赖：SLAVE_RUN_MODE=SLAVE_MODE_DUAL_XY_HW 时必须为 1，并且 X/Y 电机和传感器都启用。
#ifndef SLAVE_DUAL_XY_HARDWARE_ENABLED
#define SLAVE_DUAL_XY_HARDWARE_ENABLED 0
#endif

// 启用 MT6701 快速读取器。
// 默认值：1，控制热路径必须走 fast reader。
// 用途：避免 digitalWrite、SPI begin/endTransaction 或多次 transfer 慢路径进入 5kHz 闭环。
// 风险：设为 0 会破坏实时性，当前从机禁止关闭。
// 依赖：common/sensors 只读取本宏传入的公共诊断开关，不反向依赖 slave 目录。
#ifndef SLAVE_FAST_SENSOR_READER_ENABLED
#define SLAVE_FAST_SENSOR_READER_ENABLED 1
#endif

// 自动绘图功能开关。
// 默认值：0，当前阶段只做主从手动同步，不做自动绘图。
// 用途：后续自动绘图阶段由专用 planner/trajectory 模块启用。
// 风险：误开会绕过本轮 SingleX 性能验证边界。
// 依赖：只能在 DualXYHardware 模式下启用。
#ifndef SLAVE_AUTO_DRAW_ENABLED
#define SLAVE_AUTO_DRAW_ENABLED 0
#endif

// Y 轴软件仿真开关。
// 默认值：1，只用于双轴框架状态显示和 bring-up 准备，不访问 Y 硬件。
// 用途：Y 硬件关闭时提供默认显示/仿真值。
// 风险：仿真值不能代表真实 Y 轴状态，不能作为闭环安全依据。
// 依赖：真实 Y 闭环或 DualXYHardware 阶段应改为控制任务发布的真实快照。
#ifndef SLAVE_Y_SIMULATION_ENABLED
#define SLAVE_Y_SIMULATION_ENABLED 1
#endif

// Y 轴 bring-up 模式。
// 默认值：SLAVE_Y_BRINGUP_DISABLED，SingleX 默认不读 Y 传感器、不驱动 Y 电机。
// 用途：按 SensorOnly、MotorOpenLoop、ClosedLoop 分阶段验证 Y 硬件。
// 风险：SensorOnly 可能与 X 闭环共享 SPI；ClosedLoop 会驱动真实 Y 电机。
// 依赖：各模式需要对应的 Y 传感器/电机宏，并受 static_assert 互斥保护。
#ifndef SLAVE_Y_BRINGUP_MODE
#define SLAVE_Y_BRINGUP_MODE SLAVE_Y_BRINGUP_DISABLED
#endif

// DengFoc/BLDCDriver3PWM 使能脚电平配置。
// 默认值：1，按高电平有效处理。
// 用途：适配不同驱动板 EN 脚极性。
// 风险：极性设错会导致电机无法使能或上电默认输出风险。
// 依赖：改动后必须先调用 configureSlaveSafeOutputs() 验证禁用态。
#ifndef SLAVE_DRIVER_ENABLE_ACTIVE_HIGH
#define SLAVE_DRIVER_ENABLE_ACTIVE_HIGH 1
#endif
