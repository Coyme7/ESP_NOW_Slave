#pragma once

// 从机板级引脚定义。
// 当前 PCB 是信号转接板，从机 IO 分布与主机信号板复用同一套排针。
// 修改 GPIO 前先同步 docs/pinmap_slave.md，并确认没有占用启动绑带脚或保留脚。
namespace board_pins_slave {

// 紫光灯 MOS 输出。安全默认必须关闭，只有 UV 联锁满足时才允许打开。
// 当前 MOS 模块按高电平导通处理；如果实际模块为低电平导通，只改这里的极性。
static constexpr int UV_MOS = 17;
static constexpr bool UV_MOS_ACTIVE_HIGH = true;
static constexpr int UV_MOS_ACTIVE_LEVEL = UV_MOS_ACTIVE_HIGH ? 1 : 0;
static constexpr int UV_MOS_INACTIVE_LEVEL = UV_MOS_ACTIVE_HIGH ? 0 : 1;

// DengFoc 驱动共用使能脚；不能单独关闭某一电机。
// 启动、超时、故障和急停时都应保持关闭。
static constexpr int MOTOR_DRIVER_EN = 6;
static constexpr int MOTOR1_EN_X = MOTOR_DRIVER_EN;
static constexpr int MOTOR2_EN_Y = MOTOR_DRIVER_EN;

// X 轴三相 PWM 输出。当前单轴测试优先使用 X 轴通道。
static constexpr int MOTOR1_PWM_U_X = 7;
static constexpr int MOTOR1_PWM_V_X = 15;
static constexpr int MOTOR1_PWM_W_X = 16;

// Y 轴三相 PWM 输出，保留给后续双轴绘图测试。
static constexpr int MOTOR2_PWM_U_Y = 9;
static constexpr int MOTOR2_PWM_V_Y = 10;
static constexpr int MOTOR2_PWM_W_Y = 11;

// X 轴 MT6701 编码器接线。
static constexpr int ENCODER1_CS_X = 12;
static constexpr int ENCODER1_DO_X = 13;
static constexpr int ENCODER1_CLK_X = 14;

// Y 轴 MT6701 编码器接线，当前单轴测试可暂不连接。
static constexpr int ENCODER2_CS_Y = 38;
static constexpr int ENCODER2_DO_Y = 37;
static constexpr int ENCODER2_CLK_Y = 36;

// 校准按钮和状态灯。
static constexpr int LIMIT_CALIB_BUTTON = 0;
static constexpr int STATUS_RGB = 48;

// 串口和 USB 固定功能脚，保留给日志、烧录和调试。
static constexpr int UART_TX = 43;
static constexpr int UART_RX = 44;
static constexpr int USB_D_PLUS = 20;
static constexpr int USB_D_MINUS = 19;

// 弱驱动或调试相关未用脚；启用前先核对电气能力和 pinmap。
static constexpr int UNUSED_WEAK_1 = 1;
static constexpr int UNUSED_WEAK_2 = 2;
static constexpr int UNUSED_WEAK_35 = 35;
static constexpr int UNUSED_WEAK_39 = 39;
static constexpr int UNUSED_WEAK_40 = 40;
static constexpr int UNUSED_WEAK_41 = 41;
static constexpr int UNUSED_WEAK_47 = 47;
static constexpr int UNUSED_WEAK_21 = 21;
static constexpr int UNUSED_JTAG_42 = 42;

// 当前保留不用的普通 GPIO。
static constexpr int UNUSED_STRONG_4 = 4;
static constexpr int UNUSED_STRONG_5 = 5;
static constexpr int UNUSED_STRONG_18 = 18;
static constexpr int UNUSED_STRONG_8 = 8;

// ESP32-S3 启动绑带相关脚，除非重新审查启动电平，否则不要用于新功能。
static constexpr int DO_NOT_USE_STRAP_1 = 3;
static constexpr int DO_NOT_USE_STRAP_2 = 46;
static constexpr int DO_NOT_USE_STRAP_3 = 45;

}  // namespace board_pins_slave
