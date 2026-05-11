#pragma once

// 从机板级引脚定义。
// 当前临时联调阶段，从机 X 轴也接 MT6701 编码器 + 2804 BLDC 电机做闭环测试；
// 后续正式从机云台仍回到 2208 电机，切换时需要同步极对数、方向和限幅参数。
namespace board_pins_slave {

// 紫光灯 MOS 输出。安全默认必须关闭，只有 UV 联锁满足时才允许打开。
static constexpr int UV_MOS_1 = 4;
static constexpr int UV_MOS_2 = 5;

// X/Y 电机驱动使能脚；启动、超时、故障和急停时都应保持关闭。
static constexpr int MOTOR1_EN_X = 6;
static constexpr int MOTOR2_EN_Y = 7;

// X 轴三相 PWM 输出。当前单轴测试优先使用 X 轴通道。
static constexpr int MOTOR1_PWM_U_X = 15;
static constexpr int MOTOR1_PWM_V_X = 16;
static constexpr int MOTOR1_PWM_W_X = 17;

// Y 轴三相 PWM 输出，保留给后续双轴绘图测试。
static constexpr int MOTOR2_PWM_U_Y = 9;
static constexpr int MOTOR2_PWM_V_Y = 10;
static constexpr int MOTOR2_PWM_W_Y = 11;

// X 轴 MT6701 编码器接线。
static constexpr int ENCODER1_CS_X = 41;
static constexpr int ENCODER1_DO_X = 40;
static constexpr int ENCODER1_CLK_X = 39;

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
static constexpr int UNUSED_WEAK_47 = 47;
static constexpr int UNUSED_WEAK_21 = 21;
static constexpr int UNUSED_JTAG_42 = 42;

// 当前保留不用的普通 GPIO。
static constexpr int UNUSED_STRONG_18 = 18;
static constexpr int UNUSED_STRONG_8 = 8;
static constexpr int UNUSED_12 = 12;
static constexpr int UNUSED_13 = 13;
static constexpr int UNUSED_14 = 14;

// ESP32-S3 启动绑带相关脚，除非重新审查启动电平，否则不要用于新功能。
static constexpr int DO_NOT_USE_STRAP_1 = 3;
static constexpr int DO_NOT_USE_STRAP_2 = 46;
static constexpr int DO_NOT_USE_STRAP_3 = 45;

}  // namespace board_pins_slave
