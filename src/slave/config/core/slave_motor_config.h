#pragma once

#include "slave/config/types/slave_motor_types.h"

// == 电机硬件参数 ===============================================================

// DengFoc/BLDCDriver3PWM EN 引脚极性：0=EN 低有效，1=EN 高有效。
#ifndef SLAVE_DRIVER_ENABLE_ACTIVE_HIGH
#define SLAVE_DRIVER_ENABLE_ACTIVE_HIGH 1
#endif

// X 轴 2208 云台电机极对数。
#ifndef SLAVE_X_MOTOR_POLE_PAIRS
#define SLAVE_X_MOTOR_POLE_PAIRS 7
#endif

// Y 轴 2208 云台电机极对数。
#ifndef SLAVE_Y_MOTOR_POLE_PAIRS
#define SLAVE_Y_MOTOR_POLE_PAIRS 7
#endif

static constexpr float kSlaveOpenLoopVoltageLimitV = 0.4f;

static constexpr SlaveMotorFocConfig kSlaveXMotorFoc = {
    {
        12.0f,                       // 驱动供电电压，单位 V。
        5.0f,                        // 对齐阶段驱动输出电压上限，单位 V。
        2.2f,                        // 闭环电机电压上限，单位 V。
        kSlaveOpenLoopVoltageLimitV, // 开环电机电压上限，单位 V。
        5.0f,                        // FOC 对齐电压，单位 V。
    },
    {
        3.0f,  // 闭环速度上限，单位 rad/s。
        0.50f, // 电流环目标限幅，单位 A。
    },
    {
        0.21f, // 位置环 P 增益。
        0.0f, // 位置环 I 增益。
        0.0f, // 位置环 D 增益。
    },
    {
        0.25f,  // 速度环 P 增益。
        0.0f,   // 速度环 I 增益。
        0.0f,   // 速度环 D 增益。
        500.0f, // 速度环输出变化斜率限制。
    },
    {
        0.01f,  // 速度反馈低通时间常数，单位 s。
        0.001f, // 角度反馈低通时间常数，单位 s。
    },
    {
        {
            0.0f,  // q 轴电流环 P 增益。
            0.0f,   // q 轴电流环 I 增益。
            0.0f,   // q 轴电流环 D 增益。
            500.0f, // q 轴电流环输出变化斜率限制。
        },
        {
            0.0f,  // d 轴电流环 P 增益。
            0.0f,   // d 轴电流环 I 增益。
            0.0f,   // d 轴电流环 D 增益。
            500.0f, // d 轴电流环输出变化斜率限制。
        },
        0.001f, // q/d 电流低通时间常数，单位 s。
    },
};

static constexpr SlaveMotorFocConfig kSlaveYMotorFoc = {
    {
        12.0f,                       // 驱动供电电压，单位 V。
        5.0f,                        // 对齐阶段驱动输出电压上限，单位 V。
        2.2f,                        // 闭环电机电压上限，单位 V。
        kSlaveOpenLoopVoltageLimitV, // 开环电机电压上限，单位 V。
        5.0f,                        // FOC 对齐电压，单位 V。
    },
    {
        3.0f,  // 闭环速度上限，单位 rad/s。
        0.50f, // 电流环目标限幅，单位 A。
    },
    {
        0.21f, // 位置环 P 增益。
        0.0f, // 位置环 I 增益。
        0.0f, // 位置环 D 增益。
    },
    {
        0.05f,  // 速度环 P 增益。
        0.0f,   // 速度环 I 增益。
        0.0f,   // 速度环 D 增益。
        500.0f, // 速度环输出变化斜率限制。
    },
    {
        0.01f,  // 速度反馈低通时间常数，单位 s。
        0.001f, // 角度反馈低通时间常数，单位 s。
    },
    {
        {
            1.40f,  // q 轴电流环 P 增益。
            0.0f,   // q 轴电流环 I 增益。
            0.0f,   // q 轴电流环 D 增益。
            500.0f, // q 轴电流环输出变化斜率限制。
        },
        {
            1.40f,  // d 轴电流环 P 增益。
            0.0f,   // d 轴电流环 I 增益。
            0.0f,   // d 轴电流环 D 增益。
            500.0f, // d 轴电流环输出变化斜率限制。
        },
        0.001f, // q/d 电流低通时间常数，单位 s。
    },
};
