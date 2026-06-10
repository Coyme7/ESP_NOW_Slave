#pragma once

#include <stdint.h>

#include "slave/config/build/slave_build_options.h"

// VOFA+ FireWater 手动调参总开关。默认关闭，不创建任务，不接管串口和控制目标。
#ifndef SLAVE_VOFA_TUNER_ENABLED
#define SLAVE_VOFA_TUNER_ENABLED 1
#endif

// 轴开关默认跟随真实闭环硬件能力，也可以通过 build_flags 单独覆盖。
#ifndef SLAVE_VOFA_TUNER_X_ENABLED
#if SLAVE_VOFA_TUNER_ENABLED && SLAVE_X_MOTOR_HW_ENABLED && SLAVE_X_SENSOR_HW_ENABLED
#define SLAVE_VOFA_TUNER_X_ENABLED 1
#else
#define SLAVE_VOFA_TUNER_X_ENABLED 0
#endif
#endif

#ifndef SLAVE_VOFA_TUNER_Y_ENABLED
#if SLAVE_VOFA_TUNER_ENABLED && SLAVE_Y_MOTOR_HW_ENABLED && SLAVE_Y_SENSOR_HW_ENABLED
#define SLAVE_VOFA_TUNER_Y_ENABLED 1
#else
#define SLAVE_VOFA_TUNER_Y_ENABLED 0
#endif
#endif

static constexpr uint32_t SLAVE_VOFA_SAMPLE_PERIOD_MIN_MS = 20UL;
static constexpr uint32_t SLAVE_VOFA_SAMPLE_PERIOD_MAX_MS = 50UL;
static constexpr uint32_t SLAVE_VOFA_SAMPLE_PERIOD_DEFAULT_MS = 20UL;

static constexpr uint32_t SLAVE_VOFA_WAVE_PERIOD_MIN_MS = 2000UL;
static constexpr uint32_t SLAVE_VOFA_WAVE_PERIOD_MAX_MS = 60000UL;
static constexpr uint32_t SLAVE_VOFA_WAVE_PERIOD_DEFAULT_MS = 8000UL;

static constexpr float SLAVE_VOFA_CURRENT_LIMIT_MIN_A = 0.01f;
static constexpr float SLAVE_VOFA_CURRENT_LIMIT_MAX_A = 0.30f;
static constexpr float SLAVE_VOFA_CURRENT_LIMIT_DEFAULT_A = 0.10f;

static constexpr float SLAVE_VOFA_VOLTAGE_LIMIT_MIN_V = 0.10f;
static constexpr float SLAVE_VOFA_VOLTAGE_LIMIT_MAX_V = 2.20f;
static constexpr float SLAVE_VOFA_VOLTAGE_LIMIT_DEFAULT_V = 1.50f;

static constexpr float SLAVE_VOFA_VELOCITY_LIMIT_MIN_RAD_S = 0.05f;
static constexpr float SLAVE_VOFA_VELOCITY_LIMIT_MAX_RAD_S = 1.50f;
static constexpr float SLAVE_VOFA_VELOCITY_LIMIT_DEFAULT_RAD_S = 0.50f;

static constexpr float SLAVE_VOFA_ANGLE_LIMIT_MIN_RAD = 0.01f;
static constexpr float SLAVE_VOFA_ANGLE_LIMIT_MAX_RAD = 0.35f;
static constexpr float SLAVE_VOFA_ANGLE_LIMIT_DEFAULT_RAD = 0.10f;

static constexpr float SLAVE_VOFA_CURRENT_PID_P_MAX = 20.0f;
static constexpr float SLAVE_VOFA_CURRENT_PID_I_MAX = 500.0f;
static constexpr float SLAVE_VOFA_CURRENT_PID_D_MAX = 1.0f;
static constexpr float SLAVE_VOFA_VELOCITY_PID_P_MAX = 10.0f;
static constexpr float SLAVE_VOFA_VELOCITY_PID_I_MAX = 100.0f;
static constexpr float SLAVE_VOFA_VELOCITY_PID_D_MAX = 2.0f;
static constexpr float SLAVE_VOFA_ANGLE_PID_P_MAX = 20.0f;
static constexpr float SLAVE_VOFA_ANGLE_PID_I_MAX = 50.0f;
static constexpr float SLAVE_VOFA_ANGLE_PID_D_MAX = 5.0f;

