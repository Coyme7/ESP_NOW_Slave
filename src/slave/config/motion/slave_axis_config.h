#pragma once

#include "common/motion/paper_mapper.h"
#include "slave/config/types/slave_axis_types.h"

static constexpr float kSlaveXAxisLimitMinMmConfig = -PLOT_X_HALF_RANGE_MM;
static constexpr float kSlaveXAxisLimitMaxMmConfig = PLOT_X_HALF_RANGE_MM;
static constexpr float kSlaveYAxisLimitMinMmConfig = -PLOT_Y_HALF_RANGE_MM;
static constexpr float kSlaveYAxisLimitMaxMmConfig = PLOT_Y_HALF_RANGE_MM;
static constexpr float kSlaveXSettleErrorMmConfig = 0.5f;
static constexpr float kSlaveYSettleErrorMmConfig = 0.5f;

static constexpr SlaveAxisConfig kSlaveXAxis = {
    {
        0.0f,                      // X 纸面中心云台角，单位 rad。
        1,                         // X 机械方向符号。
        DEFAULT_THROW_DISTANCE_MM, // X 投影距离，单位 mm。
        PLOT_X_HALF_RANGE_MM,      // X 纸面半幅，单位 mm。
    },
    {
        0.0087f, // X 到位阈值，单位 rad。
        0.0025f, // X 仿真跟随系数。
    },
};

static constexpr SlaveAxisConfig kSlaveYAxis = {
    {
        0.0f,                      // Y 纸面中心云台角，单位 rad。
        1,                         // Y 机械方向符号。
        DEFAULT_THROW_DISTANCE_MM, // Y 投影距离，单位 mm。
        PLOT_Y_HALF_RANGE_MM,      // Y 纸面半幅，单位 mm。
    },
    {
        0.0087f, // Y 到位阈值，单位 rad。
        0.0025f, // Y 仿真跟随系数。
    },
};

static constexpr SlaveAxisLimitConfig kSlaveXAxisLimit = {
    kSlaveXAxisLimitMinMmConfig, // X 低端软限位，单位 mm。
    kSlaveXAxisLimitMaxMmConfig, // X 高端软限位，单位 mm。
};

static constexpr SlaveAxisLimitConfig kSlaveYAxisLimit = {
    kSlaveYAxisLimitMinMmConfig, // Y 低端软限位，单位 mm。
    kSlaveYAxisLimitMaxMmConfig, // Y 高端软限位，单位 mm。
};
