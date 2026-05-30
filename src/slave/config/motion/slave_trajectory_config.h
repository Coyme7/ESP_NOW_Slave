#pragma once

#include "slave/config/motion/slave_axis_config.h"
#include "slave/config/types/slave_trajectory_types.h"

static constexpr SlaveTrajectoryConfig kSlaveTrajectory = {
    160.0f,  // 落笔速度，单位 mm/s。
    160.0f,  // 抬笔速度，单位 mm/s。
    1200.0f, // 加速度，单位 mm/s^2。
    0.05f,   // 命令死区，单位 mm。
};

static constexpr SlaveAxisTrajectoryConfig kSlaveXTrajectory = {
    kSlaveXSettleErrorMmConfig, // X 到位阈值，单位 mm。
};

static constexpr SlaveAxisTrajectoryConfig kSlaveYTrajectory = {
    kSlaveYSettleErrorMmConfig, // Y 到位阈值，单位 mm。
};
