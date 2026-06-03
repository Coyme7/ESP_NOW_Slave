#pragma once

#include "slave/config/motion/slave_axis_config.h"
#include "slave/config/types/slave_trajectory_types.h"

static constexpr SlaveTrajectoryConfig kSlaveTrajectory = {
    320.0f,  // 落笔速度，单位 mm/s。
    320.0f,  // 抬笔速度，单位 mm/s。
    4800.0f, // 加速度，单位 mm/s^2。
    0.005f,  // 命令死区，单位 mm。
};

static constexpr float kSlaveCommandPredictMaxLeadS = 0.002f;
static constexpr float kSlaveCommandVelocityLpfAlpha = 0.35f;
static constexpr float kSlaveCommandPredictSpeedLimitMmS = 320.0f;

static constexpr SlaveAxisTrajectoryConfig kSlaveXTrajectory = {
    kSlaveXSettleErrorMmConfig, // X 到位阈值，单位 mm。
};

static constexpr SlaveAxisTrajectoryConfig kSlaveYTrajectory = {
    kSlaveYSettleErrorMmConfig, // Y 到位阈值，单位 mm。
};
