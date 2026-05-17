#include "slave/config/slave_config.h"

// 从机 X 轴默认调参值。
// 当前 A4 中心对应 0 rad，direction=1 表示正向 x_norm 让光斑向正 X 方向移动。
// 投影距离默认 300 mm，settle_error_rad 约 0.5 deg，用于判断目标是否足够稳定再开 UV。
// simulated_response_alpha 是仿真跟随系数，硬件关闭时让 actual_angle 缓慢靠近 target。
const SlaveXAxisConfig kSlaveXAxis = {
    0.0f,
    1.0f,
    DEFAULT_THROW_DISTANCE_MM,
    PLOT_X_HALF_RANGE_MM,
    0.0087f,
    0.0025f,
};

// 从机 Y 轴默认只用于软件框架和 bring-up 准备。硬件默认关闭，方向、零点和 PID 待实机确认。
const SlaveAxisConfig kSlaveYAxis = {
    0.0f,
    1.0f,
    DEFAULT_THROW_DISTANCE_MM,
    125.0f,
    0.0087f,
    0.0025f,
};

// 从机 X 轴位置闭环默认值。
// P_angle 先低于 SimpleFOC 默认值，速度环 I 也压低，避免首次闭环抖动和过冲；
// 若实际光斑响应太慢，先确认方向/零点正确，再逐步提高 angle_p 和 velocity_limit。
const SlaveMotorFocConfig kSlaveMotorFoc = {
    12.0f,
    2.0f,
    1.5f,
    0.6f,
    3.0f,
    8.0f,
    0.18f,
    2.0f,
    0.0f,
    100.0f,
    0.01f,
    0.0f,
};

const SlaveTrajectoryConfig kSlaveTrajectory = {
    160.0f,
    160.0f,
    1200.0f,
    0.05f,
    0.5f,
};

const SlaveAxisLimitConfig kSlaveAxisLimit = {
    -125.0f,
    125.0f,
};

// 当前单轴联调先使用固定主机 MAC；后续加入自动配对后再替换。
const uint8_t kSlavePeerMasterAddress[6] = {0x24, 0x58, 0x7c, 0xd0, 0xb3, 0x64};
