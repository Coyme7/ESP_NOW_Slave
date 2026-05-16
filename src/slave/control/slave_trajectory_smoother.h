#pragma once

// 从机 X 单轴纸面轨迹平滑器。
// 输入/输出都使用 mm，调用方再通过 coordinate_mapper 转成云台角度。
struct SlaveTrajectorySmootherState {
    float x_mm;
    float velocity_mm_s;
    bool initialized;
};

struct SlaveTrajectorySmootherInput {
    float target_x_mm;
    float dt_s;
    float max_speed_mm_s;
    float accel_mm_s2;
    float deadband_mm;
};

struct SlaveTrajectorySmootherOutput {
    float x_mm;
    float velocity_mm_s;
};

SlaveTrajectorySmootherOutput updateSlaveTrajectorySmoother(SlaveTrajectorySmootherState &state,
                                                            const SlaveTrajectorySmootherInput &input);
