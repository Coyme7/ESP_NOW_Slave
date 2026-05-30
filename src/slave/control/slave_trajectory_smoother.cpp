#include "slave/control/slave_trajectory_smoother.h"

#include <math.h>

#include "common/math/clamp.h"
#include "common/protocol/protocol_units.h"

namespace {

float signOf(float value) {
    if (value > 0.0f) {
        return 1.0f;
    }
    if (value < 0.0f) {
        return -1.0f;
    }
    return 0.0f;
}

float approachFloat(float current, float target, float max_delta) {
    const float delta = target - current;
    if (fabsf(delta) <= max_delta) {
        return target;
    }
    return current + signOf(delta) * max_delta;
}

}  // namespace

SlaveTrajectorySmootherOutput updateSlaveTrajectorySmoother(SlaveTrajectorySmootherState &state,
                                                            const SlaveTrajectorySmootherInput &input) {
    const float min_mm = fminf(input.min_mm, input.max_mm);
    const float max_mm = fmaxf(input.min_mm, input.max_mm);
    const float target_mm = clampFloat(input.target_mm, min_mm, max_mm);
    if (!state.initialized) {
        state.position_mm = 0.0f;
        state.velocity_mm_s = 0.0f;
        state.initialized = true;
    }

    const float dt_s = clampFloat(input.dt_s, 0.0f, 0.02f);
    const float max_speed_mm_s = fmaxf(input.max_speed_mm_s, 0.0f);
    const float accel_mm_s2 = fmaxf(input.accel_mm_s2, 1.0f);
    const float deadband_mm = fmaxf(input.deadband_mm, 0.0f);
    if (dt_s <= 0.0f || max_speed_mm_s <= 0.0f) {
        return {state.position_mm, state.velocity_mm_s};
    }

    const float error_mm = target_mm - state.position_mm;
    if (fabsf(error_mm) <= deadband_mm &&
        fabsf(state.velocity_mm_s) <= accel_mm_s2 * dt_s) {
        state.position_mm = target_mm;
        state.velocity_mm_s = 0.0f;
        return {state.position_mm, state.velocity_mm_s};
    }

    const float stop_distance_mm =
        (state.velocity_mm_s * state.velocity_mm_s) / (2.0f * accel_mm_s2);
    float desired_velocity_mm_s = signOf(error_mm) * max_speed_mm_s;
    if (stop_distance_mm + deadband_mm >= fabsf(error_mm)) {
        desired_velocity_mm_s = 0.0f;
    }

    state.velocity_mm_s =
        approachFloat(state.velocity_mm_s, desired_velocity_mm_s, accel_mm_s2 * dt_s);
    const float step_mm = state.velocity_mm_s * dt_s;
    if (fabsf(step_mm) >= fabsf(error_mm)) {
        state.position_mm = target_mm;
        state.velocity_mm_s = 0.0f;
    } else {
        state.position_mm += step_mm;
    }

    state.position_mm = clampFloat(state.position_mm, min_mm, max_mm);
    return {state.position_mm, state.velocity_mm_s};
}
