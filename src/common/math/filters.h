#pragma once

#include "common/math/clamp.h"

inline float lowPassFilter(float input, float previous, float dt_s, float tf_s) {
    if (tf_s <= 0.0f) {
        return input;
    }
    if (dt_s <= 0.0f) {
        return previous;
    }

    const float alpha = tf_s / (tf_s + dt_s);
    return alpha * previous + (1.0f - alpha) * input;
}

