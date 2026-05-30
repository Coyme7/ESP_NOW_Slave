#pragma once

#include <stddef.h>
#include <stdint.h>

#include "slave/motion/draw_primitives.h"

struct TrajectoryPlayerState {
    const DrawSegment *segments;
    size_t segment_count;
    size_t segment_index;
    float distance_on_segment_mm;
    float completed_distance_mm;
    float total_distance_mm;
    bool active;
    bool finished;
};

struct TrajectoryPlayerOutput {
    float x_mm;
    float y_mm;
    bool pen_req;
    bool active;
    bool finished;
    uint8_t progress_pct;
    size_t segment_index;
    size_t segment_count;
};

void startTrajectoryPlayer(TrajectoryPlayerState &state,
                           const DrawSegment *segments,
                           size_t segment_count);

void stopTrajectoryPlayer(TrajectoryPlayerState &state);

TrajectoryPlayerOutput updateTrajectoryPlayer(TrajectoryPlayerState &state, float dt_s);
