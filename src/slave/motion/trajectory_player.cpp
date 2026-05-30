#include "slave/motion/trajectory_player.h"

#include <math.h>

namespace {

float segmentLengthMm(const DrawSegment &segment) {
    const float dx = segment.end.x_mm - segment.start.x_mm;
    const float dy = segment.end.y_mm - segment.start.y_mm;
    return sqrtf(dx * dx + dy * dy);
}

uint8_t progressPercent(const TrajectoryPlayerState &state) {
    if (state.finished) {
        return 100;
    }
    if (state.total_distance_mm <= 0.0001f) {
        return state.active ? 0 : 100;
    }
    const float progress =
        (state.completed_distance_mm + state.distance_on_segment_mm) / state.total_distance_mm;
    const int pct = static_cast<int>(progress * 100.0f + 0.5f);
    if (pct <= 0) {
        return 0;
    }
    if (pct >= 100) {
        return 100;
    }
    return static_cast<uint8_t>(pct);
}

float totalLengthMm(const DrawSegment *segments, size_t segment_count) {
    if (segments == nullptr) {
        return 0.0f;
    }
    float total = 0.0f;
    for (size_t i = 0; i < segment_count; ++i) {
        total += segmentLengthMm(segments[i]);
    }
    return total;
}

TrajectoryPlayerOutput holdSegmentEnd(const TrajectoryPlayerState &state,
                                      const DrawSegment &segment,
                                      bool active,
                                      bool finished) {
    TrajectoryPlayerOutput output = {};
    output.x_mm = segment.end.x_mm;
    output.y_mm = segment.end.y_mm;
    output.pen_req = segment.end.pen_down;
    output.active = active;
    output.finished = finished;
    output.progress_pct = progressPercent(state);
    output.segment_index = state.segment_index;
    output.segment_count = state.segment_count;
    return output;
}

}  // namespace

void startTrajectoryPlayer(TrajectoryPlayerState &state,
                           const DrawSegment *segments,
                           size_t segment_count) {
    state.segments = segments;
    state.segment_count = segment_count;
    state.segment_index = 0;
    state.distance_on_segment_mm = 0.0f;
    state.completed_distance_mm = 0.0f;
    state.total_distance_mm = totalLengthMm(segments, segment_count);
    state.active = segments != nullptr && segment_count > 0;
    state.finished = !state.active;
}

void stopTrajectoryPlayer(TrajectoryPlayerState &state) {
    state.active = false;
    state.segment_index = 0;
    state.distance_on_segment_mm = 0.0f;
    state.completed_distance_mm = 0.0f;
    state.finished = false;
}

TrajectoryPlayerOutput updateTrajectoryPlayer(TrajectoryPlayerState &state, float dt_s) {
    TrajectoryPlayerOutput output = {};
    if (!state.active || state.segments == nullptr || state.segment_count == 0) {
        output.finished = state.finished;
        output.progress_pct = progressPercent(state);
        output.segment_index = state.segment_index;
        output.segment_count = state.segment_count;
        return output;
    }

    float advance_mm = 0.0f;
    if (dt_s > 0.0f) {
        const DrawSegment &segment = state.segments[state.segment_index];
        const float feed_mm_s = (segment.feed_mm_s > 0.0f) ? segment.feed_mm_s : 1.0f;
        advance_mm = feed_mm_s * dt_s;
    }

    while (state.segment_index < state.segment_count) {
        const DrawSegment &segment = state.segments[state.segment_index];
        const float length_mm = segmentLengthMm(segment);
        state.distance_on_segment_mm += advance_mm;
        advance_mm = 0.0f;

        if (length_mm <= 0.0001f || state.distance_on_segment_mm >= length_mm) {
            const float overflow_mm =
                (length_mm > 0.0001f) ? (state.distance_on_segment_mm - length_mm) : 0.0f;
            state.completed_distance_mm += length_mm;
            state.segment_index++;
            state.distance_on_segment_mm = (overflow_mm > 0.0f) ? overflow_mm : 0.0f;
            if (state.segment_index >= state.segment_count) {
                state.active = false;
                state.finished = true;
                state.distance_on_segment_mm = 0.0f;
                state.completed_distance_mm = state.total_distance_mm;
                return holdSegmentEnd(state, segment, false, true);
            }
            continue;
        }

        const float t = state.distance_on_segment_mm / length_mm;
        output.x_mm = segment.start.x_mm + (segment.end.x_mm - segment.start.x_mm) * t;
        output.y_mm = segment.start.y_mm + (segment.end.y_mm - segment.start.y_mm) * t;
        output.pen_req = segment.start.pen_down || segment.end.pen_down;
        output.active = true;
        output.finished = false;
        output.progress_pct = progressPercent(state);
        output.segment_index = state.segment_index;
        output.segment_count = state.segment_count;
        return output;
    }

    state.active = false;
    state.finished = true;
    output.finished = true;
    output.progress_pct = 100;
    output.segment_index = state.segment_index;
    output.segment_count = state.segment_count;
    return output;
}
