#pragma once

#include <stdint.h>

#include "slave/control/slave_runtime_snapshot.h"

struct SlaveLocalControlRuntime {
    float target_x_mm;
    float smooth_x_mm;
    float target_angle_rad;
    float actual_angle_rad;
    float target_y_mm;
    float smooth_y_mm;
    float target_y_angle_rad;
    float actual_y_angle_rad;
    bool x_limit;
    bool y_limit;
    bool x_clamped;
    bool y_clamped;
    bool boundary_hit;
    uint8_t command_valid;
    uint8_t pen_req;
    uint8_t draw_state;
    uint8_t draw_progress_pct;
    uint16_t trajectory_task_id;
    uint8_t trajectory_segment_count;
    uint8_t trajectory_segment_cursor;
    uint8_t trajectory_received_count;
    uint8_t trajectory_status_flags;
    uint32_t trajectory_received_mask_low;
    uint16_t trajectory_received_mask_high;
};

SlaveMotionSnapshot snapshotSlaveMotion();
void publishSlaveControlStateIfDue(const SlaveLocalControlRuntime &state);
