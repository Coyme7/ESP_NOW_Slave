#pragma once

#include <stdint.h>

#include "common/protocol/protocol_types.h"

struct SlaveAutoDrawRuntimeOutput {
    bool pen_req;
    bool has_paper_target;
    float x_mm;
    float y_mm;
    uint8_t draw_state;
    uint8_t progress_pct;
    uint16_t trajectory_task_id;
    uint8_t trajectory_segment_count;
    uint8_t trajectory_segment_cursor;
    uint8_t trajectory_received_count;
    uint8_t trajectory_status_flags;
    uint32_t trajectory_received_mask_low;
    uint16_t trajectory_received_mask_high;
};

SlaveAutoDrawRuntimeOutput updateSlaveAutoDrawRuntime(bool request_active, float dt_s);
bool acceptSlaveTrajectorySegment(const TrajectorySegmentPacket &packet, uint32_t now_us);
