#include "slave/control/slave_motion_state.h"

#include <Arduino.h>

#include "common/math/angle_math.h"
#include "common/protocol/protocol_units.h"
#include "common/system_state.h"
#include "slave/config/slave_config.h"
#include "slave/control/slave_coordinate_mapper.h"

namespace {

SlaveMotionSnapshot motionSnapshot = {};
portMUX_TYPE motionSnapshotMux = portMUX_INITIALIZER_UNLOCKED;

bool shouldPublishRuntimeToSysData() {
#if SLAVE_RUNTIME_PUBLISH_EVERY_N_STEPS <= 1
    return true;
#else
    static uint32_t publish_divider = 0;
    publish_divider++;
    if (publish_divider >= SLAVE_RUNTIME_PUBLISH_EVERY_N_STEPS) {
        publish_divider = 0;
        return true;
    }
    return false;
#endif
}

bool shouldPublishMotionSnapshot() {
#if SLAVE_MOTION_SNAPSHOT_EVERY_N_STEPS <= 1
    return true;
#else
    static uint32_t publish_divider = 0;
    const bool should_publish = publish_divider == 0;
    publish_divider++;
    if (publish_divider >= SLAVE_MOTION_SNAPSHOT_EVERY_N_STEPS) {
        publish_divider = 0;
    }
    return should_publish;
#endif
}

SlaveMotionSnapshot makeMotionSnapshot(const SlaveLocalControlRuntime &state) {
    SlaveMotionSnapshot snapshot = {};
    snapshot.target_x_mm = state.target_x_mm;
    snapshot.smooth_x_mm = state.smooth_x_mm;
    snapshot.target_angle_rad = state.target_angle_rad;
    snapshot.actual_angle_rad = state.actual_angle_rad;
    snapshot.x_track_err_mrad = (state.target_angle_rad - state.actual_angle_rad) * 1000.0f;
    snapshot.target_y_mm = state.target_y_mm;
    snapshot.smooth_y_mm = state.smooth_y_mm;
    snapshot.target_y_angle_rad = state.target_y_angle_rad;
    snapshot.actual_y_angle_rad = state.actual_y_angle_rad;
    snapshot.y_track_err_mrad = (state.target_y_angle_rad - state.actual_y_angle_rad) * 1000.0f;
    snapshot.x_limit = state.x_limit;
    snapshot.y_limit = state.y_limit;
    snapshot.x_clamped = state.x_clamped;
    snapshot.y_clamped = state.y_clamped;
    snapshot.boundary_hit = state.boundary_hit;
    snapshot.command_valid = state.command_valid;
    snapshot.pen_req = state.pen_req;
    snapshot.draw_state = state.draw_state;
    snapshot.draw_progress_pct = state.draw_progress_pct;
    snapshot.trajectory_task_id = state.trajectory_task_id;
    snapshot.trajectory_segment_count = state.trajectory_segment_count;
    snapshot.trajectory_segment_cursor = state.trajectory_segment_cursor;
    snapshot.trajectory_received_count = state.trajectory_received_count;
    snapshot.trajectory_status_flags = state.trajectory_status_flags;
    snapshot.trajectory_received_mask_low = state.trajectory_received_mask_low;
    snapshot.trajectory_received_mask_high = state.trajectory_received_mask_high;
    return snapshot;
}

void publishMotionSnapshot(const SlaveLocalControlRuntime &state) {
    const SlaveMotionSnapshot snapshot = makeMotionSnapshot(state);
    portENTER_CRITICAL(&motionSnapshotMux);
    motionSnapshot = snapshot;
    portEXIT_CRITICAL(&motionSnapshotMux);
}

void publishRuntimeToSysData(const SlaveLocalControlRuntime &state) {
    sysData.slave.boundary_hit = state.boundary_hit;
    sysData.slave.target_x_mm = state.target_x_mm;
    sysData.slave.smooth_x_mm = state.smooth_x_mm;
    sysData.slave.target_angle_rad = state.target_angle_rad;
    sysData.slave.actual_angle_rad = state.actual_angle_rad;
    sysData.slave.target_y_mm = state.target_y_mm;
    sysData.slave.smooth_y_mm = state.smooth_y_mm;
    sysData.slave.target_y_angle_rad = state.target_y_angle_rad;
    sysData.slave.actual_y_angle_rad = state.actual_y_angle_rad;
    sysData.slave.x_limit_hit = state.x_limit;
    sysData.slave.y_limit_hit = state.y_limit;
    sysData.slave.x_clamped = state.x_clamped;
    sysData.slave.y_clamped = state.y_clamped;
    sysData.slave.angle_deg = radToDeg(state.actual_angle_rad);
    sysData.slave.x_pos =
        normToPercent(slaveAxisGimbalAngleRadToNorm(AXIS_X, state.actual_angle_rad));
    sysData.slave.y_pos =
        normToPercent(slaveAxisGimbalAngleRadToNorm(AXIS_Y, state.actual_y_angle_rad));
    sysData.slave.draw_state = state.draw_state;
    sysData.slave.draw_progress_pct = state.draw_progress_pct;
    sysData.slave.trajectory_task_id = state.trajectory_task_id;
    sysData.slave.trajectory_segment_count = state.trajectory_segment_count;
    sysData.slave.trajectory_segment_cursor = state.trajectory_segment_cursor;
    sysData.slave.trajectory_received_count = state.trajectory_received_count;
    sysData.slave.trajectory_status_flags = state.trajectory_status_flags;
    sysData.slave.trajectory_received_mask_low = state.trajectory_received_mask_low;
    sysData.slave.trajectory_received_mask_high = state.trajectory_received_mask_high;
}

}  // namespace

SlaveMotionSnapshot snapshotSlaveMotion() {
    SlaveMotionSnapshot snapshot = {};
    portENTER_CRITICAL(&motionSnapshotMux);
    snapshot = motionSnapshot;
    portEXIT_CRITICAL(&motionSnapshotMux);
    return snapshot;
}

void publishSlaveControlStateIfDue(const SlaveLocalControlRuntime &state) {
    if (shouldPublishMotionSnapshot()) {
        publishMotionSnapshot(state);
    }
    if (shouldPublishRuntimeToSysData()) {
        publishRuntimeToSysData(state);
    }
}
