#include "slave/control/slave_auto_draw_runtime.h"

#include <Arduino.h>
#include <stddef.h>

#include "common/protocol/protocol_units.h"
#include "slave/config/slave_config.h"
#include "slave/motion/trajectory_player.h"

namespace {

#if SLAVE_AUTO_DRAW_ENABLED
constexpr size_t kDrawProgramCapacity = 48;

struct AutoDrawState {
    uint8_t draw_state;
    uint8_t progress_pct;
    uint8_t effective_pen_req;
    uint16_t active_task_id;
    uint8_t active_segment_count;
    uint8_t segment_cursor;
    uint8_t received_count;
    uint8_t status_flags;
    uint64_t received_mask;
    size_t segment_count;
    bool request_active;
};

struct ReceivedTrajectoryState {
    uint16_t task_id;
    uint8_t segment_count;
    uint8_t received_count;
    uint8_t last_segment_index;
    uint64_t received_mask;
    uint32_t last_update_us;
    bool complete;
};

struct TrajectoryRxSnapshot {
    uint16_t task_id;
    uint8_t segment_count;
    uint8_t received_count;
    uint8_t last_segment_index;
    uint64_t received_mask;
    bool complete;
};

DrawSegment receivedSegments[kDrawProgramCapacity] = {};
DrawSegment drawSegments[kDrawProgramCapacity] = {};
TrajectoryPlayerState trajectoryState = {};
ReceivedTrajectoryState receivedTrajectory = {};
portMUX_TYPE trajectoryRxMux = portMUX_INITIALIZER_UNLOCKED;
AutoDrawState autoDrawState = {
    DRAW_STATE_IDLE,
    0U,
    0U,
    0U,
    0U,
    0U,
    0U,
    0U,
    0ULL,
    0U,
    false,
};

float q10ToMm(int16_t value) {
    return static_cast<float>(value) * 0.1f;
}

float feedQ10ToMmS(uint16_t value) {
    const float feed = static_cast<float>(value) * 0.1f;
    return (feed > 0.0f) ? feed : 1.0f;
}

uint8_t sizeToTrajectoryCursor(size_t value) {
    return (value > UINT8_MAX) ? UINT8_MAX : static_cast<uint8_t>(value);
}

uint64_t trajectoryMaskForSegmentCount(uint8_t segment_count) {
    if (segment_count == 0U) {
        return 0ULL;
    }
    if (segment_count >= 64U) {
        return UINT64_MAX;
    }
    return (1ULL << segment_count) - 1ULL;
}

uint32_t trajectoryMaskLow(uint64_t mask) {
    return static_cast<uint32_t>(mask & 0xffffffffULL);
}

uint16_t trajectoryMaskHigh(uint64_t mask) {
    return static_cast<uint16_t>((mask >> 32) & 0xffffULL);
}

TrajectoryRxSnapshot snapshotReceivedTrajectoryState() {
    TrajectoryRxSnapshot snapshot = {};
    portENTER_CRITICAL(&trajectoryRxMux);
    snapshot.task_id = receivedTrajectory.task_id;
    snapshot.segment_count = receivedTrajectory.segment_count;
    snapshot.received_count = receivedTrajectory.received_count;
    snapshot.last_segment_index = receivedTrajectory.last_segment_index;
    snapshot.received_mask = receivedTrajectory.received_mask;
    snapshot.complete = receivedTrajectory.complete;
    portEXIT_CRITICAL(&trajectoryRxMux);
    return snapshot;
}

uint8_t loadingTrajectoryStatusFlags(const TrajectoryRxSnapshot &snapshot) {
    uint8_t flags = TRAJECTORY_STATUS_LOADING;
    if (snapshot.segment_count > 0U) {
        flags |= TRAJECTORY_STATUS_TASK_KNOWN;
    }
    if (snapshot.complete) {
        flags |= TRAJECTORY_STATUS_READY;
    } else if (snapshot.segment_count > 0U &&
               snapshot.received_count < snapshot.segment_count) {
        flags |= TRAJECTORY_STATUS_SEGMENT_MISSING;
    }
    return flags;
}

uint8_t receivedTrajectoryProgressPct() {
    uint8_t received_count = 0U;
    uint8_t segment_count = 0U;
    portENTER_CRITICAL(&trajectoryRxMux);
    received_count = receivedTrajectory.received_count;
    segment_count = receivedTrajectory.segment_count;
    portEXIT_CRITICAL(&trajectoryRxMux);

    if (segment_count == 0U) {
        return 0U;
    }
    const uint16_t pct =
        (static_cast<uint16_t>(received_count) * 100U) / static_cast<uint16_t>(segment_count);
    return (pct > 100U) ? 100U : static_cast<uint8_t>(pct);
}

bool copyReceivedTrajectoryIfReady(uint16_t &task_id, uint8_t &segment_count) {
    bool ready = false;
    portENTER_CRITICAL(&trajectoryRxMux);
    if (receivedTrajectory.complete &&
        receivedTrajectory.segment_count > 0U &&
        receivedTrajectory.segment_count <= kDrawProgramCapacity) {
        task_id = receivedTrajectory.task_id;
        segment_count = receivedTrajectory.segment_count;
        for (uint8_t i = 0; i < segment_count; ++i) {
            drawSegments[i] = receivedSegments[i];
        }
        ready = true;
    }
    portEXIT_CRITICAL(&trajectoryRxMux);
    return ready;
}

bool startAutoDrawProgram() {
    uint16_t task_id = 0U;
    uint8_t segment_count = 0U;
    if (!copyReceivedTrajectoryIfReady(task_id, segment_count)) {
        const TrajectoryRxSnapshot rx = snapshotReceivedTrajectoryState();
        autoDrawState.draw_state = DRAW_STATE_LOADING;
        autoDrawState.progress_pct = receivedTrajectoryProgressPct();
        autoDrawState.effective_pen_req = 0U;
        autoDrawState.active_task_id = rx.task_id;
        autoDrawState.active_segment_count = rx.segment_count;
        autoDrawState.segment_cursor = rx.last_segment_index;
        autoDrawState.received_count = rx.received_count;
        autoDrawState.status_flags = loadingTrajectoryStatusFlags(rx);
        autoDrawState.received_mask = rx.received_mask;
        autoDrawState.segment_count = rx.segment_count;
        return false;
    }

    if (segment_count == 0U) {
        autoDrawState.draw_state = DRAW_STATE_BLOCKED;
        autoDrawState.progress_pct = 0U;
        autoDrawState.effective_pen_req = 0U;
        autoDrawState.active_task_id = task_id;
        autoDrawState.active_segment_count = 0U;
        autoDrawState.segment_cursor = 0U;
        autoDrawState.received_count = 0U;
        autoDrawState.status_flags = TRAJECTORY_STATUS_BLOCKED;
        autoDrawState.received_mask = 0ULL;
        autoDrawState.segment_count = 0U;
        return false;
    }

    startTrajectoryPlayer(trajectoryState, drawSegments, segment_count);
    autoDrawState.active_task_id = task_id;
    autoDrawState.active_segment_count = segment_count;
    autoDrawState.segment_cursor = 0U;
    autoDrawState.received_count = segment_count;
    autoDrawState.status_flags =
        TRAJECTORY_STATUS_TASK_KNOWN | TRAJECTORY_STATUS_READY | TRAJECTORY_STATUS_RUNNING;
    autoDrawState.received_mask = trajectoryMaskForSegmentCount(segment_count);
    autoDrawState.segment_count = segment_count;
    autoDrawState.draw_state = DRAW_STATE_RUNNING;
    autoDrawState.progress_pct = 0U;
    autoDrawState.effective_pen_req = 0U;
    return true;
}

TrajectoryPlayerOutput updateAutoDrawProgram(bool request_active, float dt_s) {
    TrajectoryPlayerOutput output = {};
    if (!request_active) {
        if (autoDrawState.request_active || trajectoryState.active) {
            stopTrajectoryPlayer(trajectoryState);
        }
        autoDrawState.request_active = false;
        autoDrawState.draw_state = DRAW_STATE_IDLE;
        autoDrawState.progress_pct = 0U;
        autoDrawState.effective_pen_req = 0U;
        autoDrawState.active_task_id = 0U;
        autoDrawState.active_segment_count = 0U;
        autoDrawState.segment_cursor = 0U;
        autoDrawState.received_count = 0U;
        autoDrawState.status_flags = TRAJECTORY_STATUS_NONE;
        autoDrawState.received_mask = 0ULL;
        autoDrawState.segment_count = 0U;
        return output;
    }

    if (!autoDrawState.request_active ||
        (!trajectoryState.active && !trajectoryState.finished)) {
        if (!startAutoDrawProgram()) {
            autoDrawState.request_active = true;
            return output;
        }
    }
    autoDrawState.request_active = true;

    output = updateTrajectoryPlayer(trajectoryState, dt_s);
    autoDrawState.progress_pct = output.progress_pct;
    if (output.finished) {
        autoDrawState.draw_state = DRAW_STATE_FINISHED;
        autoDrawState.progress_pct = 100U;
        autoDrawState.effective_pen_req = 0U;
        autoDrawState.segment_cursor = autoDrawState.active_segment_count;
        autoDrawState.received_count = autoDrawState.active_segment_count;
        autoDrawState.status_flags =
            TRAJECTORY_STATUS_TASK_KNOWN | TRAJECTORY_STATUS_COMPLETE;
        autoDrawState.received_mask =
            trajectoryMaskForSegmentCount(autoDrawState.active_segment_count);
        output.pen_req = false;
        return output;
    }

    autoDrawState.draw_state = output.active ? DRAW_STATE_RUNNING : DRAW_STATE_BLOCKED;
    autoDrawState.effective_pen_req = output.pen_req ? 1U : 0U;
    autoDrawState.segment_cursor = sizeToTrajectoryCursor(output.segment_index);
    autoDrawState.segment_count = output.segment_count;
    autoDrawState.received_count = autoDrawState.active_segment_count;
    autoDrawState.status_flags =
        TRAJECTORY_STATUS_TASK_KNOWN |
        (output.active ? TRAJECTORY_STATUS_RUNNING : TRAJECTORY_STATUS_BLOCKED);
    autoDrawState.received_mask =
        trajectoryMaskForSegmentCount(autoDrawState.active_segment_count);
    return output;
}

SlaveAutoDrawRuntimeOutput makeAutoDrawRuntimeOutput(const TrajectoryPlayerOutput &output) {
    SlaveAutoDrawRuntimeOutput state = {};
    state.pen_req = output.pen_req;
    state.has_paper_target = output.active || output.finished;
    state.x_mm = output.x_mm;
    state.y_mm = output.y_mm;
    state.draw_state = autoDrawState.draw_state;
    state.progress_pct = autoDrawState.progress_pct;
    state.trajectory_task_id = autoDrawState.active_task_id;
    state.trajectory_segment_count = autoDrawState.active_segment_count;
    state.trajectory_segment_cursor = autoDrawState.segment_cursor;
    state.trajectory_received_count = autoDrawState.received_count;
    state.trajectory_status_flags = autoDrawState.status_flags;
    state.trajectory_received_mask_low = trajectoryMaskLow(autoDrawState.received_mask);
    state.trajectory_received_mask_high = trajectoryMaskHigh(autoDrawState.received_mask);
    return state;
}
#endif

}  // namespace

SlaveAutoDrawRuntimeOutput updateSlaveAutoDrawRuntime(bool request_active, float dt_s) {
#if SLAVE_AUTO_DRAW_ENABLED
    return makeAutoDrawRuntimeOutput(updateAutoDrawProgram(request_active, dt_s));
#else
    (void)request_active;
    (void)dt_s;
    return {};
#endif
}

bool acceptSlaveTrajectorySegment(const TrajectorySegmentPacket &packet, uint32_t now_us) {
#if SLAVE_AUTO_DRAW_ENABLED
    if (packet.segment_count == 0U ||
        packet.segment_count > kDrawProgramCapacity ||
        packet.segment_index >= packet.segment_count ||
        packet.feed_mm_s_q10 == 0U) {
        return false;
    }

    const uint64_t segment_bit = 1ULL << packet.segment_index;
    DrawSegment segment = {};
    segment.start = {
        q10ToMm(packet.start_x_mm_q10),
        q10ToMm(packet.start_y_mm_q10),
        packet.pen_req != 0U,
    };
    segment.end = {
        q10ToMm(packet.end_x_mm_q10),
        q10ToMm(packet.end_y_mm_q10),
        packet.pen_req != 0U,
    };
    segment.feed_mm_s = feedQ10ToMmS(packet.feed_mm_s_q10);

    portENTER_CRITICAL(&trajectoryRxMux);
    if (receivedTrajectory.task_id != packet.task_id ||
        receivedTrajectory.segment_count != packet.segment_count) {
        receivedTrajectory = {};
        receivedTrajectory.task_id = packet.task_id;
        receivedTrajectory.segment_count = packet.segment_count;
    }

    if ((receivedTrajectory.received_mask & segment_bit) == 0ULL) {
        receivedSegments[packet.segment_index] = segment;
        receivedTrajectory.received_mask |= segment_bit;
        receivedTrajectory.received_count++;
        receivedTrajectory.complete =
            receivedTrajectory.received_count >= receivedTrajectory.segment_count;
    }
    receivedTrajectory.last_segment_index = packet.segment_index;
    receivedTrajectory.last_update_us = now_us;
    portEXIT_CRITICAL(&trajectoryRxMux);
    return true;
#else
    (void)packet;
    (void)now_us;
    return false;
#endif
}
