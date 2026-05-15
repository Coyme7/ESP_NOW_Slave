#pragma once

#include "common/state/link_state.h"

struct MasterDebugState {
    volatile float angle_deg;
    volatile float target_current_a;
    volatile float current_q_a;
    volatile float current_d_a;
    volatile float voltage_q_v;
    volatile float voltage_d_v;
    volatile float x_pos;
    volatile float y_pos;
    volatile bool boundary_hit;
};

struct SlaveDebugState {
    volatile float angle_deg;
    volatile float x_pos;
    volatile float y_pos;
    volatile float target_angle_rad;
    volatile float actual_angle_rad;
    volatile bool boundary_hit;
    volatile bool uv_interlock_blocked;
};

struct SharedData {
    CommonLinkState link;
    MasterDebugState master;
    SlaveDebugState slave;
};

extern SharedData sysData;

