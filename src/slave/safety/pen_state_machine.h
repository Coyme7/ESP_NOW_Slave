#pragma once

#include <stdint.h>

#include "common/protocol/protocol_types.h"

struct PenStateMachineInput {
    bool pen_req;
    bool uv_allowed;
    bool hard_fault;
};

struct PenStateMachineState {
    uint8_t state;
};

uint8_t updatePenStateMachine(PenStateMachineState &state,
                              const PenStateMachineInput &input);

