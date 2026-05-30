#include "slave/safety/pen_state_machine.h"

uint8_t updatePenStateMachine(PenStateMachineState &state,
                              const PenStateMachineInput &input) {
    if (input.hard_fault) {
        state.state = PEN_FAULT;
        return state.state;
    }

    if (!input.pen_req) {
        state.state = PEN_UP;
        return state.state;
    }

    state.state = input.uv_allowed ? PEN_DOWN : PEN_ARMING;
    return state.state;
}

