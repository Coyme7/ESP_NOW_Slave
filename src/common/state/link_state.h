#pragma once

#include <stdint.h>

#include "common/protocol/protocol_types.h"
#include "common/state/fault_flags.h"

struct CommonLinkState {
    volatile bool pen_down;
    volatile bool command_valid;
    volatile uint8_t current_mode;
    volatile uint16_t protocol_fault_flags;
    volatile uint32_t last_command_seq;
    volatile uint32_t last_telemetry_seq;
    volatile uint32_t last_rx_us;
    volatile uint32_t espnow_send_ok_count;
    volatile uint32_t espnow_send_fail_count;
    volatile uint32_t espnow_recv_ok_count;
    volatile uint32_t espnow_recv_reject_count;
    volatile uint8_t last_send_ok;
};

inline CommonLinkState makeDefaultCommonLinkState() {
    CommonLinkState state = {};
    state.current_mode = MODE_COLLAB_DRAW;
    state.protocol_fault_flags = FAULT_NONE;
    return state;
}

