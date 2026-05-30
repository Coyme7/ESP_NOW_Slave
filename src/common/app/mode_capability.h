#pragma once

#include <stdint.h>

#include "common/protocol/protocol_types.h"

enum ModeCapabilityFlags : uint16_t {
    MODE_CAP_NONE = 0,
    MODE_CAP_X_SENSOR = 1u << 0,
    MODE_CAP_Y_SENSOR = 1u << 1,
    MODE_CAP_X_MOTOR = 1u << 2,
    MODE_CAP_Y_MOTOR = 1u << 3,
    MODE_CAP_REMOTE_COMMAND = 1u << 4,
    MODE_CAP_PEN = 1u << 5,
    MODE_CAP_UV = 1u << 6,
    MODE_CAP_TRAJECTORY = 1u << 7,
    MODE_CAP_DRY_RUN = 1u << 8,
};

struct ModeCapability {
    uint16_t flags;
    uint16_t control_rate_hz;
    uint16_t outer_rate_hz;
};

inline bool modeHasCapability(const ModeCapability &capability, uint16_t flag) {
    return (capability.flags & flag) != 0;
}

