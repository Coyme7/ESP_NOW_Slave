#pragma once

#include <stdint.h>

// 从机业务模式描述绘图语义，不描述本次固件启用了哪些外设 bring-up 开关。
enum SlaveAppMode : uint8_t {
    SLAVE_APP_MODE_MANUAL_DRAW = 0,
    SLAVE_APP_MODE_AUTO_DRAW = 1,
    SLAVE_APP_MODE_BLE_SAFE = 2,
    SLAVE_APP_MODE_DIAGNOSTICS = 3,
};

struct SlaveRuntimeModeSnapshot {
    uint8_t active_mode;
    uint8_t requested_mode;
    uint8_t request_accepted;
    uint8_t request_rejected;
    uint8_t last_protocol_mode;
    uint16_t last_command_flags;
    uint32_t request_count;
    uint32_t last_change_ms;
};
