#include "slave/modes/ble/ble_mode.h"

ModeCapability slaveBleSafeCapability() {
    ModeCapability capability = {};
    // BLE 是主机单机模式；从机侧保持 safe/idle，不允许远程绘图、轨迹或 UV。
    capability.flags = MODE_CAP_NONE;
    capability.control_rate_hz = 0;
    capability.outer_rate_hz = 0;
    return capability;
}

