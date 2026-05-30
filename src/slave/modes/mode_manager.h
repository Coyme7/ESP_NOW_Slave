#pragma once

#include "slave/modes/mode_types.h"

SlaveAppMode slaveDefaultAppMode();
bool slaveAppModeAvailable(SlaveAppMode mode);
uint8_t slaveAcceptedProtocolModeForCommand(uint8_t protocol_mode,
                                            uint16_t command_flags);
void updateSlaveRuntimeModeFromCommand(uint8_t protocol_mode,
                                       uint16_t command_flags,
                                       uint32_t now_ms);
SlaveRuntimeModeSnapshot getSlaveRuntimeModeSnapshot();
SlaveAppMode currentSlaveAppMode();
const char *slaveAppModeName(SlaveAppMode mode);
