#pragma once

#include "common/app/mode_capability.h"
#include "slave/modes/mode_types.h"

const char *slaveRunModeName();
const char *slaveAppModeNameForCurrent();
ModeCapability slaveModeCapabilityForApp(SlaveAppMode app_mode);
ModeCapability slaveModeCapability();
