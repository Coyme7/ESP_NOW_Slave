#pragma once

// 兼容入口：旧代码可继续 include shared_types.h。
// 实际定义已经拆到 common/protocol、common/math、common/timing 和 common/state。
#include "common/math/angle_math.h"
#include "common/protocol/packet_codec.h"
#include "common/protocol/protocol_units.h"
#include "common/state/shared_data.h"
#include "common/timing/loop_timing.h"

