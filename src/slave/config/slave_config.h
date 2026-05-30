#pragma once

#include <Arduino.h>

#include "common/protocol/protocol_types.h"

// 从机配置唯一聚合入口。
// 分类配置只从这里聚合，业务模式和硬件访问分别留在 slave/modes 与 slave/hardware。

#include "slave/config/build/slave_build_options.h"
#include "slave/config/diagnostics/slave_log_config.h"
#include "slave/config/core/slave_task_config.h"

#include "slave/config/core/slave_comm_config.h"
#include "slave/config/core/slave_control_config.h"
#include "slave/config/core/slave_motor_config.h"

#include "slave/config/motion/slave_axis_config.h"
#include "slave/config/motion/slave_paper_config.h"
#include "slave/config/motion/slave_trajectory_config.h"

// 非法组合校验集中入口。必须放在所有分类配置之后。
#include "slave/config/slave_config_validate.h"
