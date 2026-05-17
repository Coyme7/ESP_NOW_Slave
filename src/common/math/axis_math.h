#pragma once

#include <stdint.h>

// 主从 XY 框架共用的轴标识。当前默认只启用 X 轴，Y 轴仅作为软件框架和后续 bring-up 入口。
enum AxisId : uint8_t {
    AXIS_X = 0,
    AXIS_Y = 1,
};

// 返回短轴名，供低频启动日志和状态日志使用；不要在控制热路径里依赖字符串处理。
inline const char *axisIdName(AxisId axis) {
    return (axis == AXIS_Y) ? "Y" : "X";
}
