#pragma once

#include "common/motion/paper_mapper.h"

static constexpr PaperGeometry kSlavePaperGeometry = {
    A4_WIDTH_MM,            // 纸面宽度，单位 mm；默认 A4 竖放。
    A4_HEIGHT_MM,           // 纸面高度，单位 mm；默认 A4 竖放。
    DEFAULT_THROW_DISTANCE_MM, // 云台到纸面距离，单位 mm。
    0.0f, // X 纸面中心偏移，单位 mm。
    0.0f, // Y 纸面中心偏移，单位 mm。
    0.0f, // X 中心角，单位 rad。
    0.0f, // Y 中心角，单位 rad。
    1.0f, // X 投影方向符号。
    1.0f, // Y 投影方向符号。
};
