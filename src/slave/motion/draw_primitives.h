#pragma once

struct DrawPoint {
    float x_mm;
    float y_mm;
    bool pen_down;
};

struct DrawSegment {
    DrawPoint start;
    DrawPoint end;
    float feed_mm_s;
};
