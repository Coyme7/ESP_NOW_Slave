#include "slave/control/slave_coordinate_mapper.h"

#include <math.h>

#include "common/protocol/protocol_units.h"
#include "slave/config/slave_config.h"

float slaveXNormToPaperMm(int16_t x_norm) {
    // x_norm=-1..+1 对应纸面 X=-125..+125mm。
    return normToUnit(x_norm) * PLOT_X_HALF_RANGE_MM;
}

float slavePaperMmToGimbalAngleRad(float x_mm) {
    // 纸距 300mm、X 半幅 125mm 时，端点角约为 atan(125/300)=22.62deg。
    const float limited_x_mm = clampFloat(x_mm, -PLOT_X_HALF_RANGE_MM, PLOT_X_HALF_RANGE_MM);
    return kSlaveXAxis.center_angle_rad +
           (kSlaveXAxis.direction * atanf(limited_x_mm / kSlaveXAxis.throw_distance_mm));
}

int16_t slaveGimbalAngleRadToXNorm(float angle_rad) {
    // 遥测反算：云台角 -> 纸面 X -> 协议归一化坐标。
    const float axis_angle_rad = (angle_rad - kSlaveXAxis.center_angle_rad) * kSlaveXAxis.direction;
    const float x_mm = tanf(axis_angle_rad) * kSlaveXAxis.throw_distance_mm;
    return unitToNorm(x_mm / PLOT_X_HALF_RANGE_MM);
}
