#include <Arduino.h>

#include "common/timing/link_timing.h"
#include "slave/comm/slave_transport.h"
#include "slave/config/slave_config.h"
#include "slave/hardware/slave_hardware.h"
#include "slave/modes/mode_traits.h"
#include "slave/modes/mode_manager.h"
#include "slave/modes/mode_table.h"
#include "slave/tasks/slave_tasks.h"

namespace {

struct SlaveHardwareBootState {
    bool x_sensor_ready;
    bool x_motor_ready;
    bool y_sensor_ready;
    bool y_motor_ready;
};

const char *slaveRunPathName() {
    switch (SLAVE_RUN_MODE) {
        case SLAVE_MODE_SINGLE_X_5KHZ_ID:
            return "x_closed_loop";
        case SLAVE_MODE_SINGLE_Y_5KHZ_ID:
            return "y_closed_loop";
        case SLAVE_MODE_DUAL_XY_2KHZ_ID:
            return "dual_xy_closed_loop";
        case SLAVE_MODE_DUAL_XY_DRY_RUN_ID:
            return "dual_xy_dry_run";
        case SLAVE_MODE_YSENSOR_ONLY_ID:
            return "y_sensor_only";
        case SLAVE_MODE_Y_OPEN_LOOP_ID:
            return "y_open_loop";
        case SLAVE_MODE_Y_CLOSED_LOOP_ID:
            return "y_closed_loop";
        default:
            return "unknown";
    }
}

SlaveHardwareBootState setupSlaveHardwareForRunMode() {
    SlaveHardwareBootState state = {};

    if (slaveRunModeNeedsSensorHardware(AXIS_X)) {
        state.x_sensor_ready = setupSlaveXSensorHardware();
    }
    if (slaveRunModeNeedsMotorHardware(AXIS_X)) {
        state.x_motor_ready = setupSlaveXMotorHardware();
        state.x_sensor_ready = state.x_sensor_ready || state.x_motor_ready;
    }

    if (slaveRunModeNeedsSensorHardware(AXIS_Y)) {
        state.y_sensor_ready = setupSlaveYSensorHardware();
    }
    if (slaveRunModeNeedsMotorHardware(AXIS_Y)) {
        state.y_motor_ready = slaveRunModeUsesOpenLoopMotor(AXIS_Y)
                                  ? setupSlaveYMotorOpenLoopHardware()
                                  : setupSlaveYMotorClosedLoopHardware();
        state.y_sensor_ready = state.y_sensor_ready || (!slaveRunModeUsesOpenLoopMotor(AXIS_Y) &&
                                                        state.y_motor_ready);
    }

    return state;
}

}  // namespace

extern "C" void app_main() {
    initArduino();
    Serial.begin(115200);

    // 上电先关闭 UV 和电机使能，再按 SLAVE_RUN_MODE 初始化所需硬件。
    configureSlaveSafeOutputs();
    const SlaveHardwareBootState hw = setupSlaveHardwareForRunMode();

#if SLAVE_ESPNOW_ENABLED
    setupSlaveEspNow();
#if SLAVE_BOOT_LOG_ENABLED
    printSlaveEspNowIdentity();
#endif
#else
#if SLAVE_BOOT_LOG_ENABLED
    Serial.println("[Slave] espnow disabled for local motion/uv test");
#endif
#endif

#if SLAVE_BOOT_LOG_ENABLED
    Serial.printf("[SlaveConfig] run_mode=%s run_path=%s default_app=%s perf=%s x_motor_hw=%u y_motor_hw=%u x_sensor_hw=%u y_sensor_hw=%u uv_hw=%u fast_sensor=%u timing_level=%u status_log=%u planner_div=%lu snapshot_pub_div=%lu runtime_pub_div=%lu x_foc_div=%lu x_move_div=%lu y_foc_div=%lu y_move_div=%lu espnow_channel=%u auto_draw_compiled=%u y_sim=%u paper=%.0fx%.0fmm distance=%.0fmm\n",
                  slaveRunModeName(),
                  slaveRunPathName(),
                  slaveAppModeName(slaveDefaultAppMode()),
                  slaveControlPerfModeName(),
                  SLAVE_X_MOTOR_HW_ENABLED ? 1 : 0,
                  SLAVE_Y_MOTOR_HW_ENABLED ? 1 : 0,
                  SLAVE_X_SENSOR_HW_ENABLED ? 1 : 0,
                  SLAVE_Y_SENSOR_HW_ENABLED ? 1 : 0,
                  SLAVE_UV_HW_ENABLED ? 1 : 0,
                  SLAVE_FAST_SENSOR_READER_ENABLED ? 1 : 0,
                  static_cast<unsigned int>(SLAVE_TIMING_DIAG_LEVEL),
                  SLAVE_STATUS_LOG_ENABLED ? 1 : 0,
                  static_cast<unsigned long>(SLAVE_PLANNER_EVERY_N_STEPS),
                  static_cast<unsigned long>(SLAVE_MOTION_SNAPSHOT_EVERY_N_STEPS),
                  static_cast<unsigned long>(SLAVE_RUNTIME_PUBLISH_EVERY_N_STEPS),
                  static_cast<unsigned long>(SLAVE_X_FOC_EVERY_N_STEPS),
                  static_cast<unsigned long>(SLAVE_X_MOVE_EVERY_N_STEPS),
                  static_cast<unsigned long>(SLAVE_Y_FOC_EVERY_N_STEPS),
                  static_cast<unsigned long>(SLAVE_Y_MOVE_EVERY_N_STEPS),
                  static_cast<unsigned int>(SLAVE_ESPNOW_CHANNEL),
                  SLAVE_AUTO_DRAW_ENABLED ? 1 : 0,
                  SLAVE_Y_SIMULATION_ENABLED ? 1 : 0,
                  kSlavePaperGeometry.width_mm,
                  kSlavePaperGeometry.height_mm,
                  kSlavePaperGeometry.distance_mm);

    Serial.printf("[Slave] boot x_req=%u/%u y_req=%u/%u x_ready=%u/%u y_ready=%u/%u espnow=%u control=%luus comm=%lums x_half=%.1fmm y_half=%.1fmm x_limit=%.1f..%.1fmm y_limit=%.1f..%.1fmm x_vlim=%.2fV y_vlim=%.2fV x_vel=%.2frad/s y_vel=%.2frad/s x_angleP=%.2f y_angleP=%.2f\n",
                  slaveRunModeNeedsSensorHardware(AXIS_X) ? 1 : 0,
                  slaveRunModeNeedsMotorHardware(AXIS_X) ? 1 : 0,
                  slaveRunModeNeedsSensorHardware(AXIS_Y) ? 1 : 0,
                  slaveRunModeNeedsMotorHardware(AXIS_Y) ? 1 : 0,
                  hw.x_sensor_ready ? 1 : 0,
                  hw.x_motor_ready ? 1 : 0,
                  hw.y_sensor_ready ? 1 : 0,
                  hw.y_motor_ready ? 1 : 0,
                  SLAVE_ESPNOW_ENABLED ? 1 : 0,
                  static_cast<unsigned long>(CONTROL_LOOP_PERIOD_US),
                  static_cast<unsigned long>(SLAVE_TELEMETRY_PERIOD_MS),
                  PLOT_X_HALF_RANGE_MM,
                  PLOT_Y_HALF_RANGE_MM,
                  kSlaveXAxisLimit.min_mm,
                  kSlaveXAxisLimit.max_mm,
                  kSlaveYAxisLimit.min_mm,
                  kSlaveYAxisLimit.max_mm,
                  kSlaveXMotorFoc.voltage.motor_limit_v,
                  kSlaveYMotorFoc.voltage.motor_limit_v,
                  kSlaveXMotorFoc.limit.velocity_rad_s,
                  kSlaveYMotorFoc.limit.velocity_rad_s,
                  kSlaveXMotorFoc.position.p,
                  kSlaveYMotorFoc.position.p);
    Serial.printf("[Slave] y_half=%.1fmm y_throw=%.1fmm status_period=%lums\n",
                  kSlaveYAxis.geometry.half_range_mm,
                  kSlaveYAxis.geometry.throw_distance_mm,
                  static_cast<unsigned long>(SLAVE_STATUS_LOOP_PERIOD_MS));
    Serial.printf("[Slave] foc_every_n_steps x=%lu y=%lu move_every_n_steps x=%lu y=%lu\n",
                  static_cast<unsigned long>(SLAVE_X_FOC_EVERY_N_STEPS),
                  static_cast<unsigned long>(SLAVE_Y_FOC_EVERY_N_STEPS),
                  static_cast<unsigned long>(SLAVE_X_MOVE_EVERY_N_STEPS),
                  static_cast<unsigned long>(SLAVE_Y_MOVE_EVERY_N_STEPS));
#endif

    startSlaveTasks();
}
