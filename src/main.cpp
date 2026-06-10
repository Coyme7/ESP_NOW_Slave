#include <Arduino.h>

#include "common/timing/link_timing.h"
#include "slave/comm/slave_transport.h"
#include "slave/config/slave_config.h"
#include "slave/hardware/slave_hardware.h"
#include "slave/modes/mode_traits.h"
#include "slave/modes/mode_manager.h"
#include "slave/modes/mode_table.h"
#include "slave/tasks/slave_tasks.h"
#include "slave/vofa_tuner/vofa_tuner.h"

namespace {

struct SlaveHardwareBootState {
    bool x_sensor_ready;
    bool x_motor_ready;
    bool y_sensor_ready;
    bool y_motor_ready;
};

__attribute__((unused)) const char *slaveRunPathName() {
    switch (SLAVE_RUN_MODE) {
        case SLAVE_MODE_SINGLE_X_5KHZ_ID:
            return "x_closed_loop";
        case SLAVE_MODE_SINGLE_Y_5KHZ_ID:
            return "y_closed_loop";
        case SLAVE_MODE_DUAL_XY_4KHZ_ID:
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

    configureSlaveSafeOutputs();
#if SLAVE_VOFA_TUNER_ENABLED
    (void)startSlaveVofaTunerTask();
#endif
    const SlaveHardwareBootState hw = setupSlaveHardwareForRunMode();
    captureSlaveCurrentSenseRadioBaseline();
#if !SLAVE_BOOT_LOG_ENABLED
    (void)hw;
#endif

#if SLAVE_ESPNOW_ENABLED
    setupSlaveEspNow();
#if SLAVE_BOOT_LOG_ENABLED
    printSlaveEspNowIdentity();
    logSlaveCurrentSenseRadioFreezeProbe();
#endif
#else
#if SLAVE_BOOT_LOG_ENABLED
    Serial.println("[Slave] espnow disabled for local motion/uv test");
    logSlaveCurrentSenseRadioFreezeProbe();
#endif
#endif

    const bool current_sense_runtime_ready =
        finalizeSlaveCurrentSenseRuntimeValidation();
#if !SLAVE_BOOT_LOG_ENABLED
    (void)current_sense_runtime_ready;
#else
    if (!current_sense_runtime_ready) {
        Serial.println("[Slave] current_sense runtime validation failed");
    }
#endif

#if SLAVE_BOOT_LOG_ENABLED
    Serial.println("[SlaveBoot]");
    Serial.println("  mode:");
    Serial.printf("    run=%s path=%s default_app=%s perf=%s\n",
                  slaveRunModeName(),
                  slaveRunPathName(),
                  slaveAppModeName(slaveDefaultAppMode()),
                  slaveControlPerfModeName());
    Serial.printf("    control=%luus planner_div=%lu snapshot_div=%lu runtime_div=%lu\n",
                  static_cast<unsigned long>(CONTROL_LOOP_PERIOD_US),
                  static_cast<unsigned long>(SLAVE_PLANNER_EVERY_N_STEPS),
                  static_cast<unsigned long>(SLAVE_MOTION_SNAPSHOT_EVERY_N_STEPS),
                  static_cast<unsigned long>(SLAVE_RUNTIME_PUBLISH_EVERY_N_STEPS));
    Serial.printf("    x_foc=%lu x_move=%lu y_foc=%lu y_move=%lu\n\n",
                  static_cast<unsigned long>(SLAVE_X_FOC_EVERY_N_STEPS),
                  static_cast<unsigned long>(SLAVE_X_MOVE_EVERY_N_STEPS),
                  static_cast<unsigned long>(SLAVE_Y_FOC_EVERY_N_STEPS),
                  static_cast<unsigned long>(SLAVE_Y_MOVE_EVERY_N_STEPS));

    Serial.println("  hardware:");
    Serial.printf("    x: sensor=%u motor=%u ready=%u/%u\n",
                  SLAVE_X_SENSOR_HW_ENABLED ? 1 : 0,
                  SLAVE_X_MOTOR_HW_ENABLED ? 1 : 0,
                  hw.x_sensor_ready ? 1 : 0,
                  hw.x_motor_ready ? 1 : 0);
    Serial.printf("    y: sensor=%u motor=%u ready=%u/%u sim=%u\n",
                  SLAVE_Y_SENSOR_HW_ENABLED ? 1 : 0,
                  SLAVE_Y_MOTOR_HW_ENABLED ? 1 : 0,
                  hw.y_sensor_ready ? 1 : 0,
                  hw.y_motor_ready ? 1 : 0,
                  SLAVE_Y_SIMULATION_ENABLED ? 1 : 0);
    Serial.printf("    uv=%u espnow=%u channel=%u fast_sensor=%u auto_draw=%u status_log=%u timing=%u\n\n",
                  SLAVE_UV_HW_ENABLED ? 1 : 0,
                  SLAVE_ESPNOW_ENABLED ? 1 : 0,
                  static_cast<unsigned int>(SLAVE_ESPNOW_CHANNEL),
                  SLAVE_FAST_SENSOR_READER_ENABLED ? 1 : 0,
                  SLAVE_AUTO_DRAW_ENABLED ? 1 : 0,
                  SLAVE_STATUS_LOG_ENABLED ? 1 : 0,
                  static_cast<unsigned int>(SLAVE_TIMING_DIAG_LEVEL));

    Serial.println("  motion:");
    Serial.printf("    paper=%.0fx%.0fmm distance=%.0fmm x_half=%.1fmm y_half=%.1fmm\n",
                  kSlavePaperGeometry.width_mm,
                  kSlavePaperGeometry.height_mm,
                  kSlavePaperGeometry.distance_mm,
                  PLOT_X_HALF_RANGE_MM,
                  PLOT_Y_HALF_RANGE_MM);
    Serial.printf("    x_limit=%.1f..%.1fmm y_limit=%.1f..%.1fmm telemetry=%lums status=%lums\n\n",
                  kSlaveXAxisLimit.min_mm,
                  kSlaveXAxisLimit.max_mm,
                  kSlaveYAxisLimit.min_mm,
                  kSlaveYAxisLimit.max_mm,
                  static_cast<unsigned long>(SLAVE_TELEMETRY_PERIOD_MS),
                  static_cast<unsigned long>(SLAVE_STATUS_LOOP_PERIOD_MS));

    Serial.println("  motor:");
    Serial.printf("    x: vlim=%.2fV current=%.3fA vel=%.2frad/s anglePID=%.2f/%.2f/%.2f\n",
                  kSlaveXMotorFoc.voltage.motor_limit_v,
                  kSlaveXMotorFoc.limit.current_a,
                  kSlaveXMotorFoc.limit.velocity_rad_s,
                  kSlaveXMotorFoc.position.p,
                  kSlaveXMotorFoc.position.i,
                  kSlaveXMotorFoc.position.d);
    Serial.printf("    y: vlim=%.2fV current=%.3fA vel=%.2frad/s anglePID=%.2f/%.2f/%.2f throw=%.1fmm\n",
                  kSlaveYMotorFoc.voltage.motor_limit_v,
                  kSlaveYMotorFoc.limit.current_a,
                  kSlaveYMotorFoc.limit.velocity_rad_s,
                  kSlaveYMotorFoc.position.p,
                  kSlaveYMotorFoc.position.i,
                  kSlaveYMotorFoc.position.d,
                  kSlaveYAxis.geometry.throw_distance_mm);
    Serial.printf("    current_sense=%u adc_backend=mcpwm_sync_fast zero_test=%u diag_test=%u shunt=%.4fohm gain=%.1f lpf=%.4fs\n",
                  SLAVE_ENABLE_CURRENT_SENSE ? 1 : 0,
                  SLAVE_ENABLE_ZERO_CURRENT_TEST ? 1 : 0,
                  SLAVE_ENABLE_CURRENT_SENSE_DIAG_TEST ? 1 : 0,
                  kSlaveCurrentSenseHardware.shunt_ohm,
                  kSlaveCurrentSenseHardware.gain,
                  kSlaveXMotorFoc.current_loop.lpf_tf);
    Serial.printf("    foc_startup_skip=%u x_dir=%d x_zero=%.6frad y_dir=%d y_zero=%.6frad\n",
                  SLAVE_SKIP_FOC_ALIGNMENT_ON_STARTUP ? 1 : 0,
                  static_cast<int>(SLAVE_X_FOC_SENSOR_DIRECTION),
                  static_cast<float>(SLAVE_X_ZERO_ELECTRIC_ANGLE_RAD),
                  static_cast<int>(SLAVE_Y_FOC_SENSOR_DIRECTION),
                  static_cast<float>(SLAVE_Y_ZERO_ELECTRIC_ANGLE_RAD));
#endif

#if SLAVE_VOFA_TUNER_ENABLED
    setSlaveVofaHardwareReady(true);
#endif
    startSlaveTasks();
}
