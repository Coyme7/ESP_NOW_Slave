#include <Arduino.h>

#include "common/system_state.h"
#include "common/timing/link_timing.h"
#include "slave/comm/slave_transport.h"
#include "slave/config/slave_config.h"
#include "slave/hardware/slave_adc1_dma_sampler.h"
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

__attribute__((unused)) const char *slaveRunPathName() {
    switch (SLAVE_RUN_MODE) {
        case SLAVE_MODE_SINGLE_X_4KHZ_ID:
            return "x_closed_loop";
        case SLAVE_MODE_SINGLE_Y_4KHZ_ID:
            return "y_closed_loop";
        case SLAVE_MODE_DUAL_XY_4KHZ_ID:
            return "dual_xy_closed_loop";
        case SLAVE_MODE_DUAL_XY_DRY_RUN_ID:
            return "dual_xy_dry_run";
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
        state.y_motor_ready = setupSlaveYMotorClosedLoopHardware();
        state.y_sensor_ready = state.y_sensor_ready || state.y_motor_ready;
    }

    return state;
}

}  // namespace

extern "C" void app_main() {
    initArduino();
    Serial.begin(115200);

    // 上电先关闭 UV 和电机使能，再按 SLAVE_RUN_MODE 初始化所需硬件。
    configureSlaveSafeOutputs();
    const bool adc_dma_started = startSlaveAdc1DmaSampler();
    const bool adc_dma_ready =
        adc_dma_started && waitForSlaveAdc1DmaFirstFrame(100U);
    if (!adc_dma_ready && slaveAdc1DmaSamplerRequired()) {
        setUvOutput(false);
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
    }
    const SlaveHardwareBootState hw =
        adc_dma_ready ? setupSlaveHardwareForRunMode() : SlaveHardwareBootState{};
#if !SLAVE_BOOT_LOG_ENABLED
    (void)hw;
    (void)adc_dma_started;
    (void)adc_dma_ready;
#endif

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
    Serial.printf("    current_sense=%u zero_test=%u diag_test=%u shunt=%.4fohm gain=%.1f lpf=%.4fs\n",
                  SLAVE_ENABLE_CURRENT_SENSE ? 1 : 0,
                  SLAVE_ENABLE_ZERO_CURRENT_TEST ? 1 : 0,
                  SLAVE_ENABLE_CURRENT_SENSE_DIAG_TEST ? 1 : 0,
                  kSlaveCurrentSenseHardware.shunt_ohm,
                  kSlaveCurrentSenseHardware.gain,
                  kSlaveXMotorFoc.current_loop.lpf_tf);
    const SlaveAdc1DmaHealthSnapshot adc_health = snapshotSlaveAdc1DmaHealth();
    Serial.printf("    adc_dma required=%u started=%u first=%u seq=%lu invalid=%lu overflow=%lu stale=%lu\n",
                  adc_health.required ? 1 : 0,
                  adc_health.started ? 1 : 0,
                  adc_health.first_frame_ready ? 1 : 0,
                  static_cast<unsigned long>(adc_health.frame_sequence),
                  static_cast<unsigned long>(adc_health.invalid_frames),
                  static_cast<unsigned long>(adc_health.pool_overflows),
                  static_cast<unsigned long>(adc_health.stale_control_cycles));
    Serial.printf("    foc_startup_skip=%u x_dir=%d x_zero=%.6frad y_dir=%d y_zero=%.6frad\n",
                  SLAVE_SKIP_FOC_ALIGNMENT_ON_STARTUP ? 1 : 0,
                  static_cast<int>(SLAVE_X_FOC_SENSOR_DIRECTION),
                  static_cast<float>(SLAVE_X_ZERO_ELECTRIC_ANGLE_RAD),
                  static_cast<int>(SLAVE_Y_FOC_SENSOR_DIRECTION),
                  static_cast<float>(SLAVE_Y_ZERO_ELECTRIC_ANGLE_RAD));
#endif

    startSlaveTasks();
}
