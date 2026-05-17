#include <Arduino.h>

#include "slave/config/slave_config.h"
#include "slave/hardware/slave_hardware.h"
#include "slave/tasks/slave_tasks.h"
#include "slave/comm/slave_transport.h"

// 从机固件入口。
// 这个文件只负责“上电顺序”：启用 Arduino 运行时、启动串口、先进入安全输出状态、
// 再初始化 X 轴硬件和 ESP-NOW，最后创建任务。运动控制、UV 安全和遥测发送都在
// slave/* 分层模块中实现，便于单独审查安全边界。
extern "C" void app_main() {
    // 当前工程使用 ESP-IDF app_main 入口，需要显式初始化 Arduino 兼容层。
    initArduino();
    Serial.begin(115200);

    // 从机最重要的上电默认：紫光灯关闭，X/Y 电机使能关闭。
    // 这一步必须早于无线初始化和任务启动，避免通信包或任务调度前出现误输出。
    configureSlaveSafeOutputs();

    // 默认只初始化 X 轴真实同步硬件；Y 轴硬件默认关闭，除非显式进入 bring-up。
    const bool motor_ready = setupSlaveXMotorHardware();
    const bool y_ready = setupSlaveYHardware();

    // ESP-NOW 初始化完成后打印本机 MAC 和硬编码主机 MAC，方便现场核对两块板。
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
    Serial.printf("[SlaveConfig] mode=%s perf=%s x_motor_hw=%u y_motor_hw=%u x_sensor_hw=%u y_sensor_hw=%u uv_hw=%u fast_sensor=%u timing_level=%u status_log=%u planner_div=%lu snapshot_pub_div=%lu runtime_pub_div=%lu x_foc_div=%lu x_move_div=%lu espnow_channel=%u auto_draw=%u y_sim=%u y_bringup=%s\n",
                  slaveRunModeName(),
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
                  static_cast<unsigned int>(SLAVE_ESPNOW_CHANNEL),
                  SLAVE_AUTO_DRAW_ENABLED ? 1 : 0,
                  SLAVE_Y_SIMULATION_ENABLED ? 1 : 0,
                  slaveYBringupModeName());

    // boot 行给硬件联调使用：确认 X/Y 纸面半幅、投影距离、控制周期和通信周期。
    Serial.printf("[Slave] boot motor_hw=%u espnow=%u x_half=%.1fmm throw=%.1fmm control=%luus comm=%lums vlim=%.2fV vel=%.2frad/s angleP=%.2f\n",
                  motor_ready ? 1 : 0,
                  SLAVE_ESPNOW_ENABLED ? 1 : 0,
                  PLOT_X_HALF_RANGE_MM,
                  kSlaveXAxis.throw_distance_mm,
                  static_cast<unsigned long>(CONTROL_LOOP_PERIOD_US),
                  static_cast<unsigned long>(SLAVE_TELEMETRY_PERIOD_MS),
                  kSlaveMotorFoc.motor_voltage_limit_v,
                  kSlaveMotorFoc.velocity_limit_rad_s,
                  kSlaveMotorFoc.angle_p);
    Serial.printf("[Slave] y_ready=%u y_half=%.1fmm y_throw=%.1fmm\n",
                  y_ready ? 1 : 0,
                  kSlaveYAxis.half_range_mm,
                  kSlaveYAxis.throw_distance_mm);
    Serial.printf("[Slave] status_period=%lums\n",
                  static_cast<unsigned long>(SLAVE_STATUS_LOOP_PERIOD_MS));
    Serial.printf("[Slave] foc_every_n_steps=%lu\n",
                  static_cast<unsigned long>(SLAVE_X_FOC_EVERY_N_STEPS));
#endif

    // 进入多任务模型：Core 1 只跑 X 轴控制，Core 0 处理通信、UV 安全和状态打印。
    startSlaveTasks();
}
