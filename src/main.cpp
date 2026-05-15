#include <Arduino.h>

#include "slave/slave_config.h"
#include "slave/slave_hardware.h"
#include "slave/slave_tasks.h"
#include "slave/slave_transport.h"

// 从机固件入口。
// 这个文件只负责“上电顺序”：启用 Arduino 运行时、启动串口、先进入安全输出状态、
// 再初始化 X 轴硬件和 ESP-NOW，最后创建任务。运动控制、UV 安全和遥测发送都在
// slave/* 模块中实现，便于单独审查安全边界。
extern "C" void app_main() {
    // 当前工程使用 ESP-IDF app_main 入口，需要显式初始化 Arduino 兼容层。
    initArduino();
    Serial.begin(115200);

    // 从机最重要的上电默认：紫光灯关闭，X/Y 电机使能关闭。
    // 这一步必须早于无线初始化和任务启动，避免通信包或任务调度前出现误输出。
    configureSlaveSafeOutputs();

    // 默认 SLAVE_X_MOTOR_HW_ENABLED=0，因此通常只锁存 MOTOR_OUTPUT_DISABLED。
    // 第一阶段用仿真跟随验证通信和安全联锁，不直接驱动真实云台。
    const bool motor_ready = setupSlaveXMotorHardware();

    // ESP-NOW 初始化完成后打印本机 MAC 和硬编码主机 MAC，方便现场核对两块板。
    #if SLAVE_ESPNOW_ENABLED
    setupSlaveEspNow();
    printSlaveEspNowIdentity();
    #else
    Serial.println("[Slave] espnow disabled for local motion/uv test");
    #endif

    // boot 行给硬件联调使用：确认 X 纸面半幅、投影距离、控制周期和通信周期。
    Serial.printf("[Slave] boot motor_hw=%u espnow=%u x_half=%.1fmm throw=%.1fmm control=%luus comm=%lums vlim=%.2fV vel=%.2frad/s angleP=%.2f\n",
                  motor_ready ? 1 : 0,
                  SLAVE_ESPNOW_ENABLED ? 1 : 0,
                  PLOT_X_HALF_RANGE_MM,
                  kSlaveXAxis.throw_distance_mm,
                  static_cast<unsigned long>(CONTROL_LOOP_PERIOD_US),
                  static_cast<unsigned long>(COMM_LOOP_PERIOD_MS),
                  kSlaveMotorFoc.motor_voltage_limit_v,
                  kSlaveMotorFoc.velocity_limit_rad_s,
                  kSlaveMotorFoc.angle_p);

    // 进入多任务模型：Core 1 只跑 X 轴控制，Core 0 处理通信、UV 安全和状态打印。
    startSlaveTasks();
}
