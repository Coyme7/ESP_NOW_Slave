#include "slave/slave_tasks.h"

#include <Arduino.h>

#include "slave/slave_config.h"
#include "slave/slave_motion.h"
#include "slave/slave_safety.h"
#include "slave/slave_status.h"
#include "slave/slave_transport.h"

// 从机任务编排模块。
// 这里把实时控制、UV 安全、ESP-NOW 遥测和串口状态分成独立任务，
// 让 10 kHz 热路径保持窄而可审查。

namespace {

void task_control_loop(void *pvParameters) {
    (void)pvParameters;

    // next_us 是 100 us 子步目标时间；last_wake 用于每 1 ms 重新对齐 FreeRTOS tick。
    uint32_t next_us = micros();
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        // 每个 1 ms tick 内执行多个 100 us 子步，目标控制频率约 10 kHz。
        for (uint32_t i = 0; i < SLAVE_CONTROL_STEPS_PER_TICK; ++i) {
            // 子步间隔短于 FreeRTOS tick，因此用 taskYIELD 等待目标时间点。
            while (static_cast<int32_t>(micros() - next_us) < 0) {
                taskYIELD();
            }
            next_us += CONTROL_LOOP_PERIOD_US;

            // 从机控制热路径入口：读取命令快照、计算目标、更新实际角和故障位。
            runSlaveControlStep();
        }
        // 完成一批子步后让出到下一个 tick，避免永久占用 Core 1。
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

void task_safety_loop(void *pvParameters) {
    (void)pvParameters;

    while (true) {
        // UV 安全独立以 1 kHz 检查。即使控制或通信异常，也会按超时/联锁关灯。
        runSlaveSafetyStep(micros());
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void task_comm_loop(void *pvParameters) {
    (void)pvParameters;

    // seq 是从机遥测序号，主机用它拒绝旧遥测。
    uint32_t seq = 0;

    while (true) {
        // 遥测发送在 Core 0 低频任务，不进入控制热路径。
        sendSlaveTelemetry(seq++);
        vTaskDelay(pdMS_TO_TICKS(COMM_LOOP_PERIOD_MS));
    }
}

void task_status_loop(void *pvParameters) {
    (void)pvParameters;

    while (true) {
        // 串口打印限频到 STATUS_LOOP_PERIOD_MS，避免扰动无线和控制。
        printSlaveStatusLine();
        vTaskDelay(pdMS_TO_TICKS(STATUS_LOOP_PERIOD_MS));
    }
}

}  // namespace

void startSlaveTasks() {
    // Core 0 先启动通信、安全和状态任务，Core 1 最后启动控制任务。
    // 控制任务优先级最高；安全任务优先级高于状态打印。
    xTaskCreatePinnedToCore(task_comm_loop,
                            "SlaveComm",
                            SLAVE_COMM_TASK_STACK_BYTES,
                            NULL,
                            SLAVE_COMM_TASK_PRIORITY,
                            NULL,
                            SLAVE_IO_CORE);
    xTaskCreatePinnedToCore(task_safety_loop,
                            "SlaveSafety",
                            SLAVE_SAFETY_TASK_STACK_BYTES,
                            NULL,
                            SLAVE_SAFETY_TASK_PRIORITY,
                            NULL,
                            SLAVE_IO_CORE);
    xTaskCreatePinnedToCore(task_status_loop,
                            "SlaveStatus",
                            SLAVE_STATUS_TASK_STACK_BYTES,
                            NULL,
                            SLAVE_STATUS_TASK_PRIORITY,
                            NULL,
                            SLAVE_IO_CORE);
    xTaskCreatePinnedToCore(task_control_loop,
                            "SlaveControl",
                            SLAVE_CONTROL_TASK_STACK_BYTES,
                            NULL,
                            SLAVE_CONTROL_TASK_PRIORITY,
                            NULL,
                            SLAVE_CONTROL_CORE);
}
