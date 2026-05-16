#include "slave/tasks/slave_tasks.h"

#include <Arduino.h>
#include <esp_timer.h>

#include "common/system_state.h"
#include "slave/config/slave_config.h"
#include "slave/control/slave_motion.h"
#include "slave/safety/slave_safety.h"
#include "slave/status/slave_status.h"
#include "slave/comm/slave_transport.h"

// 从机任务编排模块。
// 这里把实时控制、UV 安全、ESP-NOW 遥测和串口状态分成独立任务，
// 让控制热路径保持窄而可审查。

namespace {

TaskHandle_t controlTaskHandle = nullptr;
esp_timer_handle_t controlTimerHandle = nullptr;
volatile uint32_t controlTimerMissedTicks = 0;
volatile uint32_t controlTimerLastDtUs = 0;
volatile uint32_t controlTimerMaxDtUs = 0;
volatile uint32_t controlStepLastUs = 0;
volatile uint32_t controlStepMaxUs = 0;
volatile uint32_t controlCommandLastUs = 0;
volatile uint32_t controlCommandMaxUs = 0;
volatile uint32_t controlTrajectoryLastUs = 0;
volatile uint32_t controlTrajectoryMaxUs = 0;
volatile uint32_t controlMotorLastUs = 0;
volatile uint32_t controlMotorMaxUs = 0;
volatile uint32_t controlFocLastUs = 0;
volatile uint32_t controlFocMaxUs = 0;
volatile uint32_t controlMoveLastUs = 0;
volatile uint32_t controlMoveMaxUs = 0;
volatile uint32_t controlAngleReadLastUs = 0;
volatile uint32_t controlAngleReadMaxUs = 0;
volatile uint32_t controlStateLastUs = 0;
volatile uint32_t controlStateMaxUs = 0;
volatile uint32_t controlPublishLastUs = 0;
volatile uint32_t controlPublishMaxUs = 0;
volatile uint32_t controlFocRunCount = 0;
volatile uint32_t controlFocSkipCount = 0;

void updateMaxUs(volatile uint32_t &slot, uint32_t value) {
    if (value > slot) {
        slot = value;
    }
}

void IRAM_ATTR controlTimerCallback(void *arg) {
    (void)arg;

    TaskHandle_t handle = controlTaskHandle;
    if (handle != nullptr) {
#ifdef CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
        BaseType_t high_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(handle, &high_task_woken);
        if (high_task_woken == pdTRUE) {
            esp_timer_isr_dispatch_need_yield();
        }
#else
        xTaskNotifyGive(handle);
#endif
    }
}

bool startControlTimer() {
    if (controlTimerHandle != nullptr) {
        return true;
    }

    esp_timer_create_args_t timer_args = {};
    timer_args.callback = controlTimerCallback;
    timer_args.arg = nullptr;
#ifdef CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
    timer_args.dispatch_method = ESP_TIMER_ISR;
    const char *dispatch_name = "isr";
#else
    timer_args.dispatch_method = ESP_TIMER_TASK;
    const char *dispatch_name = "task";
#endif
    timer_args.name = "SlaveCtrl";
    timer_args.skip_unhandled_events = true;

    esp_err_t err = esp_timer_create(&timer_args, &controlTimerHandle);
    if (err != ESP_OK) {
        Serial.printf("[Slave] control_timer create failed: %s\n", esp_err_to_name(err));
        return false;
    }

    err = esp_timer_start_periodic(controlTimerHandle, SLAVE_CONTROL_TIMER_PERIOD_US);
    if (err != ESP_OK) {
        Serial.printf("[Slave] control_timer start failed: %s\n", esp_err_to_name(err));
        esp_timer_delete(controlTimerHandle);
        controlTimerHandle = nullptr;
        return false;
    }

    Serial.printf("[Slave] control_timer started period=%luus dispatch=%s\n",
                  static_cast<unsigned long>(SLAVE_CONTROL_TIMER_PERIOD_US),
                  dispatch_name);
    return true;
}

void task_control_loop(void *pvParameters) {
    (void)pvParameters;

    controlTaskHandle = xTaskGetCurrentTaskHandle();
    uint32_t previous_us = micros();

    if (!startControlTimer()) {
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    while (true) {
        // 等待 200us 控制定时器通知。任务在通知之间阻塞，让 CPU1 Idle 能运行。
        const uint32_t pending_ticks =
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SLAVE_CONTROL_TIMER_TIMEOUT_MS));
        if (pending_ticks == 0) {
            addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
            previous_us = micros();
            continue;
        }
        if (pending_ticks > 1) {
            controlTimerMissedTicks += pending_ticks - 1;
        }

        const uint32_t now_us = micros();
        controlTimerLastDtUs = now_us - previous_us;
        if (controlTimerLastDtUs > controlTimerMaxDtUs) {
            controlTimerMaxDtUs = controlTimerLastDtUs;
        }
        previous_us = now_us;

        // 从机控制热路径入口：读取命令快照、平滑目标、更新实际角和故障位。
        const uint32_t step_start_us = micros();
        SlaveControlStepTiming step_timing = {};
        runSlaveControlStep(static_cast<float>(controlTimerLastDtUs) * 0.000001f, &step_timing);
        controlStepLastUs = micros() - step_start_us;
        updateMaxUs(controlStepMaxUs, controlStepLastUs);
        controlCommandLastUs = step_timing.command_us;
        controlTrajectoryLastUs = step_timing.trajectory_us;
        controlMotorLastUs = step_timing.motor_us;
        controlFocLastUs = step_timing.foc_us;
        controlMoveLastUs = step_timing.move_us;
        controlAngleReadLastUs = step_timing.angle_read_us;
        controlStateLastUs = step_timing.state_us;
        controlPublishLastUs = step_timing.publish_us;
        updateMaxUs(controlCommandMaxUs, step_timing.command_us);
        updateMaxUs(controlTrajectoryMaxUs, step_timing.trajectory_us);
        updateMaxUs(controlMotorMaxUs, step_timing.motor_us);
        updateMaxUs(controlFocMaxUs, step_timing.foc_us);
        updateMaxUs(controlMoveMaxUs, step_timing.move_us);
        updateMaxUs(controlAngleReadMaxUs, step_timing.angle_read_us);
        updateMaxUs(controlStateMaxUs, step_timing.state_us);
        updateMaxUs(controlPublishMaxUs, step_timing.publish_us);
        if (step_timing.foc_ran != 0) {
            controlFocRunCount++;
        } else {
            controlFocSkipCount++;
        }

        const uint32_t late_ticks = ulTaskNotifyTake(pdTRUE, 0);
        if (late_ticks > 0) {
            controlTimerMissedTicks += late_ticks;
        }
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

#if SLAVE_ESPNOW_ENABLED
void task_comm_loop(void *pvParameters) {
    (void)pvParameters;

    // seq 是从机遥测序号，主机用它拒绝旧遥测。
    uint32_t seq = 0;

    while (true) {
        // 遥测发送在 Core 0 低频任务，不进入控制热路径。
        sendSlaveTelemetry(seq++);
        vTaskDelay(pdMS_TO_TICKS(SLAVE_TELEMETRY_PERIOD_MS));
    }
}
#endif

void task_status_loop(void *pvParameters) {
    (void)pvParameters;

    while (true) {
        // 串口打印限频到 STATUS_LOOP_PERIOD_MS，避免扰动无线和控制。
        printSlaveStatusLine();
        vTaskDelay(pdMS_TO_TICKS(SLAVE_STATUS_LOOP_PERIOD_MS));
    }
}

}  // namespace

void startSlaveTasks() {
    // Core 0 先启动通信、安全和状态任务，Core 1 最后启动控制任务。
    // 控制任务优先级最高；安全任务优先级高于状态打印。
#if SLAVE_ESPNOW_ENABLED
    xTaskCreatePinnedToCore(task_comm_loop,
                            "SlaveComm",
                            SLAVE_COMM_TASK_STACK_BYTES,
                            NULL,
                            SLAVE_COMM_TASK_PRIORITY,
                            NULL,
                            SLAVE_IO_CORE);
#endif
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

uint32_t getSlaveControlTimerMissedTicks() {
    return controlTimerMissedTicks;
}

uint32_t getSlaveControlLastDtUs() {
    return controlTimerLastDtUs;
}

SlaveControlHealthSnapshot getSlaveControlHealthSnapshot() {
    static uint32_t previous_missed_total = 0;
    static uint32_t previous_foc_run_count = 0;
    static uint32_t previous_foc_skip_count = 0;

    const uint32_t missed_total = controlTimerMissedTicks;
    const uint32_t foc_run_count = controlFocRunCount;
    const uint32_t foc_skip_count = controlFocSkipCount;
    SlaveControlHealthSnapshot snapshot = {};
    snapshot.last_dt_us = controlTimerLastDtUs;
    snapshot.max_dt_us = controlTimerMaxDtUs;
    snapshot.missed_delta = missed_total - previous_missed_total;
    snapshot.missed_total = missed_total;
    snapshot.step_us = controlStepLastUs;
    snapshot.step_max_us = controlStepMaxUs;
    snapshot.command_us = controlCommandLastUs;
    snapshot.command_max_us = controlCommandMaxUs;
    snapshot.trajectory_us = controlTrajectoryLastUs;
    snapshot.trajectory_max_us = controlTrajectoryMaxUs;
    snapshot.motor_us = controlMotorLastUs;
    snapshot.motor_max_us = controlMotorMaxUs;
    snapshot.foc_us = controlFocLastUs;
    snapshot.foc_max_us = controlFocMaxUs;
    snapshot.move_us = controlMoveLastUs;
    snapshot.move_max_us = controlMoveMaxUs;
    snapshot.angle_read_us = controlAngleReadLastUs;
    snapshot.angle_read_max_us = controlAngleReadMaxUs;
    snapshot.state_us = controlStateLastUs;
    snapshot.state_max_us = controlStateMaxUs;
    snapshot.publish_us = controlPublishLastUs;
    snapshot.publish_max_us = controlPublishMaxUs;
    snapshot.foc_run_delta = foc_run_count - previous_foc_run_count;
    snapshot.foc_skip_delta = foc_skip_count - previous_foc_skip_count;
    snapshot.foc_divisor =
        (SLAVE_X_FOC_EVERY_N_STEPS > 0) ? static_cast<uint32_t>(SLAVE_X_FOC_EVERY_N_STEPS) : 1UL;

    previous_missed_total = missed_total;
    previous_foc_run_count = foc_run_count;
    previous_foc_skip_count = foc_skip_count;
    controlTimerMaxDtUs = 0;
    controlStepMaxUs = 0;
    controlCommandMaxUs = 0;
    controlTrajectoryMaxUs = 0;
    controlMotorMaxUs = 0;
    controlFocMaxUs = 0;
    controlMoveMaxUs = 0;
    controlAngleReadMaxUs = 0;
    controlStateMaxUs = 0;
    controlPublishMaxUs = 0;
    return snapshot;
}
