#include "slave/tasks/slave_tasks.h"

#include <Arduino.h>
#include <esp_timer.h>

#include "common/system_state.h"
#include "common/timing/link_timing.h"
#include "slave/config/slave_config.h"
#include "slave/control/slave_motion.h"
#include "slave/modes/mode_traits.h"
#include "slave/safety/slave_safety.h"
#include "slave/status/slave_status.h"
#include "slave/comm/slave_transport.h"
#include "slave/vofa_tuner/vofa_tuner.h"

// 从机任务编排模块。
// 这里把实时控制、UV 安全、ESP-NOW 遥测和串口状态分成独立任务，
// 让控制热路径保持窄而可审查。

namespace {

TaskHandle_t controlTaskHandle = nullptr;
esp_timer_handle_t controlTimerHandle = nullptr;
volatile uint32_t controlTimerMissedTicks = 0;
volatile uint32_t controlTimerLastDtUs = 0;
#if SLAVE_TIMING_STEP_DIAG_ENABLED
volatile uint32_t controlTimerMaxDtUs = 0;
volatile uint32_t controlStepLastUs = 0;
volatile uint32_t controlStepMaxUs = 0;
volatile uint32_t controlCommandLastUs = 0;
volatile uint32_t controlCommandMaxUs = 0;
volatile uint32_t controlTrajectoryLastUs = 0;
volatile uint32_t controlTrajectoryMaxUs = 0;
volatile uint32_t controlMotorLastUs = 0;
volatile uint32_t controlMotorMaxUs = 0;
volatile uint32_t controlCurrentSenseLastUs = 0;
volatile uint32_t controlCurrentSenseMaxUs = 0;
volatile uint32_t controlXSensorLastUs = 0;
volatile uint32_t controlXSensorMaxUs = 0;
volatile uint32_t controlXFocLastUs = 0;
volatile uint32_t controlXFocMaxUs = 0;
volatile uint32_t controlXMoveLastUs = 0;
volatile uint32_t controlXMoveMaxUs = 0;
volatile uint32_t controlYSensorLastUs = 0;
volatile uint32_t controlYSensorMaxUs = 0;
volatile uint32_t controlYFocLastUs = 0;
volatile uint32_t controlYFocMaxUs = 0;
volatile uint32_t controlYMoveLastUs = 0;
volatile uint32_t controlYMoveMaxUs = 0;
volatile uint32_t controlStateLastUs = 0;
volatile uint32_t controlStateMaxUs = 0;
volatile uint32_t controlPublishLastUs = 0;
volatile uint32_t controlPublishMaxUs = 0;
volatile uint32_t controlXFocRunCount = 0;
volatile uint32_t controlXFocSkipCount = 0;
volatile uint32_t controlYFocRunCount = 0;
volatile uint32_t controlYFocSkipCount = 0;
volatile uint32_t controlStepOverPeriodCount = 0;
volatile uint32_t controlStepOver75PctCount = 0;
volatile uint32_t controlStepOver50PctCount = 0;
volatile uint32_t controlStepOver200Count = 0;
volatile uint32_t controlStepOver300Count = 0;

void updateMaxUs(volatile uint32_t &slot, uint32_t value) {
    if (value > slot) {
        slot = value;
    }
}
#endif

constexpr bool controlRunsXAxisForTiming() {
    return SLAVE_X_MOTOR_HW_ENABLED && slaveRunModeDrivesAxis(AXIS_X);
}

constexpr bool controlRunsYAxisForTiming() {
    return SLAVE_Y_MOTOR_HW_ENABLED && slaveRunModeDrivesAxis(AXIS_Y);
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
    __attribute__((unused)) const char *dispatch_name = "isr";
#else
    timer_args.dispatch_method = ESP_TIMER_TASK;
    const char *dispatch_name = "task";
#endif
    timer_args.name = "SlaveCtrl";
    timer_args.skip_unhandled_events = true;

    esp_err_t err = esp_timer_create(&timer_args, &controlTimerHandle);
    if (err != ESP_OK) {
#if SLAVE_VOFA_TUNER_ENABLED
        Serial.printf("# [Slave] control_timer create failed: %s\n", esp_err_to_name(err));
#else
        Serial.printf("[Slave] control_timer create failed: %s\n", esp_err_to_name(err));
#endif
        return false;
    }

    err = esp_timer_start_periodic(controlTimerHandle, SLAVE_CONTROL_TIMER_PERIOD_US);
    if (err != ESP_OK) {
#if SLAVE_VOFA_TUNER_ENABLED
        Serial.printf("# [Slave] control_timer start failed: %s\n", esp_err_to_name(err));
#else
        Serial.printf("[Slave] control_timer start failed: %s\n", esp_err_to_name(err));
#endif
        esp_timer_delete(controlTimerHandle);
        controlTimerHandle = nullptr;
        return false;
    }

#if SLAVE_CONTROL_TIMER_LOG_ENABLED
    Serial.printf("[Slave] control_timer started period=%luus dispatch=%s\n",
                  static_cast<unsigned long>(SLAVE_CONTROL_TIMER_PERIOD_US),
                  dispatch_name);
#endif
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
        // 等待控制定时器通知。任务在通知之间阻塞，让 CPU1 Idle 能运行。
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
        const uint32_t control_dt_us = now_us - previous_us;
        controlTimerLastDtUs = control_dt_us;
#if SLAVE_TIMING_STEP_DIAG_ENABLED
        if (controlTimerLastDtUs > controlTimerMaxDtUs) {
            controlTimerMaxDtUs = controlTimerLastDtUs;
        }
#endif
        previous_us = now_us;

        // 从机控制热路径入口：motor tick 按 SLAVE_CONTROL_LOOP_PERIOD_US 运行；planner 按配置分频运行。
#if SLAVE_TIMING_STEP_DIAG_ENABLED
        const uint32_t step_start_us = micros();
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
        SlaveControlStepTiming step_timing = {};
        runSlaveControlStep(static_cast<float>(control_dt_us) * 0.000001f, &step_timing);
#else
        runSlaveControlStep(static_cast<float>(control_dt_us) * 0.000001f, nullptr);
#endif
        controlStepLastUs = micros() - step_start_us;
        updateMaxUs(controlStepMaxUs, controlStepLastUs);
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
        controlCommandLastUs = step_timing.command_us;
        controlTrajectoryLastUs = step_timing.trajectory_us;
        controlMotorLastUs = step_timing.motor_us;
        controlXSensorLastUs = step_timing.x_sensor_us;
        controlXFocLastUs = step_timing.x_foc_us;
        controlXMoveLastUs = step_timing.x_move_us;
        controlYSensorLastUs = step_timing.y_sensor_us;
        controlYFocLastUs = step_timing.y_foc_us;
        controlYMoveLastUs = step_timing.y_move_us;
        controlStateLastUs = step_timing.state_us;
        controlPublishLastUs = step_timing.publish_us;
        updateMaxUs(controlCommandMaxUs, step_timing.command_us);
        updateMaxUs(controlTrajectoryMaxUs, step_timing.trajectory_us);
        updateMaxUs(controlMotorMaxUs, step_timing.motor_us);
        updateMaxUs(controlXSensorMaxUs, step_timing.x_sensor_us);
        updateMaxUs(controlXFocMaxUs, step_timing.x_foc_us);
        updateMaxUs(controlXMoveMaxUs, step_timing.x_move_us);
        updateMaxUs(controlYSensorMaxUs, step_timing.y_sensor_us);
        updateMaxUs(controlYFocMaxUs, step_timing.y_foc_us);
        updateMaxUs(controlYMoveMaxUs, step_timing.y_move_us);
        updateMaxUs(controlStateMaxUs, step_timing.state_us);
        updateMaxUs(controlPublishMaxUs, step_timing.publish_us);
        if (controlRunsXAxisForTiming()) {
            if (step_timing.x_foc_ran != 0) {
                controlXFocRunCount++;
            } else {
                controlXFocSkipCount++;
            }
        }
        if (controlRunsYAxisForTiming()) {
            if (step_timing.y_foc_ran != 0) {
                controlYFocRunCount++;
            } else {
                controlYFocSkipCount++;
            }
        }
#endif
#else
        runSlaveControlStep(static_cast<float>(control_dt_us) * 0.000001f, nullptr);
#endif
#if SLAVE_TIMING_STEP_DIAG_ENABLED
        if (controlStepLastUs > SLAVE_CONTROL_LOOP_PERIOD_US) {
            controlStepOverPeriodCount++;
        }
        if (controlStepLastUs > ((SLAVE_CONTROL_LOOP_PERIOD_US * 3UL) / 4UL)) {
            controlStepOver75PctCount++;
        }
        if (controlStepLastUs > (SLAVE_CONTROL_LOOP_PERIOD_US / 2UL)) {
            controlStepOver50PctCount++;
        }
        if (controlStepLastUs > 200UL) {
            controlStepOver200Count++;
        }
        if (controlStepLastUs > 300UL) {
            controlStepOver300Count++;
        }
#endif

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
    uint32_t last_telemetry_ms = 0;

    while (true) {
        processSlaveCommand();

        const uint32_t now_ms = millis();
        if (now_ms - last_telemetry_ms >= SLAVE_TELEMETRY_PERIOD_MS) {
            // 遥测发送在 Core 0 低频任务，不进入控制热路径。
            sendSlaveTelemetry(seq++);
            last_telemetry_ms = now_ms;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
#endif

#if SLAVE_STATUS_LOG_ENABLED && !SLAVE_VOFA_TUNER_ENABLED
void task_status_loop(void *pvParameters) {
    (void)pvParameters;

    while (true) {
        processSlaveDiagShell();
        // 串口打印限频到 STATUS_LOOP_PERIOD_MS，避免扰动无线和控制。
        printSlaveStatusLine();
        vTaskDelay(pdMS_TO_TICKS(SLAVE_STATUS_LOOP_PERIOD_MS));
    }
}
#endif

#if SLAVE_VOFA_TUNER_ENABLED
void task_vofa_tuner_loop(void *pvParameters) {
    (void)pvParameters;

    while (true) {
        runSlaveVofaTunerIoStep();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
#endif

}  // namespace

extern "C" void recordSlaveTimingCurrentSenseUs(uint32_t duration_us) {
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    controlCurrentSenseLastUs = duration_us;
    updateMaxUs(controlCurrentSenseMaxUs, duration_us);
#else
    (void)duration_us;
#endif
}

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
#if SLAVE_STATUS_LOG_ENABLED && !SLAVE_VOFA_TUNER_ENABLED
    xTaskCreatePinnedToCore(task_status_loop,
                            "SlaveStatus",
                            SLAVE_STATUS_TASK_STACK_BYTES,
                            NULL,
                            SLAVE_STATUS_TASK_PRIORITY,
                            NULL,
                            SLAVE_IO_CORE);
#endif
#if SLAVE_VOFA_TUNER_ENABLED
    xTaskCreatePinnedToCore(task_vofa_tuner_loop,
                            "SlaveVofaTuner",
                            SLAVE_VOFA_TUNER_TASK_STACK_BYTES,
                            NULL,
                            SLAVE_VOFA_TUNER_TASK_PRIORITY,
                            NULL,
                            SLAVE_IO_CORE);
#endif
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
    const uint32_t missed_total = controlTimerMissedTicks;
    SlaveControlHealthSnapshot snapshot = {};
    snapshot.last_dt_us = controlTimerLastDtUs;
    snapshot.missed_delta = missed_total - previous_missed_total;
    snapshot.missed_total = missed_total;
    previous_missed_total = missed_total;

#if SLAVE_TIMING_STEP_DIAG_ENABLED
    static uint32_t previous_step_over_period_count = 0;
    static uint32_t previous_step_over_75pct_count = 0;
    static uint32_t previous_step_over_50pct_count = 0;
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    static uint32_t previous_x_foc_run_count = 0;
    static uint32_t previous_x_foc_skip_count = 0;
    static uint32_t previous_y_foc_run_count = 0;
    static uint32_t previous_y_foc_skip_count = 0;
#endif
    static uint32_t previous_step_over_200_count = 0;
    static uint32_t previous_step_over_300_count = 0;

    const uint32_t step_over_period_count = controlStepOverPeriodCount;
    const uint32_t step_over_75pct_count = controlStepOver75PctCount;
    const uint32_t step_over_50pct_count = controlStepOver50PctCount;
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t x_foc_run_count = controlXFocRunCount;
    const uint32_t x_foc_skip_count = controlXFocSkipCount;
    const uint32_t y_foc_run_count = controlYFocRunCount;
    const uint32_t y_foc_skip_count = controlYFocSkipCount;
#endif
    const uint32_t step_over_200_count = controlStepOver200Count;
    const uint32_t step_over_300_count = controlStepOver300Count;
    snapshot.max_dt_us = controlTimerMaxDtUs;
    snapshot.step_us = controlStepLastUs;
    snapshot.step_max_us = controlStepMaxUs;
    snapshot.step_over_period_delta = step_over_period_count - previous_step_over_period_count;
    snapshot.step_over_75pct_delta = step_over_75pct_count - previous_step_over_75pct_count;
    snapshot.step_over_50pct_delta = step_over_50pct_count - previous_step_over_50pct_count;
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
    snapshot.command_us = controlCommandLastUs;
    snapshot.command_max_us = controlCommandMaxUs;
    snapshot.trajectory_us = controlTrajectoryLastUs;
    snapshot.trajectory_max_us = controlTrajectoryMaxUs;
    snapshot.motor_us = controlMotorLastUs;
    snapshot.motor_max_us = controlMotorMaxUs;
    snapshot.current_sense_us = controlCurrentSenseLastUs;
    snapshot.current_sense_max_us = controlCurrentSenseMaxUs;
    snapshot.x_sensor_us = controlXSensorLastUs;
    snapshot.x_sensor_max_us = controlXSensorMaxUs;
    snapshot.x_foc_us = controlXFocLastUs;
    snapshot.x_foc_max_us = controlXFocMaxUs;
    snapshot.x_move_us = controlXMoveLastUs;
    snapshot.x_move_max_us = controlXMoveMaxUs;
    snapshot.y_sensor_us = controlYSensorLastUs;
    snapshot.y_sensor_max_us = controlYSensorMaxUs;
    snapshot.y_foc_us = controlYFocLastUs;
    snapshot.y_foc_max_us = controlYFocMaxUs;
    snapshot.y_move_us = controlYMoveLastUs;
    snapshot.y_move_max_us = controlYMoveMaxUs;
    snapshot.state_us = controlStateLastUs;
    snapshot.state_max_us = controlStateMaxUs;
    snapshot.publish_us = controlPublishLastUs;
    snapshot.publish_max_us = controlPublishMaxUs;
    snapshot.x_foc_run_delta = x_foc_run_count - previous_x_foc_run_count;
    snapshot.x_foc_skip_delta = x_foc_skip_count - previous_x_foc_skip_count;
    snapshot.x_foc_divisor =
        controlRunsXAxisForTiming()
            ? ((SLAVE_X_FOC_EVERY_N_STEPS > 0) ? static_cast<uint32_t>(SLAVE_X_FOC_EVERY_N_STEPS) : 1UL)
            : 0UL;
    snapshot.y_foc_run_delta = y_foc_run_count - previous_y_foc_run_count;
    snapshot.y_foc_skip_delta = y_foc_skip_count - previous_y_foc_skip_count;
    snapshot.y_foc_divisor =
        controlRunsYAxisForTiming()
            ? ((SLAVE_Y_FOC_EVERY_N_STEPS > 0) ? static_cast<uint32_t>(SLAVE_Y_FOC_EVERY_N_STEPS) : 1UL)
            : 0UL;
    snapshot.step_over_200_delta = step_over_200_count - previous_step_over_200_count;
    snapshot.step_over_300_delta = step_over_300_count - previous_step_over_300_count;

    previous_step_over_period_count = step_over_period_count;
    previous_step_over_75pct_count = step_over_75pct_count;
    previous_step_over_50pct_count = step_over_50pct_count;
    previous_x_foc_run_count = x_foc_run_count;
    previous_x_foc_skip_count = x_foc_skip_count;
    previous_y_foc_run_count = y_foc_run_count;
    previous_y_foc_skip_count = y_foc_skip_count;
    previous_step_over_200_count = step_over_200_count;
    previous_step_over_300_count = step_over_300_count;
    controlTimerMaxDtUs = 0;
    controlStepMaxUs = 0;
    controlCommandMaxUs = 0;
    controlTrajectoryMaxUs = 0;
    controlMotorMaxUs = 0;
    controlCurrentSenseMaxUs = 0;
    controlXSensorMaxUs = 0;
    controlXFocMaxUs = 0;
    controlXMoveMaxUs = 0;
    controlYSensorMaxUs = 0;
    controlYFocMaxUs = 0;
    controlYMoveMaxUs = 0;
    controlStateMaxUs = 0;
    controlPublishMaxUs = 0;
#else
    snapshot.step_over_200_delta = step_over_200_count - previous_step_over_200_count;
    snapshot.step_over_300_delta = step_over_300_count - previous_step_over_300_count;

    previous_step_over_period_count = step_over_period_count;
    previous_step_over_75pct_count = step_over_75pct_count;
    previous_step_over_50pct_count = step_over_50pct_count;
    previous_step_over_200_count = step_over_200_count;
    previous_step_over_300_count = step_over_300_count;
    controlTimerMaxDtUs = 0;
    controlStepMaxUs = 0;
#endif
#endif
    return snapshot;
}
