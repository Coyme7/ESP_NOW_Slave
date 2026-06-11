#include "slave/hardware/slave_adc1_dma_sampler.h"

#include <Arduino.h>
#include <board/board_pins_slave.h>
#include <esp_adc/adc_continuous.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/soc_caps.h>

#include "slave/config/slave_config.h"
#include "slave/modes/mode_traits.h"

#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
extern "C" void recordSlaveTimingCurrentSenseUs(uint32_t duration_us) __attribute__((weak));
#endif

namespace {

static constexpr uint32_t kSlaveAdc1DmaSampleRateHz = 80000UL;
static constexpr uint32_t kSlaveAdc1DmaFrameBytes = 80UL;
static constexpr uint32_t kSlaveAdc1DmaPoolFrames = 8UL;
static constexpr uint32_t kSlaveAdc1DmaReadNotifyBit = 1UL << 0;
static constexpr uint32_t kSlaveAdc1DmaOverflowNotifyBit = 1UL << 1;
static constexpr uint32_t kSlaveAdc1DmaReadTimeoutMs = 0UL;

TaskHandle_t adcConsumerTaskHandle = nullptr;
adc_continuous_handle_t adcContinuousHandle = nullptr;
adc_digi_pattern_config_t adcPattern[kSlaveAdc1DmaMaxSlots] = {};
SlaveAdc1DmaParserConfig parserConfig = {};
SlaveAdc1DmaFrameSnapshot publishedFrames[2] = {};
SlaveAdc1DmaFrameSnapshot controlFrame = {};
alignas(4) uint8_t adcReadBuffer[kSlaveAdc1DmaFrameBytes] = {};
volatile uint8_t publishedFrameIndex = 0;
volatile bool adcStarted = false;
volatile bool adcFirstFrameReady = false;
volatile bool adcFaultLatched = false;
volatile uint32_t invalidFrames = 0;
volatile uint32_t invalidSamples = 0;
volatile uint32_t readErrors = 0;
volatile uint32_t readTimeouts = 0;
volatile uint32_t poolOverflows = 0;
volatile uint32_t staleControlCycles = 0;
volatile uint32_t consumerLastUs = 0;
volatile uint32_t consumerMaxUs = 0;

constexpr bool adcDmaNeedsXAxis() {
    return SLAVE_ENABLE_CURRENT_SENSE &&
           slaveRunModeNeedsMotorHardware(AXIS_X);
}

constexpr bool adcDmaNeedsYAxis() {
    return SLAVE_ENABLE_CURRENT_SENSE &&
           slaveRunModeNeedsMotorHardware(AXIS_Y);
}

uint8_t expectedSamplesPerActiveChannel() {
    return (adcDmaNeedsXAxis() && adcDmaNeedsYAxis()) ? 5U : 10U;
}

bool slotActive(SlaveAdc1DmaSlot slot) {
    switch (slot) {
        case SLAVE_ADC1_DMA_SLOT_X_A:
        case SLAVE_ADC1_DMA_SLOT_X_B:
            return adcDmaNeedsXAxis();
        case SLAVE_ADC1_DMA_SLOT_Y_A:
        case SLAVE_ADC1_DMA_SLOT_Y_B:
            return adcDmaNeedsYAxis();
        default:
            return false;
    }
}

bool slotPin(SlaveAdc1DmaSlot slot, int &pin) {
    switch (slot) {
        case SLAVE_ADC1_DMA_SLOT_X_A:
            pin = board_pins_slave::MOTOR1_CURRENT_A_X;
            return true;
        case SLAVE_ADC1_DMA_SLOT_X_B:
            pin = board_pins_slave::MOTOR1_CURRENT_B_X;
            return true;
        case SLAVE_ADC1_DMA_SLOT_Y_A:
            pin = board_pins_slave::MOTOR2_CURRENT_A_Y;
            return true;
        case SLAVE_ADC1_DMA_SLOT_Y_B:
            pin = board_pins_slave::MOTOR2_CURRENT_B_Y;
            return true;
        default:
            return false;
    }
}

void publishParsedFrame(const SlaveAdc1DmaParserResult &result) {
    const uint8_t inactive_index = static_cast<uint8_t>(publishedFrameIndex ^ 1U);
    SlaveAdc1DmaFrameSnapshot &frame = publishedFrames[inactive_index];
    frame.valid = true;
    frame.sequence = publishedFrames[publishedFrameIndex].sequence + 1U;
    if (frame.sequence == 0U) {
        frame.sequence = 1U;
    }
    frame.published_us = micros();
    frame.slot_mask = result.present_slot_mask;
    for (uint8_t slot = 0; slot < kSlaveAdc1DmaMaxSlots; ++slot) {
        frame.raw[slot] = result.average[slot];
        frame.count[slot] = result.count[slot];
    }
    publishedFrameIndex = inactive_index;
    adcFirstFrameReady = true;
}

SlaveAdc1DmaFrameSnapshot latestPublishedFrame() {
    return publishedFrames[publishedFrameIndex];
}

bool readRawFromFrame(const SlaveAdc1DmaFrameSnapshot &frame,
                      SlaveAdc1DmaSlot slot,
                      int &raw) {
    const uint8_t slot_index = static_cast<uint8_t>(slot);
    if (!frame.valid || slot_index >= kSlaveAdc1DmaMaxSlots ||
        (frame.slot_mask & (1U << slot_index)) == 0U) {
        return false;
    }
    raw = frame.raw[slot_index];
    return true;
}

bool configureParserAndPattern() {
    parserConfig = {};
    parserConfig.slot_count = kSlaveAdc1DmaMaxSlots;
    parserConfig.raw_max =
        static_cast<uint16_t>(kSlaveCurrentSenseHardware.adc_raw_max);
    for (uint8_t slot = 0; slot < kSlaveAdc1DmaMaxSlots; ++slot) {
        parserConfig.channel_for_slot[slot] = kSlaveAdc1DmaInvalidChannel;
        parserConfig.expected_count[slot] = 0U;
    }

    uint8_t pattern_count = 0U;
    const uint8_t expected_count = expectedSamplesPerActiveChannel();
    for (uint8_t slot = 0; slot < kSlaveAdc1DmaMaxSlots; ++slot) {
        const SlaveAdc1DmaSlot dma_slot = static_cast<SlaveAdc1DmaSlot>(slot);
        if (!slotActive(dma_slot)) {
            continue;
        }

        int pin = -1;
        if (!slotPin(dma_slot, pin)) {
            return false;
        }

        adc_unit_t unit = ADC_UNIT_1;
        adc_channel_t channel = ADC_CHANNEL_0;
        if (adc_continuous_io_to_channel(pin, &unit, &channel) != ESP_OK ||
            unit != ADC_UNIT_1) {
            return false;
        }

        parserConfig.channel_for_slot[slot] = static_cast<uint8_t>(channel);
        parserConfig.expected_count[slot] = expected_count;
        parserConfig.required_slot_mask =
            static_cast<uint16_t>(parserConfig.required_slot_mask | (1U << slot));

        adcPattern[pattern_count].atten = ADC_ATTEN_DB_12;
        adcPattern[pattern_count].channel = static_cast<uint8_t>(channel);
        adcPattern[pattern_count].unit = ADC_UNIT_1;
        adcPattern[pattern_count].bit_width = ADC_BITWIDTH_12;
        pattern_count++;
    }
    return pattern_count == 2U || pattern_count == 4U;
}

bool parseReadBuffer(uint32_t out_length) {
    if (out_length != kSlaveAdc1DmaFrameBytes ||
        (out_length % SOC_ADC_DIGI_RESULT_BYTES) != 0U) {
        invalidFrames = invalidFrames + 1U;
        return false;
    }

    const size_t sample_count = out_length / SOC_ADC_DIGI_RESULT_BYTES;
    SlaveAdc1DmaParserSample samples[kSlaveAdc1DmaFrameBytes / SOC_ADC_DIGI_RESULT_BYTES] = {};
    for (size_t i = 0; i < sample_count; ++i) {
        const adc_digi_output_data_t *data =
            reinterpret_cast<const adc_digi_output_data_t *>(
                &adcReadBuffer[i * SOC_ADC_DIGI_RESULT_BYTES]);
        samples[i].unit = static_cast<uint8_t>(data->type2.unit);
        samples[i].channel = static_cast<uint8_t>(data->type2.channel);
        samples[i].raw = static_cast<uint16_t>(data->type2.data);
        samples[i].valid =
            data->type2.channel < SOC_ADC_CHANNEL_NUM(0) &&
            data->type2.data <= kSlaveCurrentSenseHardware.adc_raw_max;
    }

    SlaveAdc1DmaParserResult result = {};
    if (!parseSlaveAdc1DmaSamples(parserConfig, samples, sample_count, result)) {
        invalidFrames = invalidFrames + 1U;
        invalidSamples = invalidSamples + result.invalid_samples;
        return false;
    }
    publishParsedFrame(result);
    return true;
}

bool IRAM_ATTR onAdcFrameDone(adc_continuous_handle_t handle,
                              const adc_continuous_evt_data_t *edata,
                              void *user_data) {
    (void)handle;
    (void)edata;
    (void)user_data;

    BaseType_t high_task_woken = pdFALSE;
    TaskHandle_t task = adcConsumerTaskHandle;
    if (task != nullptr) {
        xTaskNotifyFromISR(task,
                           kSlaveAdc1DmaReadNotifyBit,
                           eSetBits,
                           &high_task_woken);
    }
    return high_task_woken == pdTRUE;
}

bool IRAM_ATTR onAdcPoolOverflow(adc_continuous_handle_t handle,
                                 const adc_continuous_evt_data_t *edata,
                                 void *user_data) {
    (void)handle;
    (void)edata;
    (void)user_data;

    BaseType_t high_task_woken = pdFALSE;
    TaskHandle_t task = adcConsumerTaskHandle;
    if (task != nullptr) {
        xTaskNotifyFromISR(task,
                           kSlaveAdc1DmaOverflowNotifyBit,
                           eSetBits,
                           &high_task_woken);
    }
    return high_task_woken == pdTRUE;
}

void taskAdcDmaConsumer(void *pvParameters) {
    (void)pvParameters;

    while (true) {
        uint32_t notified = 0;
        xTaskNotifyWait(0U, UINT32_MAX, &notified, portMAX_DELAY);
        if ((notified & kSlaveAdc1DmaOverflowNotifyBit) != 0U) {
            poolOverflows = poolOverflows + 1U;
        }

        while (adcContinuousHandle != nullptr) {
            const uint32_t start_us = micros();
            uint32_t out_length = 0U;
            const esp_err_t err = adc_continuous_read(adcContinuousHandle,
                                                      adcReadBuffer,
                                                      sizeof(adcReadBuffer),
                                                      &out_length,
                                                      kSlaveAdc1DmaReadTimeoutMs);
            if (err == ESP_ERR_TIMEOUT) {
                readTimeouts = readTimeouts + 1U;
                break;
            }
            if (err != ESP_OK) {
                readErrors = readErrors + 1U;
                break;
            }
            (void)parseReadBuffer(out_length);
            const uint32_t elapsed_us = micros() - start_us;
            consumerLastUs = elapsed_us;
            if (elapsed_us > consumerMaxUs) {
                consumerMaxUs = elapsed_us;
            }
#if SLAVE_TIMING_DETAIL_DIAG_ENABLED
            if (recordSlaveTimingCurrentSenseUs) {
                recordSlaveTimingCurrentSenseUs(elapsed_us);
            }
#endif
        }
    }
}

}  // namespace

bool slaveAdc1DmaSamplerRequired() {
    return adcDmaNeedsXAxis() || adcDmaNeedsYAxis();
}

bool slaveAdc1DmaSlotForPin(int pin, SlaveAdc1DmaSlot &slot) {
    for (uint8_t i = 0; i < kSlaveAdc1DmaMaxSlots; ++i) {
        int candidate_pin = -1;
        if (slotPin(static_cast<SlaveAdc1DmaSlot>(i), candidate_pin) &&
            candidate_pin == pin) {
            slot = static_cast<SlaveAdc1DmaSlot>(i);
            return true;
        }
    }
    return false;
}

bool startSlaveAdc1DmaSampler() {
    if (!slaveAdc1DmaSamplerRequired()) {
        return true;
    }
    if (adcStarted) {
        return true;
    }
    if (!configureParserAndPattern()) {
        adcFaultLatched = true;
        return false;
    }

    if (adcConsumerTaskHandle == nullptr) {
        const BaseType_t created =
            xTaskCreatePinnedToCore(taskAdcDmaConsumer,
                                    "SlaveAdcDma",
                                    SLAVE_ADC_DMA_TASK_STACK_BYTES,
                                    nullptr,
                                    SLAVE_ADC_DMA_TASK_PRIORITY,
                                    &adcConsumerTaskHandle,
                                    SLAVE_CONTROL_CORE);
        if (created != pdPASS) {
            adcFaultLatched = true;
            return false;
        }
    }

    adc_continuous_handle_cfg_t handle_config = {};
    handle_config.max_store_buf_size =
        kSlaveAdc1DmaFrameBytes * kSlaveAdc1DmaPoolFrames;
    handle_config.conv_frame_size = kSlaveAdc1DmaFrameBytes;
    handle_config.flags.flush_pool = 1U;
    if (adc_continuous_new_handle(&handle_config, &adcContinuousHandle) != ESP_OK) {
        adcFaultLatched = true;
        return false;
    }

    adc_continuous_config_t adc_config = {};
    adc_config.pattern_num =
        (adcDmaNeedsXAxis() && adcDmaNeedsYAxis()) ? 4U : 2U;
    adc_config.adc_pattern = adcPattern;
    adc_config.sample_freq_hz = kSlaveAdc1DmaSampleRateHz;
    adc_config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    adc_config.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    if (adc_continuous_config(adcContinuousHandle, &adc_config) != ESP_OK) {
        adcFaultLatched = true;
        return false;
    }

    adc_continuous_evt_cbs_t callbacks = {};
    callbacks.on_conv_done = onAdcFrameDone;
    callbacks.on_pool_ovf = onAdcPoolOverflow;
    if (adc_continuous_register_event_callbacks(adcContinuousHandle,
                                                &callbacks,
                                                nullptr) != ESP_OK) {
        adcFaultLatched = true;
        return false;
    }

    if (adc_continuous_start(adcContinuousHandle) != ESP_OK) {
        adcFaultLatched = true;
        return false;
    }
    adcStarted = true;
    return true;
}

bool waitForSlaveAdc1DmaFirstFrame(uint32_t timeout_ms) {
    if (!slaveAdc1DmaSamplerRequired()) {
        return true;
    }
    const uint32_t start_ms = millis();
    while (!adcFirstFrameReady && !adcFaultLatched) {
        if ((millis() - start_ms) >= timeout_ms) {
            adcFaultLatched = true;
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return adcFirstFrameReady && !adcFaultLatched;
}

bool latchSlaveAdc1DmaControlSnapshot() {
    if (!slaveAdc1DmaSamplerRequired()) {
        return true;
    }
    const SlaveAdc1DmaFrameSnapshot latest = latestPublishedFrame();
    if (!latest.valid) {
        if (staleControlCycles != UINT32_MAX) {
            staleControlCycles = staleControlCycles + 1U;
        }
    } else if (latest.sequence != controlFrame.sequence) {
        controlFrame = latest;
        staleControlCycles = 0U;
    } else if (staleControlCycles != UINT32_MAX) {
        staleControlCycles = staleControlCycles + 1U;
    }

    if (staleControlCycles >= kSlaveCurrentSenseAdcConsecutiveErrorLimit) {
        adcFaultLatched = true;
    }
    return !adcFaultLatched;
}

bool slaveAdc1DmaReadControlRaw(SlaveAdc1DmaSlot slot, int &raw) {
    return readRawFromFrame(controlFrame, slot, raw);
}

bool slaveAdc1DmaReadLatestRaw(SlaveAdc1DmaSlot slot, int &raw) {
    const SlaveAdc1DmaFrameSnapshot latest = latestPublishedFrame();
    return readRawFromFrame(latest, slot, raw);
}

bool waitForSlaveAdc1DmaRawPair(SlaveAdc1DmaSlot slot_a,
                                SlaveAdc1DmaSlot slot_b,
                                uint32_t &last_sequence,
                                uint32_t timeout_ms,
                                int &raw_a,
                                int &raw_b) {
    if (!slaveAdc1DmaSamplerRequired()) {
        return false;
    }
    const uint32_t start_ms = millis();
    while (!adcFaultLatched) {
        const SlaveAdc1DmaFrameSnapshot latest = latestPublishedFrame();
        if (latest.valid && latest.sequence != last_sequence &&
            readRawFromFrame(latest, slot_a, raw_a) &&
            readRawFromFrame(latest, slot_b, raw_b)) {
            last_sequence = latest.sequence;
            return true;
        }
        if ((millis() - start_ms) >= timeout_ms) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

bool slaveAdc1DmaFaultLatched() {
    return adcFaultLatched;
}

SlaveAdc1DmaHealthSnapshot snapshotSlaveAdc1DmaHealth() {
    SlaveAdc1DmaHealthSnapshot snapshot = {};
    snapshot.required = slaveAdc1DmaSamplerRequired();
    snapshot.started = adcStarted;
    snapshot.first_frame_ready = adcFirstFrameReady;
    snapshot.fault_latched = adcFaultLatched;
    snapshot.frame_sequence = publishedFrames[publishedFrameIndex].sequence;
    snapshot.invalid_frames = invalidFrames;
    snapshot.invalid_samples = invalidSamples;
    snapshot.read_errors = readErrors;
    snapshot.read_timeouts = readTimeouts;
    snapshot.pool_overflows = poolOverflows;
    snapshot.stale_control_cycles = staleControlCycles;
    snapshot.consumer_last_us = consumerLastUs;
    snapshot.consumer_max_us = consumerMaxUs;
    snapshot.required_slot_mask = parserConfig.required_slot_mask;
    for (uint8_t slot = 0; slot < kSlaveAdc1DmaMaxSlots; ++slot) {
        snapshot.expected_count[slot] = parserConfig.expected_count[slot];
    }
    return snapshot;
}
