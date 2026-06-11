#pragma once

#include <stdint.h>

#include "slave/hardware/slave_adc1_dma_parser.h"

enum SlaveAdc1DmaSlot : uint8_t {
    SLAVE_ADC1_DMA_SLOT_X_A = 0,
    SLAVE_ADC1_DMA_SLOT_X_B = 1,
    SLAVE_ADC1_DMA_SLOT_Y_A = 2,
    SLAVE_ADC1_DMA_SLOT_Y_B = 3,
};

struct SlaveAdc1DmaFrameSnapshot {
    bool valid;
    uint32_t sequence;
    uint32_t published_us;
    uint16_t slot_mask;
    int raw[kSlaveAdc1DmaMaxSlots];
    uint8_t count[kSlaveAdc1DmaMaxSlots];
};

struct SlaveAdc1DmaHealthSnapshot {
    bool required;
    bool started;
    bool first_frame_ready;
    bool fault_latched;
    uint32_t frame_sequence;
    uint32_t invalid_frames;
    uint32_t invalid_samples;
    uint32_t read_errors;
    uint32_t read_timeouts;
    uint32_t pool_overflows;
    uint32_t stale_control_cycles;
    uint32_t consumer_last_us;
    uint32_t consumer_max_us;
    uint16_t required_slot_mask;
    uint8_t expected_count[kSlaveAdc1DmaMaxSlots];
};

bool slaveAdc1DmaSamplerRequired();
bool slaveAdc1DmaSlotForPin(int pin, SlaveAdc1DmaSlot &slot);

bool startSlaveAdc1DmaSampler();
bool waitForSlaveAdc1DmaFirstFrame(uint32_t timeout_ms);
bool latchSlaveAdc1DmaControlSnapshot();
bool slaveAdc1DmaReadControlRaw(SlaveAdc1DmaSlot slot, int &raw);
bool slaveAdc1DmaReadLatestRaw(SlaveAdc1DmaSlot slot, int &raw);
bool waitForSlaveAdc1DmaRawPair(SlaveAdc1DmaSlot slot_a,
                                SlaveAdc1DmaSlot slot_b,
                                uint32_t &last_sequence,
                                uint32_t timeout_ms,
                                int &raw_a,
                                int &raw_b);
bool slaveAdc1DmaFaultLatched();
SlaveAdc1DmaHealthSnapshot snapshotSlaveAdc1DmaHealth();
