#pragma once

#include <stddef.h>
#include <stdint.h>

static constexpr uint8_t kSlaveAdc1DmaMaxSlots = 4U;
static constexpr uint8_t kSlaveAdc1DmaInvalidChannel = 0xffU;
static constexpr uint8_t kSlaveAdc1DmaUnitAdc1 = 0U;

struct SlaveAdc1DmaParserConfig {
    uint8_t slot_count;
    uint8_t channel_for_slot[kSlaveAdc1DmaMaxSlots];
    uint8_t expected_count[kSlaveAdc1DmaMaxSlots];
    uint16_t required_slot_mask;
    uint16_t raw_max;
};

struct SlaveAdc1DmaParserSample {
    uint8_t unit;
    uint8_t channel;
    uint16_t raw;
    bool valid;
};

struct SlaveAdc1DmaParserResult {
    uint16_t present_slot_mask;
    uint32_t invalid_samples;
    uint32_t sum[kSlaveAdc1DmaMaxSlots];
    uint8_t count[kSlaveAdc1DmaMaxSlots];
    int average[kSlaveAdc1DmaMaxSlots];
};

void resetSlaveAdc1DmaParserResult(SlaveAdc1DmaParserResult &result);

bool parseSlaveAdc1DmaSamples(const SlaveAdc1DmaParserConfig &config,
                              const SlaveAdc1DmaParserSample *samples,
                              size_t sample_count,
                              SlaveAdc1DmaParserResult &result);
