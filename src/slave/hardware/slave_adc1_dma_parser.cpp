#include "slave/hardware/slave_adc1_dma_parser.h"

namespace {

uint8_t slotForChannel(const SlaveAdc1DmaParserConfig &config, uint8_t channel) {
    for (uint8_t slot = 0; slot < config.slot_count && slot < kSlaveAdc1DmaMaxSlots; ++slot) {
        if (config.channel_for_slot[slot] == channel) {
            return slot;
        }
    }
    return kSlaveAdc1DmaMaxSlots;
}

bool requiredSlotComplete(const SlaveAdc1DmaParserConfig &config,
                          const SlaveAdc1DmaParserResult &result,
                          uint8_t slot) {
    const uint16_t slot_mask = static_cast<uint16_t>(1U << slot);
    if ((config.required_slot_mask & slot_mask) == 0U) {
        return true;
    }
    return result.count[slot] == config.expected_count[slot] &&
           config.expected_count[slot] != 0U;
}

}  // namespace

void resetSlaveAdc1DmaParserResult(SlaveAdc1DmaParserResult &result) {
    result.present_slot_mask = 0U;
    result.invalid_samples = 0U;
    for (uint8_t i = 0; i < kSlaveAdc1DmaMaxSlots; ++i) {
        result.sum[i] = 0U;
        result.count[i] = 0U;
        result.average[i] = 0;
    }
}

bool parseSlaveAdc1DmaSamples(const SlaveAdc1DmaParserConfig &config,
                              const SlaveAdc1DmaParserSample *samples,
                              size_t sample_count,
                              SlaveAdc1DmaParserResult &result) {
    resetSlaveAdc1DmaParserResult(result);
    if (samples == nullptr || config.slot_count == 0U ||
        config.slot_count > kSlaveAdc1DmaMaxSlots) {
        result.invalid_samples++;
        return false;
    }

    for (size_t i = 0; i < sample_count; ++i) {
        const SlaveAdc1DmaParserSample &sample = samples[i];
        const uint8_t slot = slotForChannel(config, sample.channel);
        if (!sample.valid ||
            sample.unit != kSlaveAdc1DmaUnitAdc1 ||
            sample.raw > config.raw_max ||
            slot >= kSlaveAdc1DmaMaxSlots) {
            result.invalid_samples++;
            continue;
        }

        result.sum[slot] += sample.raw;
        if (result.count[slot] != UINT8_MAX) {
            result.count[slot]++;
        }
        result.present_slot_mask =
            static_cast<uint16_t>(result.present_slot_mask | (1U << slot));
    }

    if (result.invalid_samples != 0U) {
        return false;
    }

    for (uint8_t slot = 0; slot < config.slot_count; ++slot) {
        if (!requiredSlotComplete(config, result, slot)) {
            return false;
        }
        if (result.count[slot] != 0U) {
            result.average[slot] =
                static_cast<int>((result.sum[slot] + (result.count[slot] / 2U)) /
                                 result.count[slot]);
        }
    }
    return true;
}
