#include <Arduino.h>
#include <unity.h>

#include "slave/hardware/slave_adc1_dma_parser.h"
#include "../../src/slave/hardware/slave_adc1_dma_parser.cpp"

namespace {

SlaveAdc1DmaParserConfig dualAxisConfig(uint8_t expected_count) {
    SlaveAdc1DmaParserConfig config = {};
    config.slot_count = kSlaveAdc1DmaMaxSlots;
    config.required_slot_mask = 0x0fU;
    config.raw_max = 4095U;
    for (uint8_t slot = 0; slot < kSlaveAdc1DmaMaxSlots; ++slot) {
        config.channel_for_slot[slot] = static_cast<uint8_t>(3U + slot);
        config.expected_count[slot] = expected_count;
    }
    return config;
}

SlaveAdc1DmaParserSample sample(uint8_t channel, uint16_t raw) {
    SlaveAdc1DmaParserSample value = {};
    value.unit = kSlaveAdc1DmaUnitAdc1;
    value.channel = channel;
    value.raw = raw;
    value.valid = true;
    return value;
}

void test_parser_averages_out_of_order_scans() {
    const SlaveAdc1DmaParserConfig config = dualAxisConfig(2U);
    const SlaveAdc1DmaParserSample samples[] = {
        sample(5U, 300U),
        sample(3U, 100U),
        sample(6U, 400U),
        sample(4U, 200U),
        sample(4U, 220U),
        sample(6U, 420U),
        sample(3U, 120U),
        sample(5U, 320U),
    };

    SlaveAdc1DmaParserResult result = {};
    TEST_ASSERT_TRUE(parseSlaveAdc1DmaSamples(config,
                                             samples,
                                             sizeof(samples) / sizeof(samples[0]),
                                             result));
    TEST_ASSERT_EQUAL_UINT16(0x0fU, result.present_slot_mask);
    TEST_ASSERT_EQUAL_UINT8(2U, result.count[0]);
    TEST_ASSERT_EQUAL_INT(110, result.average[0]);
    TEST_ASSERT_EQUAL_INT(210, result.average[1]);
    TEST_ASSERT_EQUAL_INT(310, result.average[2]);
    TEST_ASSERT_EQUAL_INT(410, result.average[3]);
}

void test_parser_rejects_missing_required_channel() {
    const SlaveAdc1DmaParserConfig config = dualAxisConfig(2U);
    const SlaveAdc1DmaParserSample samples[] = {
        sample(3U, 100U),
        sample(4U, 200U),
        sample(5U, 300U),
        sample(6U, 400U),
        sample(3U, 120U),
        sample(4U, 220U),
        sample(5U, 320U),
    };

    SlaveAdc1DmaParserResult result = {};
    TEST_ASSERT_FALSE(parseSlaveAdc1DmaSamples(config,
                                              samples,
                                              sizeof(samples) / sizeof(samples[0]),
                                              result));
    TEST_ASSERT_EQUAL_UINT8(1U, result.count[3]);
}

void test_parser_rejects_invalid_result() {
    const SlaveAdc1DmaParserConfig config = dualAxisConfig(1U);
    SlaveAdc1DmaParserSample samples[] = {
        sample(3U, 100U),
        sample(4U, 200U),
        sample(5U, 5000U),
        sample(6U, 400U),
    };

    SlaveAdc1DmaParserResult result = {};
    TEST_ASSERT_FALSE(parseSlaveAdc1DmaSamples(config,
                                              samples,
                                              sizeof(samples) / sizeof(samples[0]),
                                              result));
    TEST_ASSERT_EQUAL_UINT32(1U, result.invalid_samples);
}

void test_parser_result_is_reset_for_latest_frame() {
    const SlaveAdc1DmaParserConfig config = dualAxisConfig(1U);
    const SlaveAdc1DmaParserSample first[] = {
        sample(3U, 100U),
        sample(4U, 200U),
        sample(5U, 300U),
        sample(6U, 400U),
    };
    const SlaveAdc1DmaParserSample second[] = {
        sample(3U, 900U),
        sample(4U, 800U),
        sample(5U, 700U),
        sample(6U, 600U),
    };

    SlaveAdc1DmaParserResult result = {};
    TEST_ASSERT_TRUE(parseSlaveAdc1DmaSamples(config,
                                             first,
                                             sizeof(first) / sizeof(first[0]),
                                             result));
    TEST_ASSERT_TRUE(parseSlaveAdc1DmaSamples(config,
                                             second,
                                             sizeof(second) / sizeof(second[0]),
                                             result));
    TEST_ASSERT_EQUAL_INT(900, result.average[0]);
    TEST_ASSERT_EQUAL_INT(800, result.average[1]);
    TEST_ASSERT_EQUAL_INT(700, result.average[2]);
    TEST_ASSERT_EQUAL_INT(600, result.average[3]);
}

}  // namespace

void setUp() {}

void tearDown() {}

extern "C" void app_main() {
    initArduino();
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_parser_averages_out_of_order_scans);
    RUN_TEST(test_parser_rejects_missing_required_channel);
    RUN_TEST(test_parser_rejects_invalid_result);
    RUN_TEST(test_parser_result_is_reset_for_latest_frame);
    UNITY_END();
}
