#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

// MT6701 SSI 通用帧解析和主机 fast primitive。
// common 不依赖 SimpleFOC、board、master 或 slave；SimpleFOC 适配层放在各节点 hardware 内。

#ifndef MT6701_SSI_CLOCK_HZ
#define MT6701_SSI_CLOCK_HZ 4000000
#endif

#ifndef MT6701_SSI_SPI_MODE
#define MT6701_SSI_SPI_MODE SPI_MODE1
#endif

#ifndef MT6701_SSI_TIMING_DIAG_ENABLED
#define MT6701_SSI_TIMING_DIAG_ENABLED 0
#endif

// 一帧 SSI 读取结果：原始 24-bit 帧、14-bit 角度和磁场状态。
struct Mt6701SsiFrame {
    uint32_t frame;
    uint16_t raw_angle;
    uint8_t magnetic_status;
};

// MT6701 帧解析：高 14 位为角度，低位中包含磁场状态。
inline Mt6701SsiFrame parseMt6701SsiFrame(uint32_t frame) {
    Mt6701SsiFrame parsed = {};
    parsed.frame = frame & 0x00FFFFFFu;
    parsed.raw_angle = static_cast<uint16_t>((parsed.frame >> 10) & 0x3FFFu);
    parsed.magnetic_status = static_cast<uint8_t>((parsed.frame >> 6) & 0x0Fu);
    return parsed;
}

// 快速读取器：只负责单个 CS 对应的 MT6701，不在热路径做动态分配。
class Mt6701SsiFastReader {
public:
    explicit Mt6701SsiFastReader(int cs_pin);

    void init(SPIClass *spi = &SPI);
    Mt6701SsiFrame readFrame();

    uint32_t lastFrame() const;
    uint16_t rawAngle() const;
    uint8_t magneticStatus() const;
    uint32_t lastReadDurationUs() const;

private:
    void setCsLow() const;
    void setCsHigh() const;

    int cs_pin_;
    SPIClass *spi_;
    uint32_t cs_bank0_mask_;
    uint32_t cs_bank1_mask_;
    Mt6701SsiFrame last_;
    uint32_t last_read_duration_us_;
};

