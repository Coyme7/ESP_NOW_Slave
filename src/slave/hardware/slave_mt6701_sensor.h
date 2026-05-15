#pragma once

#include <SPI.h>
#include <SimpleFOC.h>

#ifndef MT6701_SSI_CLOCK_HZ
#define MT6701_SSI_CLOCK_HZ 500000
#endif

#ifndef MT6701_SSI_TIMING_DIAG_ENABLED
#define MT6701_SSI_TIMING_DIAG_ENABLED 1
#endif

#include "common/sensors/mt6701_ssi_sensor.h"

// 从机本地 SimpleFOC 适配层。
// common 只负责 MT6701 帧解析；SPI transaction 留在从机硬件层，避免污染 common 热路径约束。
class SlaveMt6701Sensor : public Sensor {
public:
    explicit SlaveMt6701Sensor(int cs_pin);

    void init(SPIClass *spi = &SPI);
    uint32_t lastFrame() const;
    uint16_t rawAngle() const;
    uint8_t magneticStatus() const;
    uint32_t lastReadDurationUs() const;

protected:
    float getSensorAngle() override;

private:
    uint32_t readFrame();

    int cs_pin_;
    SPIClass *spi_;
    SPISettings settings_;
    Mt6701SsiFrame last_;
    uint32_t last_read_duration_us_;
};
