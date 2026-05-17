#pragma once

#include <SPI.h>
#include <SimpleFOC.h>

#include "slave/config/slave_log_config.h"

#ifndef MT6701_SSI_TIMING_DIAG_ENABLED
#define MT6701_SSI_TIMING_DIAG_ENABLED SLAVE_TIMING_DETAIL_DIAG_ENABLED
#endif

#include "common/sensors/mt6701_ssi_sensor.h"

// 从机本地 SimpleFOC 适配层。
// 内部复用 common 的 MT6701 fast reader，控制热路径禁止 digitalWrite、SPI transaction 和三次 transfer 慢路径。
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
    Mt6701SsiFastReader reader_;
};
