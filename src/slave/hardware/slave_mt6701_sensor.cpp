#include "slave/hardware/slave_mt6701_sensor.h"

#include <Arduino.h>

SlaveMt6701Sensor::SlaveMt6701Sensor(int cs_pin)
    : cs_pin_(cs_pin),
      spi_(&SPI),
      settings_(MT6701_SSI_CLOCK_HZ, MSBFIRST, MT6701_SSI_SPI_MODE),
      last_({0, 0, 0}),
      last_read_duration_us_(0) {}

void SlaveMt6701Sensor::init(SPIClass *spi) {
    spi_ = spi;
    settings_ = SPISettings(MT6701_SSI_CLOCK_HZ, MSBFIRST, MT6701_SSI_SPI_MODE);

    pinMode(cs_pin_, OUTPUT);
    digitalWrite(cs_pin_, HIGH);

    readFrame();
    Sensor::init();
}

uint32_t SlaveMt6701Sensor::lastFrame() const {
    return last_.frame;
}

uint16_t SlaveMt6701Sensor::rawAngle() const {
    return last_.raw_angle;
}

uint8_t SlaveMt6701Sensor::magneticStatus() const {
    return last_.magnetic_status;
}

uint32_t SlaveMt6701Sensor::lastReadDurationUs() const {
    return last_read_duration_us_;
}

float SlaveMt6701Sensor::getSensorAngle() {
    readFrame();
    return static_cast<float>(last_.raw_angle) * (_2PI / 16384.0f);
}

uint32_t SlaveMt6701Sensor::readFrame() {
#if MT6701_SSI_TIMING_DIAG_ENABLED
    const uint32_t read_start_us = micros();
#endif
    spi_->beginTransaction(settings_);
    digitalWrite(cs_pin_, LOW);

    const uint8_t b0 = spi_->transfer(0x00);
    const uint8_t b1 = spi_->transfer(0x00);
    const uint8_t b2 = spi_->transfer(0x00);

    digitalWrite(cs_pin_, HIGH);
    spi_->endTransaction();

    const uint32_t frame = (static_cast<uint32_t>(b0) << 16) |
                           (static_cast<uint32_t>(b1) << 8) |
                           static_cast<uint32_t>(b2);
    last_ = parseMt6701SsiFrame(frame);
#if MT6701_SSI_TIMING_DIAG_ENABLED
    last_read_duration_us_ = micros() - read_start_us;
#endif
    return last_.frame;
}

