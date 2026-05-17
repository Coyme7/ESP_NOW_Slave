#include "slave/hardware/slave_mt6701_sensor.h"

#include <Arduino.h>

SlaveMt6701Sensor::SlaveMt6701Sensor(int cs_pin) : reader_(cs_pin) {}

void SlaveMt6701Sensor::init(SPIClass *spi) {
    reader_.init(spi);
    Sensor::init();
}

uint32_t SlaveMt6701Sensor::lastFrame() const {
    return reader_.lastFrame();
}

uint16_t SlaveMt6701Sensor::rawAngle() const {
    return reader_.rawAngle();
}

uint8_t SlaveMt6701Sensor::magneticStatus() const {
    return reader_.magneticStatus();
}

uint32_t SlaveMt6701Sensor::lastReadDurationUs() const {
    return reader_.lastReadDurationUs();
}

float SlaveMt6701Sensor::getSensorAngle() {
    const Mt6701SsiFrame frame = reader_.readFrame();
    return static_cast<float>(frame.raw_angle) * (_2PI / 16384.0f);
}
