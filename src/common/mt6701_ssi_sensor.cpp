#include "common/mt6701_ssi_sensor.h"

// MT6701 SSI 帧格式：14 bit angle + 4 bit status + 6 bit CRC。
// DO 在 CLK 上升沿后更新，下降沿后采样；硬件 SPI 默认用 SPI_MODE1 对齐该采样点。
// 当前先使用 angle/status；CRC6 后续在故障体系中统一接入。
namespace {

static constexpr float kRawToRad = _2PI / 16384.0f;
static constexpr uint32_t kAngleMask = 0x3FFFu;

}  // namespace

Mt6701SsiSensor::Mt6701SsiSensor(int cs_pin)
    : cs_pin_(cs_pin),
      spi_(&SPI),
      settings_(MT6701_SSI_CLOCK_HZ, MSBFIRST, MT6701_SSI_SPI_MODE),
      last_frame_(0),
      raw_angle_(0),
      magnetic_status_(0) {}

void Mt6701SsiSensor::init(SPIClass *spi) {
    spi_ = spi;
    settings_ = SPISettings(MT6701_SSI_CLOCK_HZ, MSBFIRST, MT6701_SSI_SPI_MODE);

    pinMode(cs_pin_, OUTPUT);
    digitalWrite(cs_pin_, HIGH);

    readFrame();
    Sensor::init();
}

uint32_t Mt6701SsiSensor::lastFrame() const {
    return last_frame_;
}

uint16_t Mt6701SsiSensor::rawAngle() const {
    return raw_angle_;
}

uint8_t Mt6701SsiSensor::magneticStatus() const {
    return magnetic_status_;
}

float Mt6701SsiSensor::getSensorAngle() {
    readFrame();
    return static_cast<float>(raw_angle_) * kRawToRad;
}

uint32_t Mt6701SsiSensor::readFrame() {
    spi_->beginTransaction(settings_);
    digitalWrite(cs_pin_, LOW);

    const uint8_t b0 = spi_->transfer(0x00);
    const uint8_t b1 = spi_->transfer(0x00);
    const uint8_t b2 = spi_->transfer(0x00);

    digitalWrite(cs_pin_, HIGH);
    spi_->endTransaction();

    last_frame_ = (static_cast<uint32_t>(b0) << 16) |
                  (static_cast<uint32_t>(b1) << 8) |
                  static_cast<uint32_t>(b2);
    raw_angle_ = static_cast<uint16_t>((last_frame_ >> 10) & kAngleMask);
    magnetic_status_ = static_cast<uint8_t>((last_frame_ >> 6) & 0x0Fu);
    return last_frame_;
}
