#include "common/sensors/mt6701_ssi_sensor.h"

#include "soc/gpio_struct.h"

Mt6701SsiFastReader::Mt6701SsiFastReader(int cs_pin)
    : cs_pin_(cs_pin),
      spi_(&SPI),
      cs_bank0_mask_(0),
      cs_bank1_mask_(0),
      last_({0, 0, 0}),
      last_read_duration_us_(0) {}

void Mt6701SsiFastReader::init(SPIClass *spi) {
    spi_ = spi;
    pinMode(cs_pin_, OUTPUT);
    digitalWrite(cs_pin_, HIGH);

    const uint8_t cs_gpio = static_cast<uint8_t>(digitalPinToGPIONumber(cs_pin_));
    cs_bank0_mask_ = (cs_gpio < 32) ? (1UL << cs_gpio) : 0;
    cs_bank1_mask_ = (cs_gpio >= 32) ? (1UL << (cs_gpio - 32)) : 0;

    spi_->setFrequency(MT6701_SSI_CLOCK_HZ);
    spi_->setBitOrder(MSBFIRST);
    spi_->setDataMode(MT6701_SSI_SPI_MODE);

    readFrame();
}

Mt6701SsiFrame Mt6701SsiFastReader::readFrame() {
#if MT6701_SSI_TIMING_DIAG_ENABLED
    const uint32_t read_start_us = micros();
#endif
    setCsLow();

    uint32_t frame = 0;
    spiTransferBitsNL(spi_->bus(), 0, &frame, 24);

    setCsHigh();

    last_ = parseMt6701SsiFrame(frame);
#if MT6701_SSI_TIMING_DIAG_ENABLED
    last_read_duration_us_ = micros() - read_start_us;
#endif
    return last_;
}

uint32_t Mt6701SsiFastReader::lastFrame() const {
    return last_.frame;
}

uint16_t Mt6701SsiFastReader::rawAngle() const {
    return last_.raw_angle;
}

uint8_t Mt6701SsiFastReader::magneticStatus() const {
    return last_.magnetic_status;
}

uint32_t Mt6701SsiFastReader::lastReadDurationUs() const {
    return last_read_duration_us_;
}

void Mt6701SsiFastReader::setCsLow() const {
    if (cs_bank0_mask_ != 0) {
        GPIO.out_w1tc = cs_bank0_mask_;
    } else {
        GPIO.out1_w1tc.data = cs_bank1_mask_;
    }
}

void Mt6701SsiFastReader::setCsHigh() const {
    if (cs_bank0_mask_ != 0) {
        GPIO.out_w1ts = cs_bank0_mask_;
    } else {
        GPIO.out1_w1ts.data = cs_bank1_mask_;
    }
}

