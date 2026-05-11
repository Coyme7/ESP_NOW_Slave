#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>

// MT6701 SSI 编码器封装。
//
// 职责：按 MT6701 的 SSI 24 bit 数据帧读取 14 bit 绝对角度，并适配 SimpleFOC Sensor 接口。
// 运行位置：可被 10 kHz FOC 路径间接调用，因此这里不能打印、分配内存或等待无线事件。
// 热路径禁忌：不做 Serial、不做 ESP-NOW、不做动态内存；只执行一次固定长度 SPI 事务。
// 未来扩展：可增加 CRC6 校验、磁场状态 fault 映射和 I2C 诊断模式。

#ifndef MT6701_SSI_CLOCK_HZ
#define MT6701_SSI_CLOCK_HZ 500000
#endif

#ifndef MT6701_SSI_SPI_MODE
#define MT6701_SSI_SPI_MODE SPI_MODE1
#endif

class Mt6701SsiSensor : public Sensor {
public:
    explicit Mt6701SsiSensor(int cs_pin);

    // 输入：已经用 SPI.begin(clk, miso, -1, cs) 配好的 SPI 总线。
    // 输出：CS 初始化为高电平，并初始化 SimpleFOC Sensor 基类缓存。
    // 故障行为：无返回值；若 DO 悬空或协议不通，后续 rawAngle() 通常保持 0/0x3fff 或跳变。
    // 10 kHz 路径：只在启动阶段调用，不在控制热路径调用。
    void init(SPIClass *spi = &SPI);

    // 最近一次读到的 24 bit SSI 原始帧，便于后续串口诊断。
    uint32_t lastFrame() const;

    // 最近一次读到的 14 bit 原始角度。
    uint16_t rawAngle() const;

    // 最近一次读到的 4 bit 磁场/按压状态。
    uint8_t magneticStatus() const;

protected:
    // 输入：无。输出：0..2pi 机械角度。
    // 故障行为：当前阶段不拦截 CRC/status，保持原始角度输出用于先跑通接线。
    // 10 kHz 路径：可调用；固定 24 个 SPI clock，不打印、不分配、不阻塞等待。
    float getSensorAngle() override;

private:
    uint32_t readFrame();

    int cs_pin_;
    SPIClass *spi_;
    SPISettings settings_;
    uint32_t last_frame_;
    uint16_t raw_angle_;
    uint8_t magnetic_status_;
};
