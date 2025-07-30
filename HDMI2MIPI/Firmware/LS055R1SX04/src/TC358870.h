#include <Wire.h>

class TC358870
{
private:
    TwoWire *i2c;
    uint8_t iaddr;

    inline void begin()
    {
        i2c->beginTransmission(iaddr);
    }

    inline void read()
    {
        i2c->endTransmission(false);
    }

    inline void end()
    {
        i2c->endTransmission();
    }

public:
    TC358870(TwoWire *i2c, uint8_t iaddr);

    inline void delay_us(uint32_t us = 200)
    {
        delayMicroseconds(us);
    }

    inline void delay_ms(uint32_t ms = 1)
    {
        delay(ms);
    }

    inline void delay_sec(uint32_t s = 1)
    {
        delay(s * 1000);
    }

    inline void address(uint16_t addr)
    {
        i2c->write(addr >> 8);
        i2c->write(addr);
    }

    inline uint8_t readU8()
    {
        i2c->requestFrom(iaddr, static_cast<size_t>(1), false);
        return i2c->read();
    }

    inline uint16_t readU16()
    {
        i2c->requestFrom(iaddr, static_cast<size_t>(2), false);
        return i2c->read() | i2c->read() << 8;
    }

    inline uint32_t readU32()
    {
        i2c->requestFrom(iaddr, static_cast<size_t>(4), false);
        return i2c->read() | i2c->read() << 8 | i2c->read() << 16 | i2c->read() << 24;
    }

    uint8_t readU8(uint16_t addr);
    uint16_t readU16(uint16_t addr);
    uint32_t readU32(uint16_t addr);

    void writeU8(uint16_t addr, uint8_t dat);
    void writeU16(uint16_t addr, uint16_t dat);
    void writeU32(uint16_t addr, uint32_t dat);

    bool init();
};