#include "TC358870.h"
#include "TC358870_REG.h"

#define READ(t)                     \
    {                               \
        begin();                    \
        address(addr);              \
        read();                     \
        uint##t##_t v = readU##t(); \
        end();                      \
        return v;                   \
    }

TC358870::TC358870(TwoWire *i2c, uint8_t iaddr)
{
    this->i2c = i2c;
    this->iaddr = iaddr;
}

uint8_t TC358870::readU8(uint16_t addr) READ(8);
uint16_t TC358870::readU16(uint16_t addr) READ(16);
uint32_t TC358870::readU32(uint16_t addr) READ(32);

void TC358870::writeU8(uint16_t addr, uint8_t dat)
{
    begin();
    address(addr);
    i2c->write(dat);
    end();
}

void TC358870::writeU16(uint16_t addr, uint16_t dat)
{
    begin();
    address(addr);
    i2c->write(dat);
    i2c->write(dat >> 8);
    end();
}

void TC358870::writeU32(uint16_t addr, uint32_t dat)
{
    begin();
    address(addr);
    i2c->write(dat);
    i2c->write(dat >> 8);
    i2c->write(dat >> 16);
    i2c->write(dat >> 24);
    end();
}

bool TC358870::init()
{
    uint16_t v = readU16(TC_ChipID);
    if (v != 0x4700)
    {
        log_e("Incorrect chip ID or revision! (0x%04x), should be (0x0047)", v);
        return false;
    }
    return true;
}