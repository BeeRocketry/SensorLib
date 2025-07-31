#ifndef I2CLASS_H
#define I2CLASS_H

#include <Arduino.h>
#include <Wire.h>
#include "debugprinter.h"

#define TIMEOUT_I2C 1000

class I2Class
{
private:
    uint8_t minimum(uint8_t x, uint8_t y);
    TwoWire* wireI2C = nullptr;
    bool isSet = false;
    bool created = false;

public:
    bool I2CWriteByte(uint8_t chipadr, uint8_t regadr, uint8_t data);
    uint8_t I2CReadByte(uint8_t chipadr, uint8_t regadr, uint8_t* temp, uint16_t timeout = TIMEOUT_I2C);
    uint8_t I2CReadBytes(uint8_t chipadr, uint8_t regadr, uint8_t* temp, uint8_t length, uint16_t timeout = TIMEOUT_I2C);

    I2Class();
    I2Class(uint8_t sda, uint8_t scl);
    ~I2Class();
};

#endif