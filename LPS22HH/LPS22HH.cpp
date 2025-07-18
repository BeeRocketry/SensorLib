#include "LPS22HH.h"

LPS22HH::LPS22HH(I2Class* _i2c) {
    this->i2c = _i2c;
}

uint8_t LPS22HH::lpsReadBytes(uint8_t regadr, uint8_t* temp, uint8_t length, uint16_t timeout){
    return this->i2c->I2CReadBytes(CHIP_ADDRESS, regadr, temp, length, timeout);
}
bool LPS22HH::lpsWriteByte(uint8_t regadr, uint8_t data){
    return this->i2c->I2CWriteByte(CHIP_ADDRESS, regadr, data);
}

float LPS22HH::getPressure() {
    uint8_t buffer[3] = {0};
    uint32_t press_data = 0;

    this->lpsReadBytes(PRESSURE_OUT_XL, buffer, 3, TIMEOUT_I2C);
    press_data = buffer[0] | ((uint32_t) buffer[1] << 8) | ((uint32_t) buffer[2] << 16);
    
    return (float)press_data/PRES_SENS;
}

float LPS22HH::getTemperature() {
    uint8_t buffer[2] = {0};
    int16_t temp_data = 0;
    
    this->lpsReadBytes(TEMP_OUT_L, buffer, 2);
    temp_data = buffer[0] | (uint16_t) buffer[1] << 8;
    return (float)temp_data/TEMP_SENS;
}

float LPS22HH::getAltitude(float pressure){
    double Tb = 288.15;
    double Lb = 0.0065;
    double Pb = SeaLevelhPa * 100;
    double exp = 1.0 / 5.255;
    double fac = Tb / Lb;

    float altitude = fac * (1 - pow((float)(pressure / Pb), (float)exp));

    return altitude;
}

bool LPS22HH::readWhoAmI() {
    uint8_t buffer = 0;
    this->lpsReadBytes(WHO_AM_I, &buffer, 1);
    return buffer == 0b10110011;
}

void LPS22HH::setCTRL_REG1(LPS_OUTPUT_DATA_RATE odrRate, LPS_EN_LPFP lpfp, LPS_BDU bdu) {
    uint8_t temp = 0;

    temp |= ((uint8_t)lpfp << 3) | (uint8_t)odrRate << 4 | (uint8_t)bdu << 1;
    this->lpsWriteByte(CTRL_REG1, temp);
}

void LPS22HH::setCTRL_REG2(LPS_LOW_NOISE lpsLowNoise) {
    uint8_t temp = 0;
    
    temp |= ((uint8_t)lpsLowNoise << 1);
    this->lpsWriteByte(CTRL_REG2, temp);
}

void LPS22HH::resetLPS() {
    uint8_t temp = 0;

    temp |= (1 << 2);
    this->lpsWriteByte(CTRL_REG2, temp);
    delay(100);
}
