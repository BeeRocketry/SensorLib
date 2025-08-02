#include "LPS22HH.h"

LPS22HH::LPS22HH(I2Class* _i2c) {
    this->i2c = _i2c;
}

void LPS22HH::LPSInit(LPS_OUTPUT_DATA_RATE odrRate, LPS_EN_LPFP lpfp, LPS_BDU bdu, LPS_LOWNOISE lpsLowNoise){
    this->setCTRL_REG2(lpsLowNoise);
    this->setCTRL_REG1(odrRate, lpfp, bdu);
}

uint8_t LPS22HH::lpsReadBytes(uint8_t regadr, uint8_t* temp, uint8_t length, uint16_t timeout){
    return this->i2c->I2CReadBytes(CHIP_ADDRESS, regadr, temp, length, timeout);
}
bool LPS22HH::lpsWriteByte(uint8_t regadr, uint8_t data){
    return this->i2c->I2CWriteByte(CHIP_ADDRESS, regadr, data);
}

BaroData LPS22HH::LPSGetData() {
    BaroData tempData = {0};

    tempData.temperature = this->getTemperature();
    tempData.pressure = this->getPressure();
    tempData.altitude = this->getAltitude(tempData.pressure);

    return tempData;
}

float LPS22HH::getPressure() {
    uint8_t buffer[3] = {0};
    uint32_t press_data = 0;

    this->lpsReadBytes(PRESSURE_OUT_XL, buffer, 3, TIMEOUT_I2C);
    press_data = (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16);;
    
    return (float)(press_data / PRES_SENS);
}

float LPS22HH::getTemperature() {
    uint8_t buffer[2] = {0};
    int16_t temp_data = 0;
    
    this->lpsReadBytes(TEMP_OUT_L, buffer, 2);
    temp_data = (int16_t)(buffer[0] | ((uint16_t)buffer[1] << 8));
    return (float)(temp_data / TEMP_SENS);
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

    this->lpsReadBytes(CTRL_REG1, &temp, 1);

    temp &= (uint8_t)0b000 << 4;
    temp |= ((uint8_t)odrRate << 4);

    if(lpfp == LPS_EN_LPFP::LPS_LPFPENABLE){
        temp |= ((uint8_t)lpfp << 3);
    }
    else if(lpfp == LPS_EN_LPFP::LPS_LPFPDISABLE){
        temp &= ((uint8_t)lpfp << 3);
    }

    if(bdu == LPS_BDU::LPS_BDU_NONCONT){
        temp |= ((uint8_t)bdu << 1);
    }
    else if(bdu == LPS_BDU::LPS_BDU_CONT){
        temp &= ((uint8_t)bdu << 1);
    }

    this->lpsWriteByte(CTRL_REG1, temp);
}

void LPS22HH::setCTRL_REG2(LPS_LOWNOISE lpsLowNoise, LPS_AUTOINC autoInc) {
    uint8_t temp = 0;

    this->lpsReadBytes(CTRL_REG2, &temp, 1);

    if(lpsLowNoise == LPS_LOWNOISE::LPS_LOW_NOISE){
        temp |= ((uint8_t)lpsLowNoise << 1);
    }
    else if(lpsLowNoise == LPS_LOWNOISE::LPS_LOW_CURRENT){
        temp &= ((uint8_t)lpsLowNoise << 1);
    }

    if(autoInc == LPS_AUTOINC::LPS_AUTO_ON){
        temp |= ((uint8_t)autoInc << 4);
    }
    else if(autoInc == LPS_AUTOINC::LPS_AUTO_OFF){
        temp &= ((uint8_t)autoInc << 4);
    }
    
    this->lpsWriteByte(CTRL_REG2, temp);
}

void LPS22HH::resetLPS() {
    uint8_t temp = 0;

    temp |= (1 << 2);
    this->lpsWriteByte(CTRL_REG2, temp);
    delay(100);
}
