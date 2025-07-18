#include <bno055.h>

Bno055::Bno055(I2Class* _i2c) : _i2c(_i2c){}

bool Bno055::bnoWriteByte(uint8_t regadr, uint8_t data){
    return this->_i2c->I2CWriteByte(BNO_I2C_Adr, regadr, data);
}

uint8_t Bno055::bnoReadByte(uint8_t regadr, uint8_t* temp, uint16_t timeout){
    return this->_i2c->I2CReadByte(BNO_I2C_Adr, regadr, temp, timeout);
}

uint8_t Bno055::bnoReadBytes(uint8_t regadr, uint8_t* temp, uint8_t length, uint16_t timeout){
    return this->_i2c->I2CReadBytes(BNO_I2C_Adr, regadr, temp, length, timeout);
}

void Bno055::setOperationMode(){
    uint8_t reg = 0;

    reg |= this->bnoReg.registersPage_0.operationMode.operationMode;

    bnoWriteByte(BNO_REG_OPERATIONMODE, reg);
}

void Bno055::setUnits(){
    uint8_t reg = 0;
    BNO_STR_UNITSELECT tempStruct;
    tempStruct = this->bnoReg.registersPage_0.unitSelection;

    reg |= (tempStruct.dataUnit << 7) | (tempStruct.tempUnit << 4) | (tempStruct.eulerUnit << 2) | (tempStruct.gyroUnit << 1) | (tempStruct.accUnit);

    bnoWriteByte(BNO_REG_UNITSELECT, reg);
}

void Bno055::setPowerMode(){
    uint8_t reg = 0;

    reg |= (this->bnoReg.registersPage_0.powerMode.powerMode);

    bnoWriteByte(BNO_REG_POWERMODE, reg);
}

void Bno055::setAccConfig(){
    uint8_t reg = 0;
    BNO_STR_ACCCONFIG tempStruct;
    tempStruct = this->bnoReg.registersPage_1.accConfig;

    reg |= (tempStruct.accOperation << 5) | (tempStruct.accBandwidth << 2) | tempStruct.accScale;

    bnoWriteByte(BNO_REG_ACCEL_CONFIG, reg);
}

void Bno055::setGyroConfig(){
    uint8_t reg = 0;
    BNO_STR_GYRO_CONFIG_0 tempStruct_0;
    BNO_STR_GYRO_CONFIG_1 tempStruct_1;
    tempStruct_0 = this->bnoReg.registersPage_1.gyroConfig;
    tempStruct_1 = this->bnoReg.registersPage_1.gyroConfig_1;

    reg |= (tempStruct_0.gyroBandwidth << 3) | (tempStruct_0.gyroScale);

    bnoWriteByte(BNO_REG_GYRO_CONFIG_0, reg);

    reg = 0;
    
    delay(5);

    reg |= (tempStruct_1.gyroOperation);

    bnoWriteByte(BNO_REG_GYRO_CONFIG_1, reg);
}

void Bno055::setMagConfig(){
    uint8_t reg = 0;
    BNO_STR_MAG_CONFIG tempStruct;
    tempStruct = this->bnoReg.registersPage_1.magConfig;

    reg |= (tempStruct.magPower << 5) | (tempStruct.magOperation << 3) | (tempStruct.magOutputRate);

    bnoWriteByte(BNO_REG_MAG_CONFIG, reg);
}

void Bno055::setIntEnable(){
    uint8_t reg = 0;
    BNO_STR_INT_EN tempStruct;
    tempStruct = this->bnoReg.registersPage_1.intEnable;

    reg |= (tempStruct.accNoMotion << 7) | (tempStruct.accAnyMotion << 6) | (tempStruct.accHighG << 5)
           | (tempStruct.gyroDataReady << 4) | (tempStruct.gyroHighRate << 3) | (tempStruct.gyroAnyMotion << 2)
           | (tempStruct.magDataReady << 1) | (tempStruct.accDataReady);

    bnoWriteByte(BNO_REG_INT_EN, reg);
}

void Bno055::setAccIntSettings(){
    uint8_t reg = 0;
    BNO_STR_ACC_INT_SETTINGS tempStruct;
    tempStruct = this->bnoReg.registersPage_1.accIntSettings;

    reg |= (tempStruct.HighG_X << 7) | (tempStruct.HighG_Y << 6) | (tempStruct.HighG_Z << 5)
            | (tempStruct.AnyMotion_X << 4) | (tempStruct.AnyMotion_Y << 3) | (tempStruct.AnyMotion_Z << 2)
            | (tempStruct.AnyMotionDuration);

    bnoWriteByte(BNO_REG_ACC_INT_SETTINGS, reg);
}

void Bno055::BnoReset(){
    uint8_t reg = 0;
    
    reg |= 0b00100000;
    bnoWriteByte(BNO_REG_SYSTRIGGER, reg);
    delay(10);
}

void Bno055::BnoSetExternalClock(){
    uint8_t reg = 0;

    reg |= 0b10000000;
    bnoWriteByte(BNO_REG_SYSTRIGGER, reg);
}

void Bno055::changePage(uint8_t page){
    bnoWriteByte(BNO_REG_PAGE_CHANGE, page);
}

BNO_Calib_Status Bno055::getCalibrationStatus(){
    uint8_t reg = 0;
    BNO_Calib_Status tempStat;

    bnoReadByte(BNO_REG_CALIB_STATUS, &reg, TIMEOUT_I2C);
    
    tempStat.systemCalib = (reg & 0b11000000) >> 6;
    tempStat.gyroCalib = (reg & 0b00110000) >> 4;
    tempStat.accCalib = (reg & 0b00001100) >> 2;
    tempStat.magCalib = (reg & 0b00000011);

    return tempStat;
}

BNO_DOF3_Float Bno055::getAccData(){
    BNO_DOF3_int16 accData;
    BNO_DOF3_Float returnData;
    uint8_t buffer[6];

    bnoReadBytes(BNO_REG_ACC_DATA_X_LSB, buffer, 6, TIMEOUT_I2C);

    accData.x = ((int16_t)buffer[1] << 8) | buffer[0];
    accData.y = ((int16_t)buffer[3] << 8) | buffer[2];
    accData.z = ((int16_t)buffer[5] << 8) | buffer[4];

    returnData.x = (float)accData.x / this->accResolution;
    returnData.y = (float)accData.y / this->accResolution;
    returnData.z = (float)accData.z / this->accResolution;

    return returnData;
}

BNO_DOF3_int16 Bno055::getRawGyroData(){
    uint8_t buffer[6];
    BNO_DOF3_int16 gyroData;

    bnoReadBytes(BNO_REG_GYRO_DATA_X_LSB, buffer, 6, TIMEOUT_I2C);

    gyroData.x = ((int16_t)buffer[1] << 8) | buffer[0];
    gyroData.y = ((int16_t)buffer[3] << 8) | buffer[2];
    gyroData.z = ((int16_t)buffer[5] << 8) | buffer[4];

    return gyroData;
}

BNO_DOF3_Float Bno055::getGyroData(){
    BNO_DOF3_int16 gyroData;
    BNO_DOF3_Float returnData;
    uint8_t buffer[6];

    bnoReadBytes(BNO_REG_GYRO_DATA_X_LSB, buffer, 6, TIMEOUT_I2C);

    gyroData.x = ((int16_t)buffer[1] << 8) | buffer[0];
    gyroData.y = ((int16_t)buffer[3] << 8) | buffer[2];
    gyroData.z = ((int16_t)buffer[5] << 8) | buffer[4];

    gyroData.x -= this->gyroOffsets[0];
    gyroData.y -= this->gyroOffsets[1];
    gyroData.z -= this->gyroOffsets[2];

    returnData.x = (float)gyroData.x / this->gyroResolution;
    returnData.y = (float)gyroData.y / this->gyroResolution;
    returnData.z = (float)gyroData.z / this->gyroResolution;

    return returnData;
}

BNO_DOF3_Float Bno055::getMagData(){
    BNO_DOF3_int16 magData;
    BNO_DOF3_Float returnData;
    uint8_t buffer[6];

    bnoReadBytes(BNO_REG_MAG_DATA_X_LSB, buffer, 6, TIMEOUT_I2C);

    magData.x = ((int16_t)buffer[1] << 8) | buffer[0];
    magData.y = ((int16_t)buffer[3] << 8) | buffer[2];
    magData.z = ((int16_t)buffer[5] << 8) | buffer[4];

    magData.x -= this->magOffsets[0];
    magData.y -= this->magOffsets[1];
    magData.z -= this->magOffsets[2];

    returnData.x = (float)magData.x / this->magResolution;
    returnData.y = (float)magData.y / this->magResolution;
    returnData.z = (float)magData.z / this->magResolution;

    return returnData;
}

void Bno055::GyroCalibration(uint32_t numSample){
    BNO_DOF3_int16 gyro;
    int16_t gyromin[3] = {32767 , 32767 , 32767};
    int16_t gyromax[3] = {-32767 , -32767 , -32767};
    for(int i = 0; i < numSample; i++){
        if(numSample % 100 == 0){
            DEBUG_PRINT(F("Sample "));
            DEBUG_PRINTLN(i);
        }

        gyro = getRawGyroData();

        if(gyro.x > gyromax[0]){
            gyromax[0] = gyro.x;
        }
        if(gyro.x < gyromin[0]){
            gyromin[0] = gyro.x;
        }

        if(gyro.y > gyromax[1]){
            gyromax[1] = gyro.y;
        }
        if(gyro.y < gyromin[1]){
            gyromin[1] = gyro.y;
        }

        if(gyro.z > gyromax[2]){
            gyromax[2] = gyro.z;
        }
        if(gyro.z < gyromin[2]){
            gyromin[2] = gyro.z;
        }

        delay(20);
    }

    for(int i = 0; i < 3; i++){
        this->gyroOffsets[i] = (gyromax[i] + gyromin[i]) / 2;
    }
}

void Bno055::BNOInit(){
    this->changePage(0x00);
    BNO_OPERATIONMODE tempMode = this->bnoReg.registersPage_0.operationMode.operationMode;
    this->bnoReg.registersPage_0.operationMode.operationMode = OPERATIONMODE_CONFIG;
    this->setOperationMode();
    delay(10);
    this->setUnits();
    delay(10);
    this->setPowerMode();
    delay(10);

    delay(20);

    this->changePage(0x01);
    this->setAccConfig();
    delay(10);
    this->setGyroConfig();
    delay(10);
    this->setMagConfig();
    delay(10);
    this->setIntEnable();
    delay(10);
    this->setAccIntSettings();
    delay(10);

    this->bnoReg.registersPage_0.operationMode.operationMode = tempMode;

    this->changePage(0x00);
    this->setOperationMode();
    delay(20);
}