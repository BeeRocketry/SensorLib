#ifndef LPS22HH_H
#define LPS22HH_H

#include "I2Class.h"

#define SeaLevelhPa 1013.25

#define CHIP_ADDRESS 0x5D

#define INTERRUPT_CFG 0x0B
#define THS_P_L 0x0C
#define THS_P_H 0x0D
#define IF_CTRL 0x0E
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x10
#define CTRL_REG2 0x11
#define CTRL_REG3 0x12
#define FIFO_CTRL 0x13
#define FIFO_WTM 0x14
#define REF_P_L 0x15
#define REF_P_H 0x16
#define RPDS_L 0x18
#define RPDS_H 0x19
#define INT_SOURCE 0x24
#define FIFO_STATUS1 0x25
#define FIFO_STATUS2 0x26
#define STATUS 0x27
#define PRESSURE_OUT_XL 0x28
#define PRESSURE_OUT_L 0x29
#define PRESSURE_OUT_H 0x2A
#define TEMP_OUT_L 0x2B
#define TEMP_OUT_H 0x2C
#define FIFO_DATA_OUT_PRESS_XL 0x78
#define FIFO_DATA_OUT_PRESS_L 0x79
#define FIFO_DATA_OUT_PRESS_H 0x7A
#define FIFO_DATA_OUT_TEMP_L 0x7B
#define FIFO_DATA_OUT_TEMP_H 0x7C

#define PRES_SENS 4096.0
#define TEMP_SENS 100.0

typedef enum LPS_OUTPUT_DATA_RATE : uint8_t {
    LPS_OUTPUTDATARATE_ONESHOT = 0b000,
    LPS_OUTPUTDATARATE1HZ = 0b001,
    LPS_OUTPUTDATARATE10HZ = 0b010,
    LPS_OUTPUTDATARATE25HZ = 0b011,
    LPS_OUTPUTDATARATE50HZ = 0b100,
    LPS_OUTPUTDATARATE75HZ = 0b101,
    LPS_OUTPUTDATARATE100HZ = 0b110,
    LPS_OUTPUTDATARATE200HZ = 0b111,
}LPS_OUTPUT_DATA_RATE;

typedef enum LPS_EN_LPFP : uint8_t {
    LPS_LPFPDISABLE = 0b0,
    LPS_LPFPENABLE = 0b1,
}LPS_EN_LPFP;

typedef enum LPS_BDU : uint8_t {
    LPS_BDU_CONT = 0b0,
    LPS_BDU_NONCONT = 0b1,
}LPS_BDU;

typedef enum LPS_LOW_NOISE : uint8_t {
    LPS_LOW_CURRENT = 0b0,
    LPS_LOW_NOISE = 0b1,
}LPS_LOW_NOISE;

class LPS22HH {
    private:
        I2Class* i2c;
    public:
    LPS22HH(I2Class* _i2c);

    uint8_t lpsReadBytes(uint8_t regadr, uint8_t* temp, uint8_t length, uint16_t timeout = TIMEOUT_I2C);
    bool lpsWriteByte(uint8_t regadr, uint8_t data);
    void setCTRL_REG1(LPS_OUTPUT_DATA_RATE odrRate, LPS_EN_LPFP lpfp, LPS_BDU bdu);
    void setCTRL_REG2(LPS_LOW_NOISE lpsLowNoise);
    void resetLPS();
    float getPressure(); 
    float getTemperature();
    float getAltitude(float pressure);
    bool readWhoAmI();
};

#endif
