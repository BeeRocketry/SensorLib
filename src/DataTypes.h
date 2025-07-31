#ifndef DATATYPES_H
#define DATATYPES_H

typedef struct BaroData {
    float altitude;
    float pressure;
    float temperature;
}BaroData;

typedef struct imuData_float{
    float x = 0;
    float y = 0;
    float z = 0;
};

typedef struct imuData_int16{
    int16_t x;
    int16_t y;
    int16_t z;
};

#endif