#ifndef DATATYPES_H
#define DATATYPES_H

typedef struct BaroData {
    float altitude = 0.0f;
    float pressure = 0.0f;
    float temperature = 0.0f;
}BaroData;

typedef struct imuData_float{
    float x = 0;
    float y = 0;
    float z = 0;
}imuData_float;

typedef struct imuData_int16{
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
}imuData_int16;

typedef struct ImuData{
    imuData_float acc;
    imuData_float gyro;
    imuData_float mag;
}ImuData;


typedef struct GpsData{
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    float speed = 0.0f;
    float course = 0.0f;
    int satellites = 0;
    int hdop = 0;
    unsigned long time = 0;
    unsigned long date = 0;
}GpsData;

#endif