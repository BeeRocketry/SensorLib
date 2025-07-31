#ifndef L76_H
#define L76_H

#include <Arduino.h>
#include "HardwareSerial.h"
#include "TinyGPSPlus.h"
#include "DataTypes.h"

class L76 {
private:
    TinyGPSPlus* GPS = nullptr;
    HardwareSerial *serialPort = nullptr;
    HardwareSerial *debuggerPort = nullptr;

    struct _paramConfs{
        time_t serialTimeout = 1000;
    }paramConfs;

    bool dynamicSerial = false;

    GpsData data;

public:
    L76(HardwareSerial *serial);
    L76(HardwareSerial *serial, HardwareSerial *debugger);
    L76(uint8_t rxPin, uint8_t txPin);

    ~L76();

    void begin();
    void getData();
    void printData() const;
    void setTimeoutRx(time_t timeout);

    GpsData getDataStruct() const;
};

#endif