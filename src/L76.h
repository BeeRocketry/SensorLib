#include <Arduino.h>
#include "HardwareSerial.h"
#include "TinyGPSPlus.h"

struct L76_Data{
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    float speed = 0.0f;
    float course = 0.0f;
    int satellites = 0;
    int hdop = 0;
    unsigned long time = 0;
    unsigned long date = 0;
};

class L76 {
private:
    TinyGPSPlus* GPS = nullptr;
    HardwareSerial *serialPort = nullptr;
    HardwareSerial *debuggerPort = nullptr;

    struct _paramConfs{
        time_t serialTimeout = 1000;
    }paramConfs;

    bool dynamicSerial = false;

    L76_Data data;

public:
    L76(HardwareSerial *serial);
    L76(HardwareSerial *serial, HardwareSerial *debugger);
    L76(uint8_t rxPin, uint8_t txPin);

    ~L76();

    void begin();
    void getData();
    void printData() const;
    void setTimeoutRx(time_t timeout);

    L76_Data getDataStruct() const;
};