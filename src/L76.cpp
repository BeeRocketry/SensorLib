#include "L76.h"

L76::L76(HardwareSerial *serial) : serialPort(serial) {
    GPS = new TinyGPSPlus();
}

L76::L76(HardwareSerial *serial, HardwareSerial *debugger) : serialPort(serial), debuggerPort(debugger) {
    GPS = new TinyGPSPlus();
}

L76::L76(uint8_t rxPin, uint8_t txPin) {
    serialPort = new HardwareSerial(rxPin, txPin);
    GPS = new TinyGPSPlus();

    this->dynamicSerial = true;
}

L76::~L76() {
    if(this->GPS != nullptr){
        delete GPS;
    }

    if(this->dynamicSerial == true && this->serialPort != nullptr){
        delete this->serialPort;
    }
}

void L76::begin(){
    this->serialPort->begin(9600, SERIAL_8N1);
}

void L76::getData(){
    time_t startTime = millis();
    while (millis() - startTime < this->paramConfs.serialTimeout) {
        while (this->serialPort->available()) {
            char c = this->serialPort->read();
            if (GPS->encode(c)) {
                if (GPS->location.isUpdated()) {
                    this->data.latitude = GPS->location.lat();
                    this->data.longitude = GPS->location.lng();
                    this->data.altitude = GPS->altitude.meters();
                    this->data.speed = GPS->speed.kmph();
                    this->data.course = GPS->course.deg();
                    this->data.satellites = GPS->satellites.value();
                    this->data.hdop = GPS->hdop.hdop();
                    this->data.time = GPS->time.value();
                    this->data.date = GPS->date.value();
                }
                return;
            }
        }
    }
}

L76_Data L76::getDataStruct() const {
    return this->data;
}

void L76::printData() const {
    if(this->debuggerPort == nullptr){
        return;
    }

    Serial.print("Latitude: ");
    Serial.println(data.latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(data.longitude, 6);
    Serial.print("Altitude: ");
    Serial.println(data.altitude, 2);
    Serial.print("Speed: ");
    Serial.println(data.speed, 2);
    Serial.print("Course: ");
    Serial.println(data.course, 2);
    Serial.print("Satellites: ");
    Serial.println(data.satellites);
    Serial.print("HDOP: ");
    Serial.println(data.hdop);
    Serial.print("Time: ");
    Serial.println(data.time);
    Serial.print("Date: ");
    Serial.println(data.date);
}

void L76::setTimeoutRx(time_t timeout) {
    this->paramConfs.serialTimeout = timeout;
}