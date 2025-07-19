#include <Arduino.h>
#include <HardwareSerial.h>

#define MAX_TX_BUFFER_SIZE 200L
#define MAX_TX_BUFFER_SIZE_CRC (this->maxTxBufferSize - 1)
#define MAX_TX_BUFFER_SIZE_FIXED_CRC (this->maxTxBufferSize - 4)

#define RF_Operating_Config() {digitalWrite(this->_pinConfs.M0Pin, HIGH); digitalWrite(this->_pinConfs.M1Pin, HIGH);}
#define RF_Operating_Normal() {digitalWrite(this->_pinConfs.M0Pin, LOW); digitalWrite(this->_pinConfs.M1Pin, LOW);}
#define RF_WaitAUX() {if(waitAUX(this->_paramConfs.auxTimeout) == E220_Timeout)return E220_Timeout;}
#define RF_PackageTimerCheck() {if(packageTimerCheck() == E220_NoPackageTime)return E220_NoPackageTime;}

#define RF_PACKET_SPEC_HIGH 0x9F
#define RF_PACKET_SPEC_MID 0xA0
#define RF_PACKET_SPEC_LOW 0xC9

typedef enum Error_Status : uint8_t{
    E220_Success = 1,
    E220_Timeout,
    E220_CrcBroken,
    E220_FailureMode,
    E220_NoMessage,
    E220_BigPacket,
    E220_BrokenGetSet,
    E220_NoPackageTime,
    E220_OutOfLimit
} Status;

typedef enum RF_FREQ : uint8_t{
    FREQ_410 = 0x00,
    FREQ_411 = 0x01,
    FREQ_412 = 0x02,
    FREQ_413 = 0x03,
    FREQ_414 = 0x04,
    FREQ_415 = 0x05,
    FREQ_416 = 0x06,
    FREQ_417 = 0x07,
    FREQ_418 = 0x08,
    FREQ_419 = 0x09,
    FREQ_420 = 0x0A,
    FREQ_421 = 0x0B,
    FREQ_422 = 0x0C,
    FREQ_423 = 0x0D,
    FREQ_424 = 0x0E,
    FREQ_425 = 0x0F,
    FREQ_426 = 0x10,
    FREQ_427 = 0x11,
    FREQ_428 = 0x12,
    FREQ_429 = 0x13,
    FREQ_430 = 0x14,
    FREQ_431 = 0x15,
    FREQ_432 = 0x16,
    FREQ_433 = 0x17,
    FREQ_434 = 0x18,
    FREQ_435 = 0x19,
    FREQ_436 = 0x1A,
    FREQ_437 = 0x1B,
    FREQ_438 = 0x1C,
    FREQ_439 = 0x1D,
    FREQ_440 = 0x1E,
    FREQ_441 = 0x1F,
    FREQ_442 = 0x20,
    FREQ_443 = 0x21,
    FREQ_444 = 0x22,
    FREQ_445 = 0x23,
    FREQ_446 = 0x24,
    FREQ_447 = 0x25,
    FREQ_448 = 0x26,
    FREQ_449 = 0x27,
    FREQ_450 = 0x28,
    FREQ_451 = 0x29,
    FREQ_452 = 0x2A,
    FREQ_453 = 0x2B,
    FREQ_454 = 0x2C,
    FREQ_455 = 0x2D,
    FREQ_456 = 0x2E,
    FREQ_457 = 0x2F,
    FREQ_458 = 0x30,
    FREQ_459 = 0x31,
    FREQ_460 = 0x32,
    FREQ_461 = 0x33,
    FREQ_462 = 0x34,
    FREQ_463 = 0x35,
    FREQ_464 = 0x36,
    FREQ_465 = 0x37,
    FREQ_466 = 0x38,
    FREQ_467 = 0x39,
    FREQ_468 = 0x3A,
    FREQ_469 = 0x3B,
    FREQ_470 = 0x3C,
    FREQ_471 = 0x3D,
    FREQ_472 = 0x3E,
    FREQ_473 = 0x3F,
    FREQ_474 = 0x40,
    FREQ_475 = 0x41,
    FREQ_476 = 0x42,
    FREQ_477 = 0x43,
    FREQ_478 = 0x44,
    FREQ_479 = 0x45,
    FREQ_480 = 0x46,
    FREQ_481 = 0x47,
    FREQ_482 = 0x48,
    FREQ_483 = 0x49,
    FREQ_484 = 0x4A,
    FREQ_485 = 0x4B,
    FREQ_486 = 0x4C,
    FREQ_487 = 0x4D,
    FREQ_488 = 0x4E,
    FREQ_489 = 0x4F,
    FREQ_490 = 0x50,
    FREQ_491 = 0x51,
}RF_FREQ;

typedef enum RF_DEBUGGER_UART_PARITY{
    DEBUGGER_UART_PARITY_8N1 = SERIAL_8N1,
    DEBUGGER_UART_PARITY_8O1 = SERIAL_8O1,
    DEBUGGER_UART_PARITY_8E1 = SERIAL_8E1
}RF_DEBUGGER_UART_PARITY;

/*
------------------------
 Ayar Öntanim Makrolari
------------------------
*/
typedef enum RF_ONOFF : uint8_t{
    RF_OFF = 0b0,
    RF_ON = 0b1,
}RF_ONOFF;

// 0x02 REG0 -- 7, 6, 5. Bit
typedef enum RF_UART_BAUD : uint8_t{
    UARTBAUDRATE_1200 = 0b000,
    UARTBAUDRATE_2400 = 0b001,
    UARTBAUDRATE_4800 = 0b010,
    UARTBAUDRATE_9600 = 0b011,
    UARTBAUDRATE_19200 = 0b100,
    UARTBAUDRATE_38400 = 0b101,
    UARTBAUDRATE_57600 = 0b110,
    UARTBAUDRATE_115200 = 0b111,
}RF_UART_BAUD;

// 0x02 REG0 -- 4, 3. Bit
typedef enum RF_UART_PARITY : uint8_t{
    UARTPARITY_8N1 = 0b00,
    UARTPARITY_8O1 = 0b01,
    UARTPARITY_8E1 = 0b10,
}RF_UART_PARITY;

// 0x02 REG0 -- 2, 1, 0. Bit
typedef enum RF_AIR_DATA : uint8_t{
    AIRDATARATE_24k = 0b000,
    AIRDATARATE_48k = 0b011,
    AIRDATARATE_96k = 0b100,
    AIRDATARATE_192k = 0b101,
    AIRDATARATE_384k = 0b110,
    AIRDATARATE_625k = 0b111
}RF_AIR_DATA;

// 0x03 REG1 -- 7, 6. Bit
typedef enum RF_PACKET_SIZE : uint8_t{
    PACKET_200 = 0b00,
    PACKET_128 = 0b01,
    PACKET_64 = 0b10,
    PACKET_32 = 0b11
}RF_PACKET_SIZE;

// 0x03 REG1 -- 1, 0. Bit
typedef enum RF_TRANS_POWER : uint8_t{
    TRANSMISSIONPOWER_30 = 0b00,
    TRANSMISSIONPOWER_27 = 0b01,
    TRANSMISSIONPOWER_24 = 0b10,
    TRANSMISSIONPOWER_21 = 0b11,
}RF_TRANS_POWER;

// 0x05 REG3 -- 6. Bit
typedef enum RF_TRANS_MODE : uint8_t{
    TRANSPARENTMODE = 0b0,
    FIXEDMODE = 0b1,
    BROADCASTMODE = 0b11111,
}RF_TRANS_MODE;

// 0x05 REG3 -- 2, 1, 0. Bit
typedef enum RF_WIRELESS : uint8_t{
    WIRELESSWAKEUP_500 = 0b000,
    WIRELESSWAKEUP_1000 = 0b001,
    WIRELESSWAKEUP_1500 = 0b010,
    WIRELESSWAKEUP_2000 = 0b011,
    WIRELESSWAKEUP_2500 = 0b100,
    WIRELESSWAKEUP_3000 = 0b101,
    WIRELESSWAKEUP_3500 = 0b110,
    WIRELESSWAKEUP_4000 = 0b111
}RF_WIRELESS;

/* 
--------------------
 Struct Tanimlari
--------------------
*/
struct REG0{
    RF_UART_BAUD UARTBaud           =   UARTBAUDRATE_9600;  // 7, 6, 5 bit
    RF_UART_PARITY UARTParity       =   UARTPARITY_8N1;     // 4, 3 bit
    RF_AIR_DATA AirDataRate         =   AIRDATARATE_24k;    // 2, 1, 0 bit
};

struct REG1{
    RF_PACKET_SIZE PacketSize           =   PACKET_64;              // 7, 6 bit
    RF_ONOFF RSSIAmbient                =   RF_OFF;                 // 5 bit
    RF_TRANS_POWER TransmissionPower    =   TRANSMISSIONPOWER_30;   // 1, 0 bit
};

struct REG3{
    RF_ONOFF RSSIByte                   =   RF_ON;                // 7 bit
    RF_TRANS_MODE TransmissionMode      =   FIXEDMODE;            // 6 bit
    RF_ONOFF LBTEnable                  =   RF_OFF;               // 4. Bit
    RF_WIRELESS WirelessWakeUp          =   WIRELESSWAKEUP_500;   // 2, 1, 0 bit
};

struct ConfigRF{
    uint8_t AddressHigh =   0x12;         // 0x00 ADDH
    uint8_t AddressLow  =   0x83;         // 0x01 ADDL
    struct REG0 RFReg0;                   // 0x02 REG0
    struct REG1 RFReg1;                   // 0x03 REG1
    RF_FREQ Channel     =   FREQ_433;     // 0x04 Chan
    struct REG3 RFReg3;                   // 0x05 REG3
    uint8_t keyHigh     =   0;            // 0x06 KeyH
    uint8_t keyLow      =   0;            // 0x07 KeyL
};

struct RF_Msg{
    Status status = E220_Timeout;
    uint8_t buffer[MAX_TX_BUFFER_SIZE] = {0};
    uint8_t size = 0;
    int8_t rssiValue = -1;
    float rssiDbm = 0.0f;
    uint8_t crc = 0;

    RF_Msg(Status stat) 
        : status(stat){}

    RF_Msg(){

    }
};

class E220{
private:
    // Config Variables
    HardwareSerial *RFSerialPort = nullptr;
    HardwareSerial *DebuggerPort = nullptr;
    ConfigRF *tempConfig = nullptr;

    struct _paramConfs{
        time_t auxTimeout = 1000;
        time_t noAuxTimeOut = 30;
        time_t serialTimeout = 1000;
        time_t packetStartTimeStamp = 0;
        time_t packetEndTimeStamp = 0;
    }_paramConfs;

    struct _debuggerConfs{
        unsigned long baudRate = 115200;
        RF_DEBUGGER_UART_PARITY parity = DEBUGGER_UART_PARITY_8N1;
    }_debuggerConfs;
    
    struct ConfigRF _devConfs;
    
    // Pin Variables
    struct _pinConfs{
        int16_t M0Pin;
        int16_t M1Pin;
        int16_t AUXPin;
    }_pinConfs;

    // Private Functions
    void clearSerialBuffer() const;
    Status waitAUX(unsigned long timeout) const;
    void managedDelay(unsigned long timeout) const;
    Status setSettings(void) const;
    Status getSettings(void);

    String getTranmissionPower(const byte &transmissionpower) const;
    String getOnOff(const byte &onoff) const;
    String getPacketSize(const byte &packetsize) const;
    String getWirelessWakeup(const byte &wireless) const;
    String getTransmissionType(const byte &transmissiontype) const;
    String getAirData(const byte &airdata) const;
    String getUARTParity(const byte &uartparity) const;
    String getUARTBaudRate(const byte &uartbaud) const;

    float airDataRateEnum2Value(const RF_AIR_DATA &dataRate) const;
    long UARTRateEnum2Value(const RF_UART_BAUD &dataRate) const;
    time_t calculatePacketSendTime(const size_t &packetSize) const;
    Status setPinConfig(const int8_t &m0, const int8_t &m1, const int8_t &aux);
    Status setSerialBaudRateBegin() const;
    uint8_t setSerialParityBegin() const;
    Status packageTimerCheck() const;
    Status tempConftoDevice();
    Status maxTxConverter();
    Status rssiByteChecker();
    Status rssiAmbientChecker();
    Status checker();

    bool mPinSet = false;
    bool auxPinSet = false;
    bool dynamicRFPortSet = false;
    bool rssiAmbientSet = false;
    bool rssiByteSet = false;

    uint8_t maxTxBufferSize = 0;
    
public:
// Constructors
    E220(HardwareSerial *serialPort);
    E220(HardwareSerial *serialPort, HardwareSerial *debugger);
    E220(HardwareSerial *serialPort, const uint8_t& M0_Pin, const uint8_t& M1_Pin);
    E220(HardwareSerial *serialPort, const uint8_t& M0_Pin, const uint8_t& M1_Pin, HardwareSerial *debugger);
    E220(HardwareSerial *serialPort, const uint8_t& AUX_Pin, const uint8_t& M0_Pin, const uint8_t& M1_Pin);
    E220(HardwareSerial *serialPort, const uint8_t& AUX_Pin, const uint8_t& M0_Pin, const uint8_t& M1_Pin, HardwareSerial *debugger);
    E220(const uint8_t& TX_Pin, const uint8_t& RX_Pin, const uint8_t& AUX_Pin, const uint8_t& M0_Pin, const uint8_t& M1_Pin);
    E220(const uint8_t& TX_Pin, const uint8_t& RX_Pin, const uint8_t& M0_Pin, const uint8_t& M1_Pin);
    E220(const uint8_t& TX_Pin, const uint8_t& RX_Pin);

    // Destructor
    ~E220();

    // Public
    uint8_t calculateCRC8(const uint8_t *data, const size_t& length) const;
    Status receiveSingleData(uint8_t *data) const;
    Status receiveDataPacket(uint8_t *data, const size_t& size) const;
    RF_Msg receive() const;
    Status sendFixedSingleData(const uint8_t& AddressHigh, const uint8_t& AddressLow, const uint8_t& Channel, const uint8_t& data);
    Status sendTransparentSingleData(const uint8_t& data);
    Status sendFixedDataPacket(const uint8_t& AddressHigh, const uint8_t& AddressLow, const uint8_t& Channel, const uint8_t *data, const size_t& size);
    Status sendBroadcastDataPacket(const uint8_t& Channel, const uint8_t *data, const size_t& size);
    Status sendTransparentDataPacket(uint8_t *data, const size_t& size);
    Status send(const uint8_t& AddressHigh, const uint8_t& AddressLow, const uint8_t& Channel, const uint8_t* data, int size);

    Status findLeastFrequency();
    float checkFreq(const RF_FREQ &freq);
    Status viewFreqs() const;

    // Setters
    Status setTransmissionMode(const RF_TRANS_MODE &Mode);
    Status setAddresses(const uint8_t &AddHigh, const uint8_t &AddLow);
    Status setChannel(const RF_FREQ &channel);
    Status setTransmissionPower(const RF_TRANS_POWER &power);
    Status setWirelesWakeup(const RF_WIRELESS &time);
    Status setUARTParity(const RF_UART_PARITY &paritybyte);
    Status setUARTBaudRate(const RF_UART_BAUD &baudrate);
    Status setAirDataRate(const RF_AIR_DATA &airdatarate);
    Status setLTBEnable(const RF_ONOFF &ltb);
    Status setRSSIAmbient(const RF_ONOFF &rssiambient);
    Status setRSSIByte(const RF_ONOFF &rssi);
    Status setPacketSize(const RF_PACKET_SIZE &packetsize);
    Status setAuxTimeoutTime(const time_t &time);
    Status setNoAuxTimeoutTime(const time_t &time);

    Status viewSettings();

    Status RFBegin(const uint8_t& HighAddress, const uint8_t& LowAddress, const uint8_t& channel,
                   const RF_UART_BAUD& baud, const RF_AIR_DATA& airdata, const RF_ONOFF& rssiByte, const RF_PACKET_SIZE& packetsize, const RF_TRANS_MODE& transmode = FIXEDMODE,
                   const RF_TRANS_POWER& transpower = TRANSMISSIONPOWER_30,  const RF_WIRELESS& wirelesswake = WIRELESSWAKEUP_500, const RF_UART_PARITY& parity = UARTPARITY_8N1,
                   const RF_ONOFF& rssiAmbient = RF_OFF, const RF_ONOFF& ltb = RF_OFF);
    Status RFStart();
};