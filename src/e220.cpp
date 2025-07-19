#include "e220.h"

E220::E220(HardwareSerial *serialPort){
    this->RFSerialPort = serialPort;

    this->RFSerialPort->begin(9600);

    setPinConfig(-1, -1, -1);
}

E220::E220(HardwareSerial *serialPort, HardwareSerial *debugger){
    this->RFSerialPort = serialPort;
    this->DebuggerPort = debugger;

    this->RFSerialPort->begin(9600);
    //this->DebuggerPort->begin(this->_debuggerConfs.baudRate, this->_debuggerConfs.parity);

    setPinConfig(-1, -1, -1);
}

E220::E220(HardwareSerial *serialPort, const uint8_t& M0_Pin, const uint8_t& M1_Pin){
    this->RFSerialPort = serialPort;

    this->RFSerialPort->begin(9600);
    setPinConfig(M0_Pin, M1_Pin, -1);
    this->mPinSet = true;
}

E220::E220(HardwareSerial *serialPort, const uint8_t& M0_Pin, const uint8_t& M1_Pin, HardwareSerial *debugger){
    this->RFSerialPort = serialPort;
    this->DebuggerPort = debugger;

    this->RFSerialPort->begin(9600);
    //this->DebuggerPort->begin(this->_debuggerConfs.baudRate, this->_debuggerConfs.parity);

    setPinConfig(M0_Pin, M1_Pin, -1);
    this->mPinSet = true;
}

E220::E220(HardwareSerial *serialPort, const uint8_t &AUX_Pin, const uint8_t &M0_Pin, const uint8_t& M1_Pin){
    this->RFSerialPort = serialPort;

    this->RFSerialPort->begin(9600);
    setPinConfig(M0_Pin, M1_Pin, AUX_Pin);
    this->mPinSet = true;
    this->auxPinSet = true;
}

E220::E220(HardwareSerial *serialPort, const uint8_t& AUX_Pin, const uint8_t& M0_Pin, const uint8_t& M1_Pin, HardwareSerial *debugger){
    this->RFSerialPort = serialPort;
    this->DebuggerPort = debugger;

    this->RFSerialPort->begin(9600);

    //this->DebuggerPort->end();
    //this->DebuggerPort->begin(this->_debuggerConfs.baudRate, this->_debuggerConfs.parity);

    setPinConfig(M0_Pin, M1_Pin, AUX_Pin);
    this->mPinSet = true;
    this->auxPinSet = true;
}

E220::E220(const uint8_t& TX_Pin, const uint8_t& RX_Pin, const uint8_t& AUX_Pin, const uint8_t& M0_Pin, const uint8_t& M1_Pin){
    this->RFSerialPort = new HardwareSerial(TX_Pin, RX_Pin);
    this->dynamicRFPortSet = true;

    this->RFSerialPort->begin(9600);
    setPinConfig(M0_Pin, M1_Pin, AUX_Pin);
    this->mPinSet = true;
    this->auxPinSet = true;
}

E220::E220(const uint8_t& TX_Pin, const uint8_t& RX_Pin, const uint8_t& M0_Pin, const uint8_t& M1_Pin){
    this->RFSerialPort = new HardwareSerial(TX_Pin, RX_Pin);
    this->dynamicRFPortSet = true;

    this->RFSerialPort->begin(9600);
    setPinConfig(M0_Pin, M1_Pin, -1);
    this->mPinSet = true;
}

E220::E220(const uint8_t& TX_Pin, const uint8_t& RX_Pin){
    this->RFSerialPort = new HardwareSerial(TX_Pin, RX_Pin);
    this->dynamicRFPortSet = true;

    this->RFSerialPort->begin(9600);

    setPinConfig(-1, -1, -1);
}

uint8_t E220::calculateCRC8(const uint8_t *data, const size_t& length) const {
    if(sizeof(data) + 1 > MAX_TX_BUFFER_SIZE){
        DebuggerPort->println(F("CRC8 Fonksiyonu Maksimum Paketten Büyük"));
        return E220_CrcBroken;
    }

    uint8_t crc = 0x00;

    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x80){
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

void E220::clearSerialBuffer() const{
    while (RFSerialPort->available() > 0) {
        RFSerialPort->read();
    }
}

Status E220::waitAUX(unsigned long timeout) const{
    long startTime = millis();
    if(this->auxPinSet){
        while(digitalRead(this->_pinConfs.AUXPin) == LOW){
            if (millis() - startTime > timeout) {
                return E220_Timeout;
            }
            managedDelay(2);
        }

        managedDelay(10);
    }
    else{
        while(millis() - startTime < this->_paramConfs.noAuxTimeOut){
            managedDelay(2);
        }
    }

    return E220_Success;
}

void E220::managedDelay(unsigned long timeout) const{
    unsigned long t = millis();

    if((unsigned long) (t + timeout) == 0){
        t = 0;
    }

    while((millis()-t) < timeout){
    }
}

Status E220::setPinConfig(const int8_t &m0, const int8_t &m1, const int8_t &aux){
    this->_pinConfs.M0Pin = m0;
    this->_pinConfs.M1Pin = m1;
    this->_pinConfs.AUXPin = aux;
    return E220_Success;
}

Status E220::setTransmissionMode(const RF_TRANS_MODE &Mode){
    switch (Mode)
    {
    case TRANSPARENTMODE:
        this->_devConfs.RFReg3.TransmissionMode = TRANSPARENTMODE;
        break;
    
    case FIXEDMODE:
        this->_devConfs.RFReg3.TransmissionMode = FIXEDMODE;
        break;

    case BROADCASTMODE:
        this->_devConfs.RFReg3.TransmissionMode = FIXEDMODE;
        break;

    default:
        this->_devConfs.RFReg3.TransmissionMode = TRANSPARENTMODE;
        return E220_FailureMode;
    }

    return E220_Success;
}

Status E220::setPacketSize(const RF_PACKET_SIZE &packetsize){
    switch (packetsize)
    {
    case PACKET_200:
        this->_devConfs.RFReg1.PacketSize = PACKET_200;
        break;

    case PACKET_128:
        this->_devConfs.RFReg1.PacketSize = PACKET_128;
        break;

    case PACKET_64:
        this->_devConfs.RFReg1.PacketSize = PACKET_64;
        break;

    case PACKET_32:
        this->_devConfs.RFReg1.PacketSize = PACKET_32;
        break;

    default:
        this->_devConfs.RFReg1.PacketSize = PACKET_64;
        return E220_FailureMode;
    }

    return E220_Success;
}

Status E220::setRSSIAmbient(const RF_ONOFF &rssiambient){
    if(rssiambient != RF_ON && rssiambient != RF_OFF){
        return E220_FailureMode;
    }

    this->_devConfs.RFReg1.RSSIAmbient = rssiambient;
    return E220_Success;
}

Status E220::setRSSIByte(const RF_ONOFF &rssiByte){
    if(rssiByte != RF_ON && rssiByte != RF_OFF){
        return E220_FailureMode;
    }

    this->_devConfs.RFReg3.RSSIByte = rssiByte;
    return E220_Success;
}

Status E220::setLTBEnable(const RF_ONOFF &ltb){
    if(ltb != RF_ON && ltb != RF_OFF){
        return E220_FailureMode;
    }

    this->_devConfs.RFReg3.LBTEnable = ltb;
    return E220_Success;
}

Status E220::setAddresses(const uint8_t &AddHigh, const uint8_t &AddLow){
    if(AddHigh > 255 || AddHigh < 0){
        return E220_OutOfLimit;
    }
    if(AddLow > 255 || AddLow < 0){
        return E220_OutOfLimit;
    }

    this->_devConfs.AddressHigh = AddHigh;
    this->_devConfs.AddressLow = AddLow;
    return E220_Success;
}

Status E220::setChannel(const RF_FREQ &channel){
    if(channel > FREQ_491 || channel < FREQ_410){
        return E220_OutOfLimit;
    }

    this->_devConfs.Channel = channel;
    return E220_Success;
}

Status E220::setTransmissionPower(const RF_TRANS_POWER &power){
    if(!(power > TRANSMISSIONPOWER_21 || power < TRANSMISSIONPOWER_30)){
        return E220_FailureMode;
    }

    this->_devConfs.RFReg1.TransmissionPower = power;
    return E220_Success;
}

Status E220::setWirelesWakeup(const RF_WIRELESS &time){
    if(!(time > WIRELESSWAKEUP_4000 || time < WIRELESSWAKEUP_500)){
        return E220_Success;
    }

    this->_devConfs.RFReg3.WirelessWakeUp = time;
    return E220_Success;
}

Status E220::setUARTParity(const RF_UART_PARITY &paritybyte){
    if(!(paritybyte > UARTPARITY_8E1 || paritybyte << UARTPARITY_8N1)){
        return E220_FailureMode;
    }

    this->_devConfs.RFReg0.UARTParity = paritybyte;
    return E220_Success;
}

Status E220::setUARTBaudRate(const RF_UART_BAUD &baudrate){
    if(!(baudrate > UARTBAUDRATE_115200 || baudrate < UARTBAUDRATE_1200)){
        return E220_FailureMode;
    }

    this->_devConfs.RFReg0.UARTBaud = baudrate;
    return E220_Success;
}

Status E220::setAirDataRate(const RF_AIR_DATA &airdatarate){
    if(!(airdatarate > AIRDATARATE_625k || airdatarate < AIRDATARATE_24k)){
        return E220_FailureMode;
    }

    this->_devConfs.RFReg0.AirDataRate = airdatarate;
    return E220_Success;
}

Status E220::setSerialBaudRateBegin() const{
    switch (this->_devConfs.RFReg0.UARTBaud)
    {
    case UARTBAUDRATE_1200:
        this->RFSerialPort->end();
        this->RFSerialPort->begin(1200, setSerialParityBegin());
        break;

    case UARTBAUDRATE_2400:
        this->RFSerialPort->end();
        this->RFSerialPort->begin(2400, setSerialParityBegin());
        break;

    case UARTBAUDRATE_4800:
        this->RFSerialPort->end();
        this->RFSerialPort->begin(4800, setSerialParityBegin());
        break;

    case UARTBAUDRATE_9600:
        this->RFSerialPort->end();
        this->RFSerialPort->begin(9600, setSerialParityBegin());
        break;
    
    case UARTBAUDRATE_19200:
        this->RFSerialPort->end();
        this->RFSerialPort->begin(19200, setSerialParityBegin());
        break;

    case UARTBAUDRATE_38400:
        this->RFSerialPort->end();
        this->RFSerialPort->begin(38400, setSerialParityBegin());
        break;

    case UARTBAUDRATE_57600:
        this->RFSerialPort->end();
        this->RFSerialPort->begin(57600, setSerialParityBegin());
        break;

    case UARTBAUDRATE_115200:
        this->RFSerialPort->end();
        this->RFSerialPort->begin(115200, setSerialParityBegin());
        break;
    }

    return E220_Success;
}

uint8_t E220::setSerialParityBegin() const{
    switch (this->_devConfs.RFReg0.UARTParity)
    {
    case UARTPARITY_8N1:
        return SERIAL_8N1;
    
    case UARTPARITY_8E1:
        return SERIAL_8E1;

    case UARTPARITY_8O1:
        return SERIAL_8O1;
    }

    return SERIAL_8N1;
}

Status E220::setAuxTimeoutTime(const time_t &time){
    if(time < 30){
        return E220_FailureMode;
    }

    this->_paramConfs.auxTimeout = time;
    return E220_Success;
}

Status E220::setNoAuxTimeoutTime(const time_t &time){
    if(time < 30){
        return E220_FailureMode;
    }

    this->_paramConfs.noAuxTimeOut = time;
    return E220_Success;
}

String E220::getUARTBaudRate(const byte &uartbaud) const{
    switch (uartbaud)
    {
    case UARTBAUDRATE_1200:
        return F("1200 bps");
        break;
    
    case UARTBAUDRATE_2400:
        return F("2400 bps");
        break;

    case UARTBAUDRATE_4800:
        return F("4800 bps");
        break;

    case UARTBAUDRATE_9600:
        return F("9600 bps (Varsayilan)");
        break;

    case UARTBAUDRATE_19200:
        return F("19200 bps");
        break;

    case UARTBAUDRATE_38400:
        return F("38400 bps");
        break;

    case UARTBAUDRATE_57600:
        return F("57600 bps");
        break;

    case UARTBAUDRATE_115200:
        return F("115200 bps");
        break;
    }
    return F("9600 bps (Varsayilan)");
}

String E220::getPacketSize(const byte &packetsize) const{
    switch (packetsize)
    {
    case PACKET_32:
        return F("32 Byte");
        break;

    case PACKET_64:
        return F("64 Byte");
        break;

    case PACKET_128:
        return F("128 Byte");
        break;

    case PACKET_200:
        return F("200 Byte");
        break;
    }
    return F("64 Byte");
}

String E220::getUARTParity(const byte &uartparity) const{
    switch (uartparity)
    {
    case UARTPARITY_8N1:
        return F("8 Bit, Parity Yok, 1 Durdurma Biti");
        break;
    
    case UARTPARITY_8E1:
        return F("8 Bit, Çift Parity, 1 Durdurma Biti");
        break;

    case UARTPARITY_8O1:
        return F("8 Bit, Tek Parity, 1 Durdurma Biti");
        break;
    }
    return F("8 Bit, Parity Yok, 1 Durdurma Biti");
}

String E220::getAirData(const byte &airdata) const{
    switch (airdata)
    {
    case AIRDATARATE_24k:
        return F("2.4k bps (Varsayilan)");
        break;

    case AIRDATARATE_48k:
        return F("4.8k bps");
        break;

    case AIRDATARATE_96k:
        return F("9.6k bps");
        break;

    case AIRDATARATE_192k:
        return F("19.2k bps");
        break;

    case AIRDATARATE_384k:
        return F("38.4k bps");
        break;

    case AIRDATARATE_625k:
        return F("62.5k bps");
        break;
    }
    return F("2.4k bps (Varsayilan)");
}

String E220::getOnOff(const byte &onoff) const{
    switch (onoff)
    {
    case RF_ON:
        return F("Açık");
        break;
    
    case RF_OFF:
        return F("Kapalı");
        break;
    }
    return F("Kapalı");
}

String E220::getTransmissionType(const byte &transmissiontype) const{
    switch (transmissiontype)
    {
    case TRANSPARENTMODE:
        return F("Seffaf Mod");
        break;
    
    case FIXEDMODE:
        return F("Sabit Kanal Modu");
        break;
    }
    return F("Sabit Kanal Modu");
}

String E220::getWirelessWakeup(const byte &wireless) const{
    switch (wireless)
    {
    case WIRELESSWAKEUP_500:
        return F("500 ms");
        break;
    
    case WIRELESSWAKEUP_1000:
        return F("1000 ms");
        break;
    
    case WIRELESSWAKEUP_1500:
        return F("1500 ms");
        break;

    case WIRELESSWAKEUP_2000:
        return F("2000 ms");
        break;

    case WIRELESSWAKEUP_2500:
        return F("2500 ms");
        break;

    case WIRELESSWAKEUP_3000:
        return F("3000 ms");
        break;

    case WIRELESSWAKEUP_3500:
        return F("3500 ms");
        break;

    case WIRELESSWAKEUP_4000:   
        return F("4000 ms");
        break;
    } 
    return F("500 ms");
}

String E220::getTranmissionPower(const byte &transmissionpower) const{
    switch (transmissionpower)
    {
    case TRANSMISSIONPOWER_21:
        return F("21 dBm");
        break;
    
    case TRANSMISSIONPOWER_24:
        return F("24 dBm");
        break;
    
    case TRANSMISSIONPOWER_27:
        return F("27 dBm");
        break;
    
    case TRANSMISSIONPOWER_30:
        return F("30 dBm");
        break;
    }
    return F("30 dBm");
}

float E220::airDataRateEnum2Value(const RF_AIR_DATA &dataRate) const{
    if(dataRate == AIRDATARATE_24k){
        return 2.4;
    }
    else if(dataRate == AIRDATARATE_48k){
        return 4.8;
    }
    else if(dataRate == AIRDATARATE_96k){
        return 9.6;
    }
    else if(dataRate == AIRDATARATE_192k){
        return 19.2;
    }
    else if(dataRate == AIRDATARATE_384k){
        return 38.4;
    }
    else if(dataRate == AIRDATARATE_625k){
        return 62.5;
    }
    else{
        return -1;
    }
}

long E220::UARTRateEnum2Value(const RF_UART_BAUD &dataRate) const{
    if(dataRate == UARTBAUDRATE_1200){
        return 1200;
    }
    else if(dataRate == UARTBAUDRATE_2400){
        return 2400;
    }
    else if(dataRate == UARTBAUDRATE_4800){
        return 4800;
    }
    else if(dataRate == UARTBAUDRATE_9600){
        return 9600;
    }
    else if(dataRate == UARTBAUDRATE_19200){
        return 19200;
    }
    else if(dataRate == UARTBAUDRATE_38400){
        return 38400;
    }
    else if(dataRate == UARTBAUDRATE_57600){
        return 57600;
    }
    else if(dataRate == UARTBAUDRATE_115200){
        return 115200;
    }
    return -1;
}

time_t E220::calculatePacketSendTime(const size_t &packetSize) const{
    uint16_t packetBitSize = packetSize * 8;
    time_t packetTime = 0;

    // Air Data Rate Time
    packetTime += (1000 * packetBitSize) / (airDataRateEnum2Value(this->_devConfs.RFReg0.AirDataRate) * 1000);

    // UART Time
    packetTime += (1000 * packetBitSize) / UARTRateEnum2Value(this->_devConfs.RFReg0.UARTBaud);

    return packetTime;
}

Status E220::setSettings(void) const{
    if(this->mPinSet){
        uint8_t Reg0Byte = 0, Reg1Byte = 0, Reg3Byte = 0;
        uint8_t MesArr[9];

        Reg0Byte = (this->_devConfs.RFReg0.UARTBaud << 5) | (this->_devConfs.RFReg0.UARTParity << 3) | (this->_devConfs.RFReg0.AirDataRate);

        Reg1Byte = (this->_devConfs.RFReg1.PacketSize << 6) | (this->_devConfs.RFReg1.RSSIAmbient << 5) | (this->_devConfs.RFReg1.TransmissionPower);

        Reg3Byte = (this->_devConfs.RFReg3.RSSIByte << 7) | (this->_devConfs.RFReg3.TransmissionMode << 6) | (this->_devConfs.RFReg3.LBTEnable << 4) | (this->_devConfs.RFReg3.WirelessWakeUp);

        RF_WaitAUX();

        this->RFSerialPort->end();
        this->RFSerialPort->begin(9600);
        managedDelay(200);

        RF_Operating_Config();

        managedDelay(20);

        MesArr[0] = 0xC0;
        MesArr[1] = 0x00;
        MesArr[2] = 6;
        MesArr[3] = this->_devConfs.AddressHigh;
        MesArr[4] = this->_devConfs.AddressLow;
        MesArr[5] = Reg0Byte;
        MesArr[6] = Reg1Byte;
        MesArr[7] = this->_devConfs.Channel;
        MesArr[8] = Reg3Byte;

        RFSerialPort->write((uint8_t *)MesArr, sizeof(MesArr) / sizeof(MesArr[0]));

        RF_WaitAUX();

        managedDelay(750);

        RF_Operating_Normal();

        setSerialBaudRateBegin();
        managedDelay(200);

        managedDelay(20);

        return E220_Success;
    }
    else{
        if(this->DebuggerPort != nullptr)
            this->DebuggerPort->println(F("M0 and M1 pins not set. Device's configs cannot be change."));
        return E220_FailureMode;
    }
};

Status E220::getSettings(void){
    if(this->mPinSet){
        if(this->tempConfig == nullptr){
            this->tempConfig = new ConfigRF;
        }

        memset(this->tempConfig, 0, sizeof(ConfigRF));
        
        clearSerialBuffer();
        uint8_t Reg0Byte = 0, Reg1Byte = 0, Reg3Byte = 0;
        uint8_t MesArr[9], sendpack[3];

        RF_WaitAUX();

        this->RFSerialPort->end();
        this->RFSerialPort->begin(9600);
        managedDelay(200);
        RF_Operating_Config();

        managedDelay(100);

        sendpack[0] = 0xC1;
        sendpack[1] = 0x00;
        sendpack[2] = 6;

        RFSerialPort->write((uint8_t *)sendpack, sizeof(sendpack) / sizeof(sendpack[0]));
        
        long startTime = millis();
        while(RFSerialPort->available() < sizeof(MesArr)){
            if(millis() - startTime > this->_paramConfs.serialTimeout){
                return E220_Timeout;
            }
            managedDelay(20);
        }

        RFSerialPort->readBytes(MesArr, sizeof(MesArr));

        this->tempConfig->AddressHigh = MesArr[3];
        this->tempConfig->AddressLow = MesArr[4];
        Reg0Byte = MesArr[5];
        Reg1Byte = MesArr[6];
        this->tempConfig->Channel = (RF_FREQ)MesArr[7];
        Reg3Byte = MesArr[8];

        this->tempConfig->RFReg0.UARTBaud = (RF_UART_BAUD)((Reg0Byte >> 5) & 0b111);
        this->tempConfig->RFReg0.UARTParity = (RF_UART_PARITY)((Reg0Byte >> 3) & 0b11);
        this->tempConfig->RFReg0.AirDataRate = (RF_AIR_DATA)((Reg0Byte) & 0b111);

        this->tempConfig->RFReg1.PacketSize = (RF_PACKET_SIZE)((Reg1Byte >> 6) & 0b11);
        this->tempConfig->RFReg1.RSSIAmbient = (RF_ONOFF)((Reg1Byte >> 5) & 0b1);
        this->tempConfig->RFReg1.TransmissionPower = (RF_TRANS_POWER)((Reg1Byte) & 0b11);

        this->tempConfig->RFReg3.RSSIByte = (RF_ONOFF)((Reg3Byte >> 7) & 0b1);
        this->tempConfig->RFReg3.TransmissionMode = (RF_TRANS_MODE)((Reg3Byte >> 6) & 0b1);
        this->tempConfig->RFReg3.LBTEnable = (RF_ONOFF)((Reg3Byte >> 4) & 0b1);
        this->tempConfig->RFReg3.WirelessWakeUp = (RF_WIRELESS)(Reg3Byte & 0b111);

        RF_Operating_Normal();

        setSerialBaudRateBegin();
        managedDelay(200);

        managedDelay(100);

        return E220_Success;
    }
    else{
        this->DebuggerPort->println(F("M0 and M1 pins not set. Device's configs cannot be fetch."));
        return E220_FailureMode;
    }
}

Status E220::viewSettings(void){
    if(this->DebuggerPort == nullptr) return E220_FailureMode;

    this->getSettings();
    DebuggerPort->println(F("------------------------------------------------------"));
    DebuggerPort->print(F("Yuksek Adres: "));    DebuggerPort->println(this->_devConfs.AddressHigh);

    DebuggerPort->print(F("Dusuk Adres: "));    DebuggerPort->println(this->_devConfs.AddressLow);

    DebuggerPort->print(F("Kanal: "));    DebuggerPort->print(this->_devConfs.Channel);
    DebuggerPort->print(F(" - "));    DebuggerPort->print(410+this->_devConfs.Channel);   DebuggerPort->println(F(" MHz"));
    DebuggerPort->println();

    DebuggerPort->println(F("Reg0 Ayarlari"));
    DebuggerPort->print(F("  UART Baud Rate: "));     DebuggerPort->println(getUARTBaudRate(this->_devConfs.RFReg0.UARTBaud));
    DebuggerPort->print(F(" UART Parity: "));     DebuggerPort->println(getUARTParity(this->_devConfs.RFReg0.UARTParity));
    DebuggerPort->print(F("  Air Data Rate: "));     DebuggerPort->println(getAirData(this->_devConfs.RFReg0.AirDataRate));
    DebuggerPort->println();

    DebuggerPort->println(F("Reg1 Ayarlari"));
    DebuggerPort->print(F("  Maks Paket Boyutu: "));     DebuggerPort->println(getPacketSize(this->_devConfs.RFReg1.PacketSize));
    DebuggerPort->print(F(" RSSI Ambient: "));     DebuggerPort->println(getOnOff(this->_devConfs.RFReg1.RSSIAmbient));
    DebuggerPort->print(F("  Aktarim Gucu: "));     DebuggerPort->println(getTranmissionPower(this->_devConfs.RFReg1.TransmissionPower));
    DebuggerPort->println();

    DebuggerPort->println(F("Reg3 Ayarlari"));
    DebuggerPort->print(F("  Transfer Turu: "));      DebuggerPort->println(getTransmissionType(this->_devConfs.RFReg3.TransmissionMode));
    DebuggerPort->print(F("  LTB: "));        DebuggerPort->println(getOnOff(this->_devConfs.RFReg3.LBTEnable));
    DebuggerPort->print(F("  Wireless Uyanma Suresi: "));     DebuggerPort->println(getWirelessWakeup(this->_devConfs.RFReg3.WirelessWakeUp));
    DebuggerPort->print(F("  RSSI Byte: "));       DebuggerPort->println(getOnOff(this->_devConfs.RFReg3.RSSIByte));
    DebuggerPort->println();
    DebuggerPort->println(F("------------------------------------------------------"));
    return E220_Success;
}

Status E220::packageTimerCheck() const{
    if(millis() <= this->_paramConfs.packetEndTimeStamp){
        this->DebuggerPort->println(F("Delay duration is not enough to send this data packet !!!"));
        this->DebuggerPort->println(F("Previous Packet Still Sending.."));
        this->DebuggerPort->println(F("Try to increase UART or Air Data Rate. Or decrease data packet size."));
        this->DebuggerPort->println(F("Working in this config can cause unwanted delays and can cause a damage on device..."));
        this->DebuggerPort->print(F("Delay Time : "));this->DebuggerPort->print(millis() - this->_paramConfs.packetStartTimeStamp);this->DebuggerPort->print(F("   "));
        this->DebuggerPort->print(F("Previous Package's Start Time : "));this->DebuggerPort->println(this->_paramConfs.packetStartTimeStamp);
        this->DebuggerPort->print(F("Previous Package's End Time : "));this->DebuggerPort->println(this->_paramConfs.packetEndTimeStamp);
        this->DebuggerPort->print(F("Previous Package's Duration : "));this->DebuggerPort->println(this->_paramConfs.packetEndTimeStamp - this->_paramConfs.packetStartTimeStamp);
        return E220_NoPackageTime;
    }
    return E220_Success;
}

E220::~E220(){
    if(this->dynamicRFPortSet)
        delete this->RFSerialPort;

    if(this->mPinSet)
        delete this->tempConfig;
}

Status E220::tempConftoDevice(){
    this->_devConfs.RFReg0 = this->tempConfig->RFReg0;
    this->_devConfs.RFReg1 = this->tempConfig->RFReg1;
    this->_devConfs.AddressHigh = this->tempConfig->AddressHigh;
    this->_devConfs.AddressLow = this->tempConfig->AddressLow;
    this->_devConfs.Channel = this->tempConfig->Channel;
    this->_devConfs.RFReg3 = this->tempConfig->RFReg3;
    return E220_Success;
}

Status E220::RFBegin(const uint8_t& HighAddress, const uint8_t& LowAddress, const uint8_t& channel,
                     const RF_UART_BAUD& baud, const RF_AIR_DATA& airdata, const RF_ONOFF& rssiByte, const RF_PACKET_SIZE& packetsize, const RF_TRANS_MODE& transmode = FIXEDMODE,
                     const RF_TRANS_POWER& transpower = TRANSMISSIONPOWER_30,  const RF_WIRELESS& wirelesswake = WIRELESSWAKEUP_500, const RF_UART_PARITY& parity = UARTPARITY_8N1,
                     const RF_ONOFF& rssiAmbient = RF_OFF, const RF_ONOFF& ltb = RF_OFF){
                                this->_devConfs.AddressHigh = HighAddress;
                                this->_devConfs.AddressLow = LowAddress;
                                this->_devConfs.Channel = (RF_FREQ)channel;

                                this->_devConfs.RFReg0.UARTParity = parity;
                                this->_devConfs.RFReg0.UARTBaud = baud;
                                this->_devConfs.RFReg0.AirDataRate = airdata;

                                this->_devConfs.RFReg1.TransmissionPower = transpower;
                                this->_devConfs.RFReg1.PacketSize = packetsize;
                                this->_devConfs.RFReg1.RSSIAmbient = rssiAmbient;

                                this->_devConfs.RFReg3.TransmissionMode = transmode;
                                this->_devConfs.RFReg3.WirelessWakeUp = wirelesswake;
                                this->_devConfs.RFReg3.RSSIByte = rssiByte;
                                this->_devConfs.RFReg3.LBTEnable = ltb;

                                checker();

                                return this->setSettings();
                            }

Status E220::RFStart() {
    checker();

    return setSettings();
}

Status E220::checker(){
    maxTxConverter();
    rssiByteChecker();
    rssiAmbientChecker();
}

Status E220::rssiByteChecker(){
    if(this->_devConfs.RFReg3.RSSIByte == RF_ON){
        this->rssiByteSet = true;
    }
    else{
        this->rssiByteSet = false;
    }

    return E220_Success;
}

Status E220::rssiAmbientChecker(){
    if(this->_devConfs.RFReg1.RSSIAmbient == RF_ON){
        this->rssiAmbientSet = true;
    }
    else{
        this->rssiAmbientSet = false;
    }

    return E220_Success;
}

Status E220::maxTxConverter(){
    switch(this->_devConfs.RFReg1.PacketSize){
        case PACKET_32:
            this->maxTxBufferSize = 32;
            break;

        case PACKET_64:
            this->maxTxBufferSize = 64;
            break;

        case PACKET_128:
            this->maxTxBufferSize = 128;
            break;

        case PACKET_200:
            this->maxTxBufferSize = 200;
            break;

        default:
            this->maxTxBufferSize = 64;
            return E220_FailureMode;
    }

    return E220_Success;
}

Status E220::receiveSingleData(uint8_t *data) const {
    RF_WaitAUX();

    unsigned long t = millis();
    while(this->RFSerialPort->available() == 0){
        if(millis() - t > 1000){
            this->DebuggerPort->println(F("Veri Okuma Zaman Asimina Ugradi..."));
            return E220_Timeout;
        }
        managedDelay(20);
    }

    RF_WaitAUX();

    *data = this->RFSerialPort->read();
 
    clearSerialBuffer();
    this->DebuggerPort->println(F("Veri Alindi..."));
    return E220_Success;
}

Status E220::receiveDataPacket(uint8_t *data, const size_t& size) const{
    unsigned long t = millis();

    while(this->RFSerialPort->available() < size + 1){
        if(this->RFSerialPort->available() == 0 && millis() - t > 300){
            this->DebuggerPort->println(F("Herhangi bir Veri Paketi gelmedi..."));
            return E220_NoMessage;
        }
        else if(millis() - t > this->_paramConfs.serialTimeout){
            //this->DebuggerPort->println(F("Veri okuma zaman asimina ugradi..."));
            return E220_Timeout;
        }
        managedDelay(20);
    }

    RF_WaitAUX();

    this->RFSerialPort->readBytes(data, size + 1);

    uint8_t crc = 0x00;
    crc = calculateCRC8(data, size);

    if(crc != data[size]){
        this->DebuggerPort->println(F("Paket CRC Uyuşmuyor"));
        return E220_CrcBroken;
    }

    clearSerialBuffer();
    return E220_Success;
}

Status E220::sendFixedSingleData(const uint8_t& AddressHigh, const uint8_t& AddressLow, const uint8_t& Channel, const uint8_t& data) {
    RF_PackageTimerCheck();
    
    uint8_t packet[4];
    packet[0] = AddressHigh;
    packet[1] = AddressLow;
    packet[2] = Channel;
    packet[3] = data;

    time_t packageTime = calculatePacketSendTime(sizeof(packet));

    RF_WaitAUX();

    this->_paramConfs.packetStartTimeStamp = millis();
    this->_paramConfs.packetEndTimeStamp = this->_paramConfs.packetStartTimeStamp + packageTime; 
    this->RFSerialPort->write((uint8_t *)packet, sizeof(packet) / sizeof(packet[0]));

    RF_WaitAUX();

    clearSerialBuffer();

    return E220_Success;
}

Status E220::sendTransparentSingleData(const uint8_t& data){
    RF_PackageTimerCheck();

    RF_WaitAUX();

    time_t packageTime = calculatePacketSendTime(sizeof(data));

    this->_paramConfs.packetStartTimeStamp = millis();
    this->_paramConfs.packetEndTimeStamp = this->_paramConfs.packetStartTimeStamp + packageTime;
    this->RFSerialPort->write(data);

    RF_WaitAUX();
    
    clearSerialBuffer();

    return E220_Success;
}

Status E220::sendFixedDataPacket(const uint8_t& AddressHigh, const uint8_t& AddressLow, const uint8_t& Channel, const uint8_t *data, const size_t& size) {
    RF_PackageTimerCheck();
    
    if(size > MAX_TX_BUFFER_SIZE_FIXED_CRC){
        return E220_BigPacket;
    }

    uint8_t packetSize = size + 4;

    uint8_t packet[packetSize];

    packet[0] = AddressHigh;
    packet[1] = AddressLow;
    packet[2] = Channel;
    memcpy(&packet[3], data, size);

    time_t packageTime = calculatePacketSendTime(sizeof(packet));

    uint8_t crc;
    crc = this->calculateCRC8(data, size);
    packet[packetSize - 1] = crc;

    RF_WaitAUX();

    this->_paramConfs.packetStartTimeStamp = millis();
    this->_paramConfs.packetEndTimeStamp = this->_paramConfs.packetStartTimeStamp + packageTime;
    this->RFSerialPort->write((uint8_t *)packet, sizeof(packet));

    RF_WaitAUX();

    managedDelay(5);

    clearSerialBuffer();

    return E220_Success;
}

Status E220::sendBroadcastDataPacket(const uint8_t& Channel, const uint8_t *data, const size_t& size) {
    return this->sendFixedDataPacket(0x00, 0x00, Channel, data, size);
}

Status E220::sendTransparentDataPacket(uint8_t *data, const size_t& size){
    RF_PackageTimerCheck();

    if(size > MAX_TX_BUFFER_SIZE_CRC){
        return E220_BigPacket;
    }

    uint8_t packetSize = size + 1;

    uint8_t packet[packetSize];
    memcpy(packet, data, size);

    uint8_t crc = calculateCRC8(packet, size);
    packet[(sizeof(data) / sizeof(data[0]))] = crc;

    time_t packageTime = calculatePacketSendTime(sizeof(packet));

    RF_WaitAUX();

    this->_paramConfs.packetStartTimeStamp = millis();
    this->_paramConfs.packetEndTimeStamp = this->_paramConfs.packetStartTimeStamp + packageTime;
    this->RFSerialPort->write((uint8_t *)packet, sizeof(packet));

    RF_WaitAUX();

    managedDelay(5);

    clearSerialBuffer();

    return E220_Success;
}

RF_Msg E220::receive() const{
    bool highCheck = false, midCheck = false, lowCheck = false;
    uint8_t size = 0;

    bool finish = false;

    RF_Msg msg;
    uint8_t i = 0;

    int startTime = millis();

    while(true){
        if(this->RFSerialPort->available() > 0){
            uint8_t data = this->RFSerialPort->read();

            if(finish == true){
                msg.rssiValue = data;
                msg.rssiDbm = -((float)data) / 2.0f;
                break;
            }

            if(highCheck == false && data == RF_PACKET_SPEC_HIGH){
                highCheck = true;
            }
            else if(midCheck == false && data == RF_PACKET_SPEC_MID && highCheck == true){
                midCheck = true;
            }
            else if(lowCheck == false && data == RF_PACKET_SPEC_LOW && midCheck == true && highCheck == true){
                lowCheck = true;
            }
            else{
                size++;
                highCheck = false;
                midCheck = false;
                lowCheck = false;
            }

            if(highCheck && midCheck && lowCheck){
                msg.buffer[i] = data;

                if(this->rssiByteSet == false){
                    break;
                }

                finish = true;
                continue;
            }

            msg.buffer[i++] = data;
        }

        if(millis() - startTime > this->_paramConfs.serialTimeout){
            this->DebuggerPort->println(F("Veri Okuma Zaman Asimina Ugradi..."));
            return RF_Msg{E220_Timeout};
        }

        managedDelay(2);
    }

    uint8_t crc = calculateCRC8(msg.buffer, size);

    if(crc != msg.buffer[size - 1]){
        this->DebuggerPort->println(F("Paket CRC Uyuşmuyor"));
        return RF_Msg{E220_CrcBroken};
    }

    msg.crc = crc;
    msg.size = size - 1;

    msg.status = E220_Success;

    return msg;
}

Status E220::send(const uint8_t& AddressHigh, const uint8_t& AddressLow, const uint8_t& Channel, const uint8_t* data, int size){
    if(size > this->maxTxBufferSize - 7){
        this->DebuggerPort->println(F("Paket Boyutu Cok Buyuk !!!"));
        return E220_BigPacket;
    }

    uint8_t buffer[size + 7];
    buffer[0] = AddressHigh;
    buffer[1] = AddressLow;
    buffer[2] = Channel;
    memcpy(&buffer[3], data, size);

    uint8_t crc = calculateCRC8(buffer, size + 3);
    buffer[size + 3] = crc;
    buffer[size + 4] = RF_PACKET_SPEC_HIGH;
    buffer[size + 5] = RF_PACKET_SPEC_MID;
    buffer[size + 6] = RF_PACKET_SPEC_LOW;

    time_t packageTime = calculatePacketSendTime(size + 7);

    RF_WaitAUX();

    this->_paramConfs.packetStartTimeStamp = millis();
    this->_paramConfs.packetEndTimeStamp = this->_paramConfs.packetStartTimeStamp + packageTime;
    this->RFSerialPort->write((uint8_t *)buffer, size + 7);

    RF_WaitAUX();

    managedDelay(5);

    clearSerialBuffer();

    return E220_Success;
}

Status E220::findLeastFrequency(){
    RF_ONOFF tempRssiAmbient = this->_devConfs.RFReg1.RSSIAmbient;
    RF_FREQ tempFreq = this->_devConfs.Channel;
    float minRssi = -500.0f;
    RF_FREQ maxFreq = FREQ_410;

    this->_devConfs.RFReg1.RSSIAmbient = RF_ON;

    DebuggerPort->println(F("-------------------------------------------------------"));
    DebuggerPort->println(F("Frekans Tarama Basladi..."));

    for(uint8_t i = FREQ_410; i <= FREQ_491; i++){
        float tempRssi = checkFreq((RF_FREQ)i);

        if(tempRssi > minRssi){
            minRssi = tempRssi;
            maxFreq = (RF_FREQ)i;
        }

        DebuggerPort->print(410 + i); DebuggerPort->print(F(" MHz --- "));
        DebuggerPort->print(tempRssi); DebuggerPort->println(F(" dBm"));

        managedDelay(100);
    }

    DebuggerPort->println();

    DebuggerPort->print(F("En Düşük RSSI Değerine Sahip Frekans: "));
    DebuggerPort->print(410 + maxFreq); DebuggerPort->print(F(" MHz --- "));
    DebuggerPort->print(minRssi); DebuggerPort->println(F(" dBm"));

    DebuggerPort->println(F("-------------------------------------------------------"));
}

float E220::checkFreq(const RF_FREQ &freq){
    uint8_t sendBuffer[6] = {0xC0, 0xC1, 0xC2, 0xC3, 0x00, 1};
    uint8_t buffer[4];
    float rssiValue = 0.0f;

    this->_devConfs.Channel = freq;

    this->setSettings();

    managedDelay(100);

    this->RFSerialPort->write((uint8_t *)sendBuffer, sizeof(sendBuffer) / sizeof(sendBuffer[0]));

    long startTime = millis();
    while(RFSerialPort->available() < sizeof(buffer)){
        if(millis() - startTime > this->_paramConfs.serialTimeout){
            return E220_Timeout;
        }
        managedDelay(20);
    }

    RFSerialPort->readBytes(buffer, sizeof(buffer));

    rssiValue = -1.0f * (float)buffer[3] / 2.0f;
}