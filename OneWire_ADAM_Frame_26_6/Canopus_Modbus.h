#ifndef Canopus_Modbus_h
#define Canopus_Modbus_h

#define KIT_DEV_CANOPUS

#ifdef KIT_DEV_CANOPUS
#define Serial_Canopus Serial1
#endif

#define __MODBUSMASTER_DEBUG__ (0)

#include "Arduino.h"
#include "util/crc16.h"
#include "util/word.h"

#define REGISTER 0
#define REDE_RS485 -1
#define RS485_Tranmit digitalWrite(REDE_RS485, HIGH) 
#define RS485_Receive digitalWrite(REDE_RS485, LOW)

class ModbusMaster
{
  public:
    ModbusMaster();

    void begin(uint8_t, HardwareSerial &serial);
    void idle(void (*)());

    // Modbus exception codes
    static const uint8_t ku8MBIllegalFunction = 0x01;
    static const uint8_t ku8MBIllegalDataAddress = 0x02;
    static const uint8_t ku8MBIllegalDataValue = 0x03;
    static const uint8_t ku8MBSlaveDeviceFailure = 0x04;

    static const uint8_t ku8MBSuccess = 0x00;
    static const uint8_t ku8MBInvalidSlaveID = 0xE0;
    static const uint8_t ku8MBInvalidFunction = 0xE1;
    static const uint8_t ku8MBResponseTimedOut = 0xE2;
    static const uint8_t ku8MBInvalidCRC = 0xE3;

    uint16_t getResponseBuffer(uint8_t);
    void clearResponseBuffer();
    uint8_t setTransmitBuffer(uint8_t, uint16_t);
    void clearTransmitBuffer();

    void beginTransmission(uint16_t);
    uint8_t requestFrom(uint16_t, uint16_t);
    void sendBit(bool);
    void send(uint8_t);
    void send(uint16_t);
    void send(uint32_t);
    uint8_t available(void);
    uint16_t receive(void);

    uint8_t readCoils(uint16_t, uint16_t);
    uint8_t readHoldingRegisters(uint16_t, uint16_t);
    uint8_t readInputRegisters(uint16_t, uint8_t);
    uint8_t readDiscreteInputs(uint16_t, uint16_t); // Updated to match the definition

  private:
    HardwareSerial* _serial;
    uint8_t _u8MBSlave;
    static const uint8_t ku8MaxBufferSize = 64;
    uint16_t _u16ReadAddress;
    uint16_t _u16ReadQty;
    uint16_t _u16ResponseBuffer[ku8MaxBufferSize];
    uint16_t _u16WriteAddress;
    uint16_t _u16WriteQty;
    uint16_t _u16TransmitBuffer[ku8MaxBufferSize];
    uint16_t* txBuffer;
    uint8_t _u8TransmitBufferIndex;
    uint16_t u16TransmitBufferLength;
    uint16_t* rxBuffer;
    uint8_t _u8ResponseBufferIndex;
    uint8_t _u8ResponseBufferLength;

    static const uint8_t ku8MBReadCoils = 0x01;
    static const uint8_t ku8MBReadDiscreteInputs = 0x02;
    static const uint8_t ku8MBWriteSingleCoil = 0x05;
    static const uint8_t ku8MBWriteMultipleCoils = 0x0F;

    static const uint8_t ku8MBReadHoldingRegisters = 0x03;
    static const uint8_t ku8MBReadInputRegisters = 0x04;
    static const uint8_t ku8MBWriteSingleRegister = 0x06;
    static const uint8_t ku8MBWriteMultipleRegisters = 0x10;
    static const uint8_t ku8MBMaskWriteRegister = 0x16;
    static const uint8_t ku8MBReadWriteMultipleRegisters = 0x17;

    static const uint16_t ku16MBResponseTimeout = 200;

    uint8_t ModbusMasterTransaction(uint8_t u8MBFunction);

    void (*_idle)();
    void (*_preTransmission)();
    void (*_postTransmission)();
};
#endif
