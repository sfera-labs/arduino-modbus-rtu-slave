/*
  ModbusRtuSlave.h - Modbus RTU Slave library for Arduino

    Copyright (C) 2018 Sfera Labs S.r.l. - All rights reserved.

  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  See file LICENSE.txt for further informations on licensing terms.
*/

#ifndef ModbusRtuSlave_h
#define ModbusRtuSlave_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define MB_FC_READ_COILS 0x01
#define MB_FC_READ_DISCRETE_INPUTS 0x02
#define MB_FC_READ_HOLDING_REGISTERS 0x03
#define MB_FC_READ_INPUT_REGISTER 0x04
#define MB_FC_WRITE_SINGLE_COIL 0x05
#define MB_FC_WRITE_SINGLE_REGISTER 0x06
#define MB_FC_WRITE_MULTIPLE_COILS 0x0F
#define MB_FC_WRITE_MULTIPLE_REGISTERS 0x10

#define MB_EX_ILLEGAL_FUNCTION 0x01
#define MB_EX_ILLEGAL_DATA_ADDRESS 0x02
#define MB_EX_ILLEGAL_DATA_VALUE 0x03
#define MB_EX_SERVER_DEVICE_FAILURE 0x04
#define MB_EX_ACKNOWLEDGE 0x05
#define MB_EX_SERVER_DEVICE_BUSY 0x06
#define MB_EX_MEMORY_PARITY_ERROR 0x08
#define MB_EX_GATEWAY_PATH_UNAVAILABLE 0x0A
#define MB_EX_DEVICE_FAILED_TO_RESPOND 0x0B

#define MB_RESP_OK 0x00
#define MB_RESP_IGNORE 0xFF

#define MODBUS_BUFFER_SIZE 64

class ModbusRtuSlaveClass {
  public:
    typedef byte Callback(byte, byte, word, word, byte*);
    static void begin(byte unitAddr, Stream *serial, unsigned long baud, int txEnPin);
    static void setCallback(Callback *callback);
    static void process();
    static void responseAddBit(bool on);
    static void responseAddRegister(word value);
    static bool getDataCoil(byte function, byte* data, unsigned int idx);
    static word getDataRegister(byte function, byte* data, unsigned int idx);

  private:
    static byte _unitAddr;
    static Stream *_port;
    static int _txEnPin;
    static uint16_t _last_available;
    static unsigned long _last_available_ts;
    static unsigned long _t35chars;
    static byte _inBuff[MODBUS_BUFFER_SIZE];
    static byte _outBuff[MODBUS_BUFFER_SIZE];
    static Callback *_callback;
    static int _respOffset;
};

extern ModbusRtuSlaveClass ModbusRtuSlave;

#endif
