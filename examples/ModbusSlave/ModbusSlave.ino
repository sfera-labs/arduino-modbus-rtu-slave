/*  
  ModbusSlave.ino - Arduino sketch showing the use of the ModbusRtuSlave library

    Copyright (C) 2018-2022 Sfera Labs S.r.l. - All rights reserved.

    For information, see:
    https://www.sferalabs.cc/
  
  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  See file LICENSE.txt for further informations on licensing terms.
*/

#include <ModbusRtuSlave.h>

#define SERIAL_PORT SERIAL_PORT_HARDWARE
#define BAUD 115200

bool myCoil = false;
word myRegister = 1000;

void setup() {
  /**
   * Init serial port:
   * baud rate: BAUD
   * data bits: 8
   * parity: even
   * stop bits: 1
   */
  SERIAL_PORT.begin(BAUD, SERIAL_8E1);

  /**
   * Set callback function to handle requests
   */
  ModbusRtuSlave.setCallback(&onRequest);

  /*
   * Start the modbus server with: 
   * unit address: 20
   * TX enable pin: 4
   */
  ModbusRtuSlave.begin(20, &SERIAL_PORT, BAUD, 4);
}

void loop() {
  ModbusRtuSlave.process();
}

byte onRequest(byte unitAddr, byte function, word regAddr, word qty, byte *data) {
  switch (function) {
    case MB_FC_READ_COILS:
      if (regAddr == 1 && qty == 1) {
        ModbusRtuSlave.responseAddBit(myCoil);
        return MB_RESP_OK;
      }
      return MB_EX_ILLEGAL_DATA_ADDRESS;

    case MB_FC_READ_HOLDING_REGISTERS:
      if (regAddr == 101 && qty == 1) {
        ModbusRtuSlave.responseAddRegister(myRegister);
        return MB_RESP_OK;
      }
      return MB_EX_ILLEGAL_DATA_ADDRESS;

    case MB_FC_WRITE_SINGLE_COIL:
      if (regAddr == 1) {
        myCoil = ModbusRtuSlave.getDataCoil(function, data, 0);
        return MB_RESP_OK;
      }
      return MB_EX_ILLEGAL_DATA_ADDRESS;

    case MB_FC_WRITE_SINGLE_REGISTER:
      if (regAddr == 101) {
        word value = ModbusRtuSlave.getDataRegister(function, data, 0);
        if (value < 0 || value > 10000) {
          return MB_EX_ILLEGAL_DATA_VALUE;
        }
        myRegister = value;
        return MB_RESP_OK;
      }
      return MB_EX_ILLEGAL_DATA_ADDRESS;

    default:
      return MB_EX_ILLEGAL_FUNCTION;
  }
}
