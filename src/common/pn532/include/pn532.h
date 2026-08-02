/* Original Source Ported From https://github.com/adafruit/Adafruit-PN532 */
/*
BSD 3-Clause License

Copyright (c) 2022, Darren Horrocks
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __PN532_H__
#define __PN532_H__

#include "hardware/i2c.h"
#include "i2c_device.h"

#define PN532_I2C_ADDR 0x24 //!< Default I2C address (0x48 >> 1, 7-bit form)

#define PN532_MIFARE_ISO14443A 0x00 //!< Card baud rate for readPassiveTargetID(): 106 kbps type A

//! Driver for the Adafruit PN532 NFC/RFID breakout, I2C only. Covers chip
//! bring-up, ISO14443A tag discovery and Mifare Classic block read/write --
//! the SPI/UART transports and the NDEF/Mifare Ultralight/NTAG2xx/target-mode
//! helpers from the reference library are not ported.
class PN532
{
public:
   bool begin(i2c_inst_t *i2cInst = i2c0, uint8_t addr = PN532_I2C_ADDR);

   uint32_t getFirmwareVersion(void);
   bool SAMConfig(void);
   bool setPassiveActivationRetries(uint8_t maxRetries);

   // ISO14443A functions
   bool readPassiveTargetID(uint8_t cardBaudRate, uint8_t *uid, uint8_t *uidLength, uint16_t timeout = 1000);

   // Mifare Classic functions
   bool mifareclassic_AuthenticateBlock(uint8_t *uid, uint8_t uidLen, uint32_t blockNumber, uint8_t keyNumber, uint8_t *keyData);
   bool mifareclassic_ReadDataBlock(uint8_t blockNumber, uint8_t *data);
   bool mifareclassic_WriteDataBlock(uint8_t blockNumber, uint8_t *data);

private:
   bool sendCommandCheckAck(uint8_t *cmd, uint8_t cmdlen, uint16_t timeout = 1000);
   void writecommand(uint8_t *cmd, uint8_t cmdlen);
   void readdata(uint8_t *buff, uint8_t n);
   bool isready(void);
   bool waitready(uint16_t timeout);
   bool readack(void);

   I2CDevice *i2c_dev = NULL;
   uint8_t packetbuffer[64];
};

#endif // __PN532_H__
