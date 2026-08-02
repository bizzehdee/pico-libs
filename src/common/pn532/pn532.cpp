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

#include "pn532.h"
#include <pico/time.h>
#include <string.h>

#define PN532_PREAMBLE 0x00
#define PN532_STARTCODE1 0x00
#define PN532_STARTCODE2 0xFF
#define PN532_POSTAMBLE 0x00
#define PN532_HOSTTOPN532 0xD4

#define PN532_COMMAND_GETFIRMWAREVERSION 0x02
#define PN532_COMMAND_SAMCONFIGURATION 0x14
#define PN532_COMMAND_RFCONFIGURATION 0x32
#define PN532_COMMAND_INLISTPASSIVETARGET 0x4A
#define PN532_COMMAND_INDATAEXCHANGE 0x40

#define PN532_I2C_READY 0x01

#define MIFARE_CMD_AUTH_A 0x60
#define MIFARE_CMD_AUTH_B 0x61
#define MIFARE_CMD_READ 0x30
#define MIFARE_CMD_WRITE 0xA0

static const uint8_t pn532ack[6] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
static const uint8_t pn532response_firmwarevers[6] = {0x00, 0x00, 0xFF, 0x06, 0xFA, 0xD5};

bool PN532::begin(i2c_inst_t *i2cInst, uint8_t addr)
{
   i2c_dev = new I2CDevice(addr, i2cInst);

   // the PN532 is asleep at power-on and won't ACK an address probe
   if (!i2c_dev->begin(false))
   {
      return false;
   }

   sleep_ms(10);

   // the PN532 clock-stretches the I2C bus during SAMConfig as its wakeup
   return SAMConfig();
}

bool PN532::sendCommandCheckAck(uint8_t *cmd, uint8_t cmdlen, uint16_t timeout)
{
   writecommand(cmd, cmdlen);

   sleep_ms(1);

   if (!waitready(timeout))
   {
      return false;
   }

   if (!readack())
   {
      return false;
   }

   sleep_ms(1);

   return waitready(timeout);
}

void PN532::writecommand(uint8_t *cmd, uint8_t cmdlen)
{
   uint8_t packet[8 + cmdlen];
   uint8_t len = cmdlen + 1;

   packet[0] = PN532_PREAMBLE;
   packet[1] = PN532_STARTCODE1;
   packet[2] = PN532_STARTCODE2;
   packet[3] = len;
   packet[4] = ~len + 1;
   packet[5] = PN532_HOSTTOPN532;

   uint8_t sum = 0;
   for (uint8_t i = 0; i < cmdlen; i++)
   {
      packet[6 + i] = cmd[i];
      sum += cmd[i];
   }
   packet[6 + cmdlen] = ~(PN532_HOSTTOPN532 + sum) + 1;
   packet[7 + cmdlen] = PN532_POSTAMBLE;

   i2c_dev->write(packet, 8 + cmdlen);
}

void PN532::readdata(uint8_t *buff, uint8_t n)
{
   uint8_t rbuff[n + 1]; // +1 for the leading RDY byte

   i2c_dev->read(rbuff, n + 1);
   for (uint8_t i = 0; i < n; i++)
   {
      buff[i] = rbuff[i + 1];
   }
}

bool PN532::isready(void)
{
   uint8_t rdy = 0;
   i2c_dev->read(&rdy, 1);
   return rdy == PN532_I2C_READY;
}

bool PN532::waitready(uint16_t timeout)
{
   uint16_t timer = 0;
   while (!isready())
   {
      if (timeout != 0)
      {
         timer += 10;
         if (timer > timeout)
         {
            return false;
         }
      }
      sleep_ms(10);
   }
   return true;
}

bool PN532::readack(void)
{
   uint8_t ackbuff[6];
   readdata(ackbuff, 6);
   return memcmp(ackbuff, pn532ack, 6) == 0;
}

uint32_t PN532::getFirmwareVersion(void)
{
   packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;

   if (!sendCommandCheckAck(packetbuffer, 1))
   {
      return 0;
   }

   readdata(packetbuffer, 13);

   if (memcmp(packetbuffer, pn532response_firmwarevers, 6) != 0)
   {
      return 0;
   }

   uint32_t response = packetbuffer[7];
   response <<= 8;
   response |= packetbuffer[8];
   response <<= 8;
   response |= packetbuffer[9];
   response <<= 8;
   response |= packetbuffer[10];

   return response;
}

bool PN532::SAMConfig(void)
{
   packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
   packetbuffer[1] = 0x01; // normal mode
   packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
   packetbuffer[3] = 0x01; // matches the reference frame; IRQ pin unused by this I2C-polling port

   if (!sendCommandCheckAck(packetbuffer, 4))
   {
      return false;
   }

   readdata(packetbuffer, 9);

   return packetbuffer[6] == 0x15;
}

bool PN532::setPassiveActivationRetries(uint8_t maxRetries)
{
   packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
   packetbuffer[1] = 5;    // config item 5 (MaxRetries)
   packetbuffer[2] = 0xFF; // MxRtyATR (default)
   packetbuffer[3] = 0x01; // MxRtyPSL (default)
   packetbuffer[4] = maxRetries;

   return sendCommandCheckAck(packetbuffer, 5);
}

bool PN532::readPassiveTargetID(uint8_t cardBaudRate, uint8_t *uid, uint8_t *uidLength, uint16_t timeout)
{
   packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
   packetbuffer[1] = 1; // max 1 card at once
   packetbuffer[2] = cardBaudRate;

   if (!sendCommandCheckAck(packetbuffer, 3, timeout))
   {
      return false;
   }

   readdata(packetbuffer, 20);

   if (packetbuffer[7] != 1)
   {
      return false;
   }

   *uidLength = packetbuffer[12];
   for (uint8_t i = 0; i < *uidLength; i++)
   {
      uid[i] = packetbuffer[13 + i];
   }

   return true;
}

bool PN532::mifareclassic_AuthenticateBlock(uint8_t *uid, uint8_t uidLen, uint32_t blockNumber, uint8_t keyNumber, uint8_t *keyData)
{
   packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
   packetbuffer[1] = 1; // card number
   packetbuffer[2] = keyNumber ? MIFARE_CMD_AUTH_B : MIFARE_CMD_AUTH_A;
   packetbuffer[3] = (uint8_t)blockNumber;

   memcpy(packetbuffer + 4, keyData, 6);
   for (uint8_t i = 0; i < uidLen; i++)
   {
      packetbuffer[10 + i] = uid[i];
   }

   if (!sendCommandCheckAck(packetbuffer, 10 + uidLen))
   {
      return false;
   }

   readdata(packetbuffer, 12);

   return packetbuffer[7] == 0x00;
}

bool PN532::mifareclassic_ReadDataBlock(uint8_t blockNumber, uint8_t *data)
{
   packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
   packetbuffer[1] = 1; // card number
   packetbuffer[2] = MIFARE_CMD_READ;
   packetbuffer[3] = blockNumber;

   if (!sendCommandCheckAck(packetbuffer, 4))
   {
      return false;
   }

   readdata(packetbuffer, 26);

   if (packetbuffer[7] != 0x00)
   {
      return false;
   }

   memcpy(data, packetbuffer + 8, 16);

   return true;
}

bool PN532::mifareclassic_WriteDataBlock(uint8_t blockNumber, uint8_t *data)
{
   packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
   packetbuffer[1] = 1; // card number
   packetbuffer[2] = MIFARE_CMD_WRITE;
   packetbuffer[3] = blockNumber;
   memcpy(packetbuffer + 4, data, 16);

   if (!sendCommandCheckAck(packetbuffer, 20))
   {
      return false;
   }

   sleep_ms(10);

   readdata(packetbuffer, 26);

   return true;
}
