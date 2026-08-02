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

#include "pcf8523.h"

#define PCF8523_REG_CONTROL_1 0x00
#define PCF8523_REG_CONTROL_3 0x02
#define PCF8523_REG_TIME 0x03 // also doubles as the seconds/OS-flag register
#define PCF8523_REG_CLKOUTCONTROL 0x0F

bool PCF8523::begin(i2c_inst_t *i2cInst, uint8_t addr)
{
   i2c_dev = new I2CDevice(addr, i2cInst);
   return i2c_dev->begin();
}

bool PCF8523::lostPower()
{
   return readRegister(PCF8523_REG_TIME) >> 7;
}

bool PCF8523::rtc_running()
{
   return !((readRegister(PCF8523_REG_CONTROL_1) >> 5) & 1);
}

bool PCF8523::rtc_set_datetime(datetime_t *t)
{
   if (t == NULL)
      return false;

   uint8_t buffer[8] = {PCF8523_REG_TIME,
                        bin2bcd(t->sec),
                        bin2bcd(t->min),
                        bin2bcd(t->hour),
                        bin2bcd(t->day),
                        bin2bcd(0), // day of week, unused
                        bin2bcd(t->month),
                        bin2bcd((uint8_t)(t->year - 2000))};

   if (!i2c_dev->write(buffer, 8))
      return false;

   // set to battery switchover mode
   writeRegister(PCF8523_REG_CONTROL_3, 0x00);

   return true;
}

bool PCF8523::rtc_get_datetime(datetime_t *t)
{
   if (t == NULL)
      return false;

   uint8_t readBuffer[7] = {0, 0, 0, 0, 0, 0, 0};

   if (!i2c_dev->write_then_read(readBuffer, 1, readBuffer, 7))
      return false;

   t->sec = bcd2bin(readBuffer[0] & 0x7F);
   t->min = bcd2bin(readBuffer[1]);
   t->hour = bcd2bin(readBuffer[2]);
   t->day = bcd2bin(readBuffer[3]);
   t->month = bcd2bin(readBuffer[5]);
   t->year = bcd2bin(readBuffer[6]) + 2000;

   return true;
}

void PCF8523::start()
{
   uint8_t ctlreg = readRegister(PCF8523_REG_CONTROL_1);
   if (ctlreg & (1 << 5))
      writeRegister(PCF8523_REG_CONTROL_1, ctlreg & ~(1 << 5));
}

void PCF8523::stop()
{
   writeRegister(PCF8523_REG_CONTROL_1, readRegister(PCF8523_REG_CONTROL_1) | (1 << 5));
}

pcf8523_sqw_pin_mode_t PCF8523::readSqwPinMode()
{
   uint8_t mode = readRegister(PCF8523_REG_CLKOUTCONTROL);
   mode >>= 3;
   mode &= 0x7;
   return static_cast<pcf8523_sqw_pin_mode_t>(mode);
}

void PCF8523::writeSqwPinMode(pcf8523_sqw_pin_mode_t mode)
{
   writeRegister(PCF8523_REG_CLKOUTCONTROL, mode << 3);
}
