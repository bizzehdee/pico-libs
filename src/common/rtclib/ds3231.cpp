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

#include "ds3231.h"

#define DS3231_REG_TIME 0x00
#define DS3231_REG_CONTROL 0x0E
#define DS3231_REG_STATUS 0x0F
#define DS3231_REG_TEMPERATURE 0x11 // high byte - low byte is at 0x12

bool DS3231::begin(i2c_inst_t *i2cInst, uint8_t addr)
{
   i2c_dev = new I2CDevice(addr, i2cInst);
   return i2c_dev->begin();
}

bool DS3231::rtc_running()
{
   return !(readRegister(DS3231_REG_STATUS) >> 7);
}

bool DS3231::rtc_set_datetime(datetime_t *t)
{
   if (t == NULL)
      return false;

   uint8_t buffer[8] = {DS3231_REG_TIME,
                        bin2bcd(t->sec),
                        bin2bcd(t->min),
                        bin2bcd(t->hour),
                        bin2bcd(1), // day of week, unused
                        bin2bcd(t->day),
                        bin2bcd(t->month),
                        bin2bcd((uint8_t)(t->year - 2000))};

   if (!i2c_dev->write(buffer, 8))
      return false;

   // the time has been set, so clear the oscillator stop flag
   uint8_t statreg = readRegister(DS3231_REG_STATUS);
   writeRegister(DS3231_REG_STATUS, statreg & ~0x80);

   return true;
}

bool DS3231::rtc_get_datetime(datetime_t *t)
{
   if (t == NULL)
      return false;

   uint8_t readBuffer[7] = {0, 0, 0, 0, 0, 0, 0};

   if (!i2c_dev->write_then_read(readBuffer, 1, readBuffer, 7))
      return false;

   t->sec = bcd2bin(readBuffer[0] & 0x7F);
   t->min = bcd2bin(readBuffer[1]);
   t->hour = bcd2bin(readBuffer[2]);
   t->day = bcd2bin(readBuffer[4]);
   t->month = bcd2bin(readBuffer[5] & 0x7F);
   t->year = bcd2bin(readBuffer[6]) + 2000;

   return true;
}

float DS3231::getTemperature()
{
   uint8_t buffer[2] = {DS3231_REG_TEMPERATURE, 0};
   i2c_dev->write_then_read(buffer, 1, buffer, 2);
   return (float)buffer[0] + (buffer[1] >> 6) * 0.25f;
}

ds3231_sqw_pin_mode_t DS3231::readSqwPinMode()
{
   uint8_t mode = readRegister(DS3231_REG_CONTROL) & 0x1C;
   if (mode & 0x04)
      mode = DS3231_SQW_OFF;
   return static_cast<ds3231_sqw_pin_mode_t>(mode);
}

void DS3231::writeSqwPinMode(ds3231_sqw_pin_mode_t mode)
{
   uint8_t ctrl = readRegister(DS3231_REG_CONTROL);

   ctrl &= ~0x04; // turn off INTCON
   ctrl &= ~0x18; // clear rate select bits

   writeRegister(DS3231_REG_CONTROL, ctrl | mode);
}
