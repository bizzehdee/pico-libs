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

#include "ds1307.h"
#include <stdio.h>

#define bcd2bin(x) (((x)&0x0f) + ((x) >> 4) * 10)
#define bin2bcd(x) ((((x) / 10) << 4) + (x) % 10)

bool DS1307::begin(i2c_inst_t *i2cInst, uint8_t addr)
{
   i2c_dev = new I2CDevice(addr, i2cInst);
   return i2c_dev->begin();
}

bool DS1307::rtc_running()
{
   uint8_t readBuffer[1] = {0};
   i2c_dev->write(readBuffer, 1);
   i2c_dev->read(readBuffer, 1);
   return !(readBuffer[0] >> 7);
}

bool DS1307::rtc_set_datetime(datetime_t *t)
{
   if (t == NULL)
      return false;

   uint8_t buffer[8] = {0x00,
                        bin2bcd(t->sec),
                        bin2bcd(t->min),
                        bin2bcd(t->hour),
                        0x00,
                        bin2bcd(t->day),
                        bin2bcd(t->month),
                        bin2bcd((uint8_t)(t->year - 2000))};

   return i2c_dev->write(buffer, 8);
}

bool DS1307::rtc_get_datetime(datetime_t *t)
{
   if (t == NULL)
      return false;

   uint8_t readBuffer[7] = {0, 0, 0, 0, 0, 0, 0};

   if (i2c_dev->write_then_read(readBuffer, 1, readBuffer, 7))
   {
      t->sec = bcd2bin(readBuffer[0]);
      t->min = bcd2bin(readBuffer[1]);
      t->hour = bcd2bin(readBuffer[2]);
      t->day = bcd2bin(readBuffer[4]);
      t->month = bcd2bin(readBuffer[5]);
      t->year = bcd2bin(readBuffer[6]) + 2000u;
      return true;
   }

   return false;
}