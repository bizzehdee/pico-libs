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

#include <math.h>
#include <cstdio>
#include "bh1750.h"

BH1750::BH1750(i2c_inst_t *i2cInst, uint8_t addr)
{
   i2c_dev = new I2CDevice(addr, i2cInst);
   _mode = CONTINUOUS_HIGH_RES_MODE;
}

bool BH1750::init(MODE mode, uint8_t mtreg)
{
   return !this->mode(mode) && !this->mtreg(mtreg);
}

uint8_t BH1750::mode(MODE mode)
{
   uint8_t ack = 5;

   uint8_t write_buffer[1] = {mode};
   uint8_t read_buffer[1] = {5};

   i2c_dev->write_then_read(write_buffer, 1, read_buffer, 1);

   sleep_ms(10);

   ack = read_buffer[0];

   if (ack == 0)
   {
      _mode = mode;
   }

   return ack;
}

uint8_t BH1750::mtreg(uint8_t mtreg)
{
   uint8_t ack = 5;

   uint8_t write_buffer[1] = {0};
   uint8_t read_buffer[1] = {5};

   write_buffer[0] = (0b01000 << 3) | (mtreg >> 5);
   i2c_dev->write_then_read(write_buffer, 1, read_buffer, 1);

   ack = read_buffer[0];

   write_buffer[0] = ((0b011 << 5) | (mtreg & 0b11111));
   i2c_dev->write_then_read(write_buffer, 1, read_buffer, 1);

   ack = ack | read_buffer[0];

   write_buffer[0] = _mode;
   i2c_dev->write_then_read(write_buffer, 1, read_buffer, 1);

   ack = ack | read_buffer[0];

   if (ack == 0)
   {
      _mtreg = mtreg;
   }

   sleep_ms(10);

   return ack;
}

uint16_t BH1750::raw()
{
   uint8_t buffer[2];
   i2c_dev->read((uint8_t *)buffer, 2);

   return buffer[0] << 8 | buffer[1];
}

float BH1750::lux()
{
   uint16_t raw_lux = raw();
   return rawToLux(raw_lux);
}

float BH1750::rawToLux(uint16_t raw)
{
   float measured_lux = static_cast<float>(raw);

   if (_mtreg != _BH1750_DEFAULT_MTREG)
   {
      measured_lux = measured_lux * (float)((uint8_t)_BH1750_DEFAULT_MTREG / (float)_mtreg);
   }

   if (_mode == ONE_TIME_HIGH_RES_MODE_2 ||
       _mode == CONTINUOUS_HIGH_RES_MODE_2)
   {
      measured_lux /= 2;
   }

   measured_lux = measured_lux / 1.2;

   return measured_lux;
}