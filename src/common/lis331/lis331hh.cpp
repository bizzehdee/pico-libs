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

#include "lis331hh.h"
#include "bus_register.h"

bool LIS331HH::begin(i2c_inst_t *i2cInst, uint8_t addr)
{
   if (!LIS331::begin(i2cInst, addr))
   {
      return false;
   }

   BusRegister ctrl1_reg(i2c_dev, LIS331_REG_CTRL1);
   ctrl1_reg.write(0x07); // enable all axes, normal mode

   enableHighPassFilter(false);
   setDataRate(LIS331_DATARATE_1000_HZ);
   setRange(LIS331HH_RANGE_24_G);

   return true;
}

void LIS331HH::setRange(lis331hh_range_t range)
{
   writeRange((uint8_t)range);
}

lis331hh_range_t LIS331HH::getRange(void)
{
   return (lis331hh_range_t)readRange();
}

void LIS331HH::scaleValues(void)
{
   // actually 12 bit but left justified
   x >>= 4;
   y >>= 4;
   z >>= 4;

   uint16_t scale_max = 1;
   switch (getRange())
   {
   case LIS331HH_RANGE_6_G:
      scale_max = 6;
      break;
   case LIS331HH_RANGE_12_G:
      scale_max = 12;
      break;
   case LIS331HH_RANGE_24_G:
      scale_max = 24;
      break;
   }

   float lsb_value = 2 * scale_max * (float)1 / 4096;

   x_g = (float)x * lsb_value;
   y_g = (float)y * lsb_value;
   z_g = (float)z * lsb_value;
}
