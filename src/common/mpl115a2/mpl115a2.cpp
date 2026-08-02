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

#include "mpl115a2.h"
#include <pico/time.h>

bool MPL115A2::begin(i2c_inst_t *i2cInst, uint8_t addr)
{
   i2c_dev = new I2CDevice(addr, i2cInst);
   if (!i2c_dev->begin())
   {
      return false;
   }

   // read factory coefficient values (this only needs to be done once)
   readCoefficients();
   return true;
}

void MPL115A2::readCoefficients(void)
{
   uint8_t cmd = MPL115A2_REG_A0_COEFF_MSB;
   uint8_t buffer[8];
   i2c_dev->write_then_read(&cmd, 1, buffer, 8);

   int16_t a0coeff = ((uint16_t)buffer[0] << 8) | buffer[1];
   int16_t b1coeff = ((uint16_t)buffer[2] << 8) | buffer[3];
   int16_t b2coeff = ((uint16_t)buffer[4] << 8) | buffer[5];
   int16_t c12coeff = (((uint16_t)buffer[6] << 8) | buffer[7]) >> 2;

   a0 = (float)a0coeff / 8;
   b1 = (float)b1coeff / 8192;
   b2 = (float)b2coeff / 16384;
   c12 = (float)c12coeff / 4194304.0f;
}

float MPL115A2::getPressure(void)
{
   float pressureComp, centigrade;
   getPT(&pressureComp, &centigrade);
   return pressureComp;
}

float MPL115A2::getTemperature(void)
{
   float pressureComp, centigrade;
   getPT(&pressureComp, &centigrade);
   return centigrade;
}

void MPL115A2::getPT(float *P, float *T)
{
   uint8_t cmd[2] = {MPL115A2_REG_STARTCONVERSION, 0};
   i2c_dev->write(cmd, 2);

   // wait for the conversion to complete (3ms max)
   sleep_ms(5);

   uint8_t buffer[4];
   cmd[0] = MPL115A2_REG_PRESSURE_MSB;
   i2c_dev->write_then_read(cmd, 1, buffer, 4);

   uint16_t pressure = (((uint16_t)buffer[0] << 8) | buffer[1]) >> 6;
   uint16_t temp = (((uint16_t)buffer[2] << 8) | buffer[3]) >> 6;

   // see datasheet p.6 for evaluation sequence
   float pressureComp = a0 + (b1 + c12 * temp) * pressure + b2 * temp;

   *P = ((65.0f / 1023.0f) * pressureComp) + 50.0f; // kPa
   *T = ((float)temp - 498.0f) / -5.35f + 25.0f;    // Celsius
}
