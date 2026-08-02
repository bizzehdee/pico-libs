/* Original Source Ported From https://github.com/adafruit/Adafruit_MPL115A2 */
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

#ifndef __MPL115A2_H__
#define __MPL115A2_H__

#include "hardware/i2c.h"
#include "i2c_device.h"

#define MPL115A2_ADDR 0x60 //!< Default I2C address

#define MPL115A2_REG_PRESSURE_MSB 0x00
#define MPL115A2_REG_A0_COEFF_MSB 0x04
#define MPL115A2_REG_STARTCONVERSION 0x12

class MPL115A2
{
public:
   bool begin(i2c_inst_t *i2cInst = i2c0, uint8_t addr = MPL115A2_ADDR);

   float getPressure(void); // kPa
   float getTemperature(void); // Celsius
   void getPT(float *P, float *T);

private:
   I2CDevice *i2c_dev = NULL;

   float a0 = 0.0f;
   float b1 = 0.0f;
   float b2 = 0.0f;
   float c12 = 0.0f;

   void readCoefficients(void);
};

#endif // __MPL115A2_H__
