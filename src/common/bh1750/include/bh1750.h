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

#ifndef __BH1750_H__
#define __BH1750_H__

#include "hardware/i2c.h"
#include "i2c_device.h"

#define _BH1750_DEVICE_ID 0xE1  // Correct content of WHO_AM_I register

#define _BH1750_DEFAULT_ADDRESS 0x23  // I2C address with ADDR pin low
#define _BH1750_ALT_ADDRESS 0x5C  // I2C address with ADDR pin high

#define _BH1750_MTREG_MIN 31
#define _BH1750_MTREG_MAX 254
#define _BH1750_DEFAULT_MTREG 69

enum MODE {
   // same as Power Down
   UNCONFIGURED = 0,
   // Measurement at 1 lux resolution. Measurement time is approx 120ms.
   CONTINUOUS_HIGH_RES_MODE = 0x10,
   // Measurement at 0.5 lux resolution. Measurement time is approx 120ms.
   CONTINUOUS_HIGH_RES_MODE_2 = 0x11,
   // Measurement at 4 lux resolution. Measurement time is approx 16ms.
   CONTINUOUS_LOW_RES_MODE = 0x13,
   // Measurement at 1 lux resolution. Measurement time is approx 120ms.
   ONE_TIME_HIGH_RES_MODE = 0x20,
   // Measurement at 0.5 lux resolution. Measurement time is approx 120ms.
   ONE_TIME_HIGH_RES_MODE_2 = 0x21,
   // Measurement at 4 lux resolution. Measurement time is approx 16ms.
   ONE_TIME_LOW_RES_MODE = 0x23
};

class BH1750
{
public:
   BH1750(i2c_inst_t *i2cInst, uint8_t addr = _BH1750_DEFAULT_ADDRESS);
   bool init(MODE mode = CONTINUOUS_HIGH_RES_MODE, uint8_t mtreg=_BH1750_DEFAULT_MTREG);
   uint8_t mode(MODE mode);
   uint8_t mtreg(uint8_t mtreg);
   uint16_t raw();
   float lux();
   void setMode(MODE mode);


private:
   float rawToLux(uint16_t raw);
private:
	I2CDevice *i2c_dev = NULL;
   MODE _mode = UNCONFIGURED;
   uint8_t _mtreg = 0;
};

#endif // __BH1750_H__
