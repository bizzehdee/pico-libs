/* Original Source Ported From https://github.com/adafruit/Adafruit_LIS331 */
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

#ifndef __LIS331HH_H__
#define __LIS331HH_H__

#include "lis331.h"

#define LIS331HH_ADDR 0x18 //!< Default I2C address (if SDO/SA0 is 3V, its 0x19)

//! Selectable measurement ranges
typedef enum
{
   LIS331HH_RANGE_6_G = 0x0,  //!< +/- 6G
   LIS331HH_RANGE_12_G = 0x1, //!< +/- 12G
   LIS331HH_RANGE_24_G = 0x3, //!< +/- 24G
} lis331hh_range_t;

class LIS331HH : public LIS331
{
public:
   bool begin(i2c_inst_t *i2cInst = i2c0, uint8_t addr = LIS331HH_ADDR);

   void setRange(lis331hh_range_t range);
   lis331hh_range_t getRange(void);

protected:
   void scaleValues(void) override;
};

#endif // __LIS331HH_H__
