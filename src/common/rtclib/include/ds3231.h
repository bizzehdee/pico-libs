/* Original Source Ported From https://github.com/adafruit/RTClib */
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

#ifndef __DS3231_H__
#define __DS3231_H__

#include "rtc_i2c.h"

#define DS3231_ADDR 0x68 //!< Default I2C address

//! DS3231 SQW pin mode settings
enum ds3231_sqw_pin_mode_t
{
   DS3231_SQW_OFF = 0x1C,   //!< Off
   DS3231_SQW_1HZ = 0x00,   //!< 1Hz square wave
   DS3231_SQW_1024HZ = 0x08, //!< 1.024kHz square wave
   DS3231_SQW_4096HZ = 0x10, //!< 4.096kHz square wave
   DS3231_SQW_8192HZ = 0x18, //!< 8.192kHz square wave
};

class DS3231 : RTCI2C
{
public:
   bool begin(i2c_inst_t *i2cInst = i2c0, uint8_t addr = DS3231_ADDR);
   bool rtc_running();
   bool rtc_set_datetime(datetime_t *t);
   bool rtc_get_datetime(datetime_t *t);

   float getTemperature(); // degrees Celsius

   ds3231_sqw_pin_mode_t readSqwPinMode();
   void writeSqwPinMode(ds3231_sqw_pin_mode_t mode);
};

#endif // __DS3231_H__
