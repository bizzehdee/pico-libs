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

#ifndef __PCF8523_H__
#define __PCF8523_H__

#include "rtc_i2c.h"

#define PCF8523_ADDR 0x68 //!< Default I2C address

//! PCF8523 INT/SQW pin mode settings
enum pcf8523_sqw_pin_mode_t
{
   PCF8523_SQW_OFF = 7,      //!< Off
   PCF8523_SQW_1HZ = 6,      //!< 1Hz square wave
   PCF8523_SQW_32HZ = 5,     //!< 32Hz square wave
   PCF8523_SQW_1024HZ = 4,   //!< 1.024kHz square wave
   PCF8523_SQW_4096HZ = 3,   //!< 4.096kHz square wave
   PCF8523_SQW_8192HZ = 2,   //!< 8.192kHz square wave
   PCF8523_SQW_16384HZ = 1,  //!< 16.384kHz square wave
   PCF8523_SQW_32768HZ = 0,  //!< 32.768kHz square wave
};

class PCF8523 : RTCI2C
{
public:
   bool begin(i2c_inst_t *i2cInst = i2c0, uint8_t addr = PCF8523_ADDR);
   bool rtc_running();
   bool lostPower();
   bool rtc_set_datetime(datetime_t *t);
   bool rtc_get_datetime(datetime_t *t);

   void start();
   void stop();

   pcf8523_sqw_pin_mode_t readSqwPinMode();
   void writeSqwPinMode(pcf8523_sqw_pin_mode_t mode);
};

#endif // __PCF8523_H__
