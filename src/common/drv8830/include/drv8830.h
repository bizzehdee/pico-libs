/* Original Source Ported From https://github.com/adafruit/Adafruit_DRV8830 */
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

#ifndef __DRV8830_H__
#define __DRV8830_H__

#include "hardware/i2c.h"
#include "i2c_device.h"

#define DRV8830_ADDR 0x60 //!< Default I2C address

#define DRV8830_REG_CONTROL 0x00 //!< Control register
#define DRV8830_REG_FAULT 0x01   //!< Fault condition register

#define DRV8830_FAULT_ILIMIT 0x10   //!< Current limiting
#define DRV8830_FAULT_OVERTEMP 0x08 //!< Over temperature shutdown
#define DRV8830_FAULT_UVLOW 0x04    //!< Undervoltage lockout
#define DRV8830_FAULT_OVERCURR 0x02 //!< Overcurrent protection

//! Motor direction / H-bridge configuration
typedef enum
{
   DRV8830_RELEASE = 0,
   DRV8830_BACKWARD = 1,
   DRV8830_FORWARD = 2,
   DRV8830_BRAKE = 3,
} drv8830_dir_t;

class DRV8830
{
public:
   bool begin(i2c_inst_t *i2cInst = i2c0, uint8_t addr = DRV8830_ADDR);

   bool setSpeed(uint8_t throttle); // 0-255, mapped to the DRV8830's 6-bit PWM/DAC range
   bool run(drv8830_dir_t dir);

   uint8_t getFaults(void);
   bool clearFaults(void);

private:
   I2CDevice *i2c_dev = NULL;
};

#endif // __DRV8830_H__
