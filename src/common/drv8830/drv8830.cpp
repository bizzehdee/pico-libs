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

#include "drv8830.h"
#include "bus_register.h"
#include "bus_register_bits.h"

bool DRV8830::begin(i2c_inst_t *i2cInst, uint8_t addr)
{
   i2c_dev = new I2CDevice(addr, i2cInst);
   return i2c_dev->begin();
}

bool DRV8830::setSpeed(uint8_t throttle)
{
   uint8_t dac = ((uint16_t)throttle * 0x3F) / 255;
   if (dac < 6)
      dac = 6;

   BusRegister ctrl_reg(i2c_dev, DRV8830_REG_CONTROL);
   BusRegisterBits dac_bits(&ctrl_reg, 6, 2); // #bits, bit_shift

   return dac_bits.write(dac);
}

bool DRV8830::run(drv8830_dir_t dir)
{
   BusRegister ctrl_reg(i2c_dev, DRV8830_REG_CONTROL);
   BusRegisterBits dir_bits(&ctrl_reg, 2, 0); // #bits, bit_shift

   return dir_bits.write((uint8_t)dir);
}

uint8_t DRV8830::getFaults(void)
{
   BusRegister fault_reg(i2c_dev, DRV8830_REG_FAULT);
   return (uint8_t)fault_reg.read() & 0x1F;
}

bool DRV8830::clearFaults(void)
{
   BusRegister fault_reg(i2c_dev, DRV8830_REG_FAULT);
   BusRegisterBits clear_bit(&fault_reg, 1, 7); // #bits, bit_shift

   return clear_bit.write(1);
}
