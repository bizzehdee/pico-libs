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

#include "lis331.h"
#include "bus_register.h"
#include "bus_register_bits.h"
#include <pico/time.h>

bool LIS331::begin(i2c_inst_t *i2cInst, uint8_t addr)
{
   i2c_dev = new I2CDevice(addr, i2cInst);
   if (!i2c_dev->begin())
   {
      return false;
   }

   return getDeviceID() == LIS331_CHIP_ID;
}

uint8_t LIS331::getDeviceID(void)
{
   BusRegister chip_id(i2c_dev, LIS331_REG_WHOAMI);
   return (uint8_t)chip_id.read();
}

void LIS331::read(void)
{
   // set [7] for auto-increment of the register address across the burst read
   BusRegister xyz_data(i2c_dev, LIS331_REG_OUT_X_L | 0x80, 6);

   uint8_t buffer[6];
   xyz_data.read(buffer, 6);

   x = buffer[0];
   x |= ((uint16_t)buffer[1]) << 8;
   y = buffer[2];
   y |= ((uint16_t)buffer[3]) << 8;
   z = buffer[4];
   z |= ((uint16_t)buffer[5]) << 8;

   scaleValues();
}

void LIS331::configIntDataReady(uint8_t irqnum, bool activelow, bool opendrain)
{
   BusRegister ctrl3_reg(i2c_dev, LIS331_REG_CTRL3);

   BusRegisterBits opendrain_and_polarity_bits(&ctrl3_reg, 2, 6);
   BusRegisterBits int1_bits(&ctrl3_reg, 2, 0);
   BusRegisterBits int2_bits(&ctrl3_reg, 2, 3);

   opendrain_and_polarity_bits.write((activelow << 1) | (opendrain));

   if (irqnum == 1)
   {
      int1_bits.write(0b10);
      int2_bits.write(0);
   }
   else
   {
      int2_bits.write(0b10);
      int1_bits.write(0);
   }
}

void LIS331::enableHighPassFilter(bool filter_enabled, lis331_hpf_cutoff_t cutoff, bool use_reference)
{
   BusRegister ctrl2_reg(i2c_dev, LIS331_REG_CTRL2);

   BusRegisterBits hpf_mode(&ctrl2_reg, 1, 5);
   BusRegisterBits hpf_internal_filter_en(&ctrl2_reg, 1, 4);
   BusRegisterBits hpf_cutoff(&ctrl2_reg, 2, 0);

   if (filter_enabled)
   {
      hpf_mode.write(use_reference);
      hpf_cutoff.write(cutoff);
   }
   hpf_internal_filter_en.write(filter_enabled);
}

void LIS331::setHPFReference(int8_t reference)
{
   BusRegister reference_reg(i2c_dev, LIS331_REG_REFERENCE);
   reference_reg.write(reference);
}

int8_t LIS331::getHPFReference(void)
{
   BusRegister reference_reg(i2c_dev, LIS331_REG_REFERENCE);
   return (int8_t)reference_reg.read();
}

void LIS331::HPFReset(void)
{
   BusRegister reference_reset_reg(i2c_dev, LIS331_REG_HP_FILTER_RESET);
   reference_reset_reg.read();
}

void LIS331::writeRange(uint8_t range)
{
   BusRegister ctrl4_reg(i2c_dev, LIS331_REG_CTRL4);
   BusRegisterBits range_bits(&ctrl4_reg, 2, 4);

   range_bits.write(range);
   sleep_ms(15); // delay to let new setting settle
}

uint8_t LIS331::readRange(void)
{
   BusRegister ctrl4_reg(i2c_dev, LIS331_REG_CTRL4);
   BusRegisterBits range_bits(&ctrl4_reg, 2, 4);

   return (uint8_t)range_bits.read();
}

void LIS331::setDataRate(lis331_data_rate_t dataRate)
{
   int8_t dr_value = 0;
   int8_t pm_value = 0;

   lis331_mode_t new_mode = getMode(dataRate);

   BusRegister ctrl1_reg(i2c_dev, LIS331_REG_CTRL1);
   BusRegisterBits pm_bits(&ctrl1_reg, 3, 5);

   switch (new_mode)
   {
   case LIS331_MODE_SHUTDOWN:
      break;

   case LIS331_MODE_LOW_POWER: // ODR bits are in CTRL1[7:5] (PM)
      pm_value = (dataRate & 0x1C) >> 2;
      break;

   case LIS331_MODE_NORMAL: // ODR bits are in CTRL1[4:3] (DR)
   {
      pm_value = (dataRate & 0x1C) >> 2;
      dr_value = (dataRate & 0x7);

      // only Normal mode uses DR to set ODR, so we can set it here
      BusRegisterBits dr_bits(&ctrl1_reg, 2, 3);
      dr_bits.write(dr_value);
      break;
   }
   }

   pm_bits.write(pm_value);
}

lis331_data_rate_t LIS331::getDataRate(void)
{
   BusRegister ctrl1_reg(i2c_dev, LIS331_REG_CTRL1);
   BusRegisterBits pm_dr_bits(&ctrl1_reg, 5, 3);

   return (lis331_data_rate_t)pm_dr_bits.read();
}

lis331_mode_t LIS331::getMode(void)
{
   return getMode(getDataRate());
}

lis331_mode_t LIS331::getMode(lis331_data_rate_t dataRate)
{
   uint8_t pm_value = (dataRate & 0x1C) >> 2;
   if (pm_value >= LIS331_MODE_LOW_POWER)
   {
      return LIS331_MODE_LOW_POWER;
   }
   return (lis331_mode_t)pm_value;
}

bool LIS331::setLPFCutoff(lis331_lpf_cutoff_t cutoff)
{
   if (getMode() == LIS331_MODE_NORMAL)
   {
      // the LPF cutoff bits are used to set the ODR while in Normal mode
      return false;
   }

   BusRegister ctrl1_reg(i2c_dev, LIS331_REG_CTRL1);
   BusRegisterBits data_rate_bits(&ctrl1_reg, 2, 3);

   data_rate_bits.write(cutoff);
   return true;
}

void LIS331::scaleValues(void)
{
}
