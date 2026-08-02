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

#include "ads1x15.h"

static const uint16_t MUX_BY_CHANNEL[] = {
    ADS1X15_REG_CONFIG_MUX_SINGLE_0,
    ADS1X15_REG_CONFIG_MUX_SINGLE_1,
    ADS1X15_REG_CONFIG_MUX_SINGLE_2,
    ADS1X15_REG_CONFIG_MUX_SINGLE_3,
};

bool ADS1X15::begin(i2c_inst_t *i2cInst, uint8_t addr)
{
   i2c_dev = new I2CDevice(addr, i2cInst);
   return i2c_dev->begin();
}

void ADS1X15::setGain(ads1x15_gain_t g) { gain = g; }
ads1x15_gain_t ADS1X15::getGain(void) { return gain; }
void ADS1X15::setDataRate(uint16_t rate) { dataRate = rate; }
uint16_t ADS1X15::getDataRate(void) { return dataRate; }

int16_t ADS1X15::readADC_SingleEnded(uint8_t channel)
{
   if (channel > 3)
   {
      return 0;
   }

   startADCReading(MUX_BY_CHANNEL[channel], /*continuous=*/false);

   while (!conversionComplete())
      ;

   return getLastConversionResults();
}

int16_t ADS1X15::readADC_Differential_0_1(void)
{
   startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);
   while (!conversionComplete())
      ;
   return getLastConversionResults();
}

int16_t ADS1X15::readADC_Differential_0_3(void)
{
   startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_3, /*continuous=*/false);
   while (!conversionComplete())
      ;
   return getLastConversionResults();
}

int16_t ADS1X15::readADC_Differential_1_3(void)
{
   startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_1_3, /*continuous=*/false);
   while (!conversionComplete())
      ;
   return getLastConversionResults();
}

int16_t ADS1X15::readADC_Differential_2_3(void)
{
   startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, /*continuous=*/false);
   while (!conversionComplete())
      ;
   return getLastConversionResults();
}

void ADS1X15::startComparator_SingleEnded(uint8_t channel, int16_t threshold)
{
   uint16_t config =
       ADS1X15_REG_CONFIG_CQUE_1CONV |
       ADS1X15_REG_CONFIG_CLAT_LATCH |
       ADS1X15_REG_CONFIG_CPOL_ACTVLOW |
       ADS1X15_REG_CONFIG_CMODE_TRAD |
       ADS1X15_REG_CONFIG_MODE_CONTIN;

   config |= gain;
   config |= dataRate;
   config |= MUX_BY_CHANNEL[channel];

   // shift 12-bit results left for the ADS1015
   writeRegister(ADS1X15_REG_POINTER_HITHRESH, threshold << bitShift);
   writeRegister(ADS1X15_REG_POINTER_CONFIG, config);
}

void ADS1X15::startADCReading(uint16_t mux, bool continuous)
{
   uint16_t config =
       ADS1X15_REG_CONFIG_CQUE_1CONV |
       ADS1X15_REG_CONFIG_CLAT_NONLAT |
       ADS1X15_REG_CONFIG_CPOL_ACTVLOW |
       ADS1X15_REG_CONFIG_CMODE_TRAD;

   config |= continuous ? ADS1X15_REG_CONFIG_MODE_CONTIN : ADS1X15_REG_CONFIG_MODE_SINGLE;
   config |= gain;
   config |= dataRate;
   config |= mux;
   config |= ADS1X15_REG_CONFIG_OS_SINGLE;

   writeRegister(ADS1X15_REG_POINTER_CONFIG, config);

   // set ALERT/RDY to RDY mode
   writeRegister(ADS1X15_REG_POINTER_HITHRESH, 0x8000);
   writeRegister(ADS1X15_REG_POINTER_LOWTHRESH, 0x0000);
}

bool ADS1X15::conversionComplete(void)
{
   return (readRegister(ADS1X15_REG_POINTER_CONFIG) & 0x8000) != 0;
}

int16_t ADS1X15::getLastConversionResults(void)
{
   uint16_t res = readRegister(ADS1X15_REG_POINTER_CONVERT) >> bitShift;
   if (bitShift == 0)
   {
      return (int16_t)res;
   }

   // shift 12-bit results right for the ADS1015, keeping the sign bit intact
   if (res > 0x07FF)
   {
      res |= 0xF000;
   }
   return (int16_t)res;
}

float ADS1X15::getFsRange(void)
{
   switch (gain)
   {
   case ADS1X15_GAIN_TWOTHIRDS:
      return 6.144f;
   case ADS1X15_GAIN_ONE:
      return 4.096f;
   case ADS1X15_GAIN_TWO:
      return 2.048f;
   case ADS1X15_GAIN_FOUR:
      return 1.024f;
   case ADS1X15_GAIN_EIGHT:
      return 0.512f;
   case ADS1X15_GAIN_SIXTEEN:
      return 0.256f;
   default:
      return 0.0f;
   }
}

float ADS1X15::computeVolts(int16_t counts)
{
   return counts * (getFsRange() / (32768 >> bitShift));
}

void ADS1X15::writeRegister(uint8_t reg, uint16_t value)
{
   buffer[0] = reg;
   buffer[1] = value >> 8;
   buffer[2] = value & 0xFF;
   i2c_dev->write(buffer, 3);
}

uint16_t ADS1X15::readRegister(uint8_t reg)
{
   buffer[0] = reg;
   i2c_dev->write(buffer, 1);
   i2c_dev->read(buffer, 2);
   return ((uint16_t)buffer[0] << 8) | buffer[1];
}
