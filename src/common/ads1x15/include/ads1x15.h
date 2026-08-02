/* Original Source Ported From https://github.com/adafruit/Adafruit_ADS1X15 */
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

#ifndef __ADS1X15_H__
#define __ADS1X15_H__

#include "hardware/i2c.h"
#include "i2c_device.h"

#define ADS1X15_ADDR 0x48 //!< 1001 000 (ADDR = GND)

#define ADS1X15_REG_POINTER_CONVERT 0x00   //!< Conversion
#define ADS1X15_REG_POINTER_CONFIG 0x01    //!< Configuration
#define ADS1X15_REG_POINTER_LOWTHRESH 0x02 //!< Low threshold
#define ADS1X15_REG_POINTER_HITHRESH 0x03  //!< High threshold

#define ADS1X15_REG_CONFIG_OS_SINGLE 0x8000 //!< Write: Set to start a single-conversion

#define ADS1X15_REG_CONFIG_MUX_DIFF_0_1 0x0000 //!< Differential P = AIN0, N = AIN1 (default)
#define ADS1X15_REG_CONFIG_MUX_DIFF_0_3 0x1000 //!< Differential P = AIN0, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_DIFF_1_3 0x2000 //!< Differential P = AIN1, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_DIFF_2_3 0x3000 //!< Differential P = AIN2, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_SINGLE_0 0x4000 //!< Single-ended AIN0
#define ADS1X15_REG_CONFIG_MUX_SINGLE_1 0x5000 //!< Single-ended AIN1
#define ADS1X15_REG_CONFIG_MUX_SINGLE_2 0x6000 //!< Single-ended AIN2
#define ADS1X15_REG_CONFIG_MUX_SINGLE_3 0x7000 //!< Single-ended AIN3

#define ADS1X15_REG_CONFIG_MODE_CONTIN 0x0000 //!< Continuous conversion mode
#define ADS1X15_REG_CONFIG_MODE_SINGLE 0x0100 //!< Power-down single-shot mode (default)

#define ADS1X15_REG_CONFIG_CMODE_TRAD 0x0000  //!< Traditional comparator with hysteresis (default)
#define ADS1X15_REG_CONFIG_CPOL_ACTVLOW 0x0000 //!< ALERT/RDY pin is low when active (default)
#define ADS1X15_REG_CONFIG_CLAT_NONLAT 0x0000 //!< Non-latching comparator (default)
#define ADS1X15_REG_CONFIG_CLAT_LATCH 0x0004  //!< Latching comparator
#define ADS1X15_REG_CONFIG_CQUE_1CONV 0x0000  //!< Assert ALERT/RDY after one conversion

//! ADC gain / input voltage range
typedef enum
{
   ADS1X15_GAIN_TWOTHIRDS = 0x0000, //!< +/-6.144V range = Gain 2/3
   ADS1X15_GAIN_ONE = 0x0200,       //!< +/-4.096V range = Gain 1
   ADS1X15_GAIN_TWO = 0x0400,       //!< +/-2.048V range = Gain 2 (default)
   ADS1X15_GAIN_FOUR = 0x0600,      //!< +/-1.024V range = Gain 4
   ADS1X15_GAIN_EIGHT = 0x0800,     //!< +/-0.512V range = Gain 8
   ADS1X15_GAIN_SIXTEEN = 0x0A00,   //!< +/-0.256V range = Gain 16
} ads1x15_gain_t;

// ADS1015 data rates
#define RATE_ADS1015_128SPS 0x0000  //!< 128 samples per second
#define RATE_ADS1015_250SPS 0x0020  //!< 250 samples per second
#define RATE_ADS1015_490SPS 0x0040  //!< 490 samples per second
#define RATE_ADS1015_920SPS 0x0060  //!< 920 samples per second
#define RATE_ADS1015_1600SPS 0x0080 //!< 1600 samples per second (default)
#define RATE_ADS1015_2400SPS 0x00A0 //!< 2400 samples per second
#define RATE_ADS1015_3300SPS 0x00C0 //!< 3300 samples per second

// ADS1115 data rates
#define RATE_ADS1115_8SPS 0x0000   //!< 8 samples per second
#define RATE_ADS1115_16SPS 0x0020  //!< 16 samples per second
#define RATE_ADS1115_32SPS 0x0040  //!< 32 samples per second
#define RATE_ADS1115_64SPS 0x0060  //!< 64 samples per second
#define RATE_ADS1115_128SPS 0x0080 //!< 128 samples per second (default)
#define RATE_ADS1115_250SPS 0x00A0 //!< 250 samples per second
#define RATE_ADS1115_475SPS 0x00C0 //!< 475 samples per second
#define RATE_ADS1115_860SPS 0x00E0 //!< 860 samples per second

//! Shared driver for the ADS1015/ADS1115 ADC breakouts. DO NOT USE DIRECTLY
class ADS1X15
{
public:
   bool begin(i2c_inst_t *i2cInst = i2c0, uint8_t addr = ADS1X15_ADDR);

   int16_t readADC_SingleEnded(uint8_t channel);
   int16_t readADC_Differential_0_1(void);
   int16_t readADC_Differential_0_3(void);
   int16_t readADC_Differential_1_3(void);
   int16_t readADC_Differential_2_3(void);

   void startComparator_SingleEnded(uint8_t channel, int16_t threshold);
   void startADCReading(uint16_t mux, bool continuous);
   bool conversionComplete(void);
   int16_t getLastConversionResults(void);

   float getFsRange(void);
   float computeVolts(int16_t counts);

   void setGain(ads1x15_gain_t gain);
   ads1x15_gain_t getGain(void);
   void setDataRate(uint16_t rate);
   uint16_t getDataRate(void);

protected:
   I2CDevice *i2c_dev = NULL;
   uint8_t bitShift = 0;
   ads1x15_gain_t gain = ADS1X15_GAIN_TWOTHIRDS;
   uint16_t dataRate = 0;

private:
   void writeRegister(uint8_t reg, uint16_t value);
   uint16_t readRegister(uint8_t reg);
   uint8_t buffer[3];
};

#endif // __ADS1X15_H__
