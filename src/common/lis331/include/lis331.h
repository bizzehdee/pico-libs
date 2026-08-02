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

#ifndef __LIS331_H__
#define __LIS331_H__

#include "hardware/i2c.h"
#include "i2c_device.h"

#define LIS331_ADDR 0x18   //!< Default I2C address (if SDO/SA0 is 3V, its 0x19)
#define LIS331_CHIP_ID 0x32 //!< Expected response from LIS331_REG_WHOAMI

#define LIS331_REG_WHOAMI 0x0F          //!< Device identification register
#define LIS331_REG_CTRL1 0x20           //!< Power mode, data rate, axis enable
#define LIS331_REG_CTRL2 0x21           //!< Memory reboot, HPF config
#define LIS331_REG_CTRL3 0x22           //!< Interrupt config, polarity, pin mode, latching, pin enable
#define LIS331_REG_CTRL4 0x23           //!< BDU, Endianness, Range, SPI mode
#define LIS331_REG_CTRL5 0x24           //!< Sleep to wake enable
#define LIS331_REG_HP_FILTER_RESET 0x25 //!< Dummy register to reset filter
#define LIS331_REG_REFERENCE 0x26       //!< HPF reference value
#define LIS331_REG_STATUS 0x27          //!< Data overrun status, Data available status
#define LIS331_REG_OUT_X_L 0x28         //!< X-axis acceleration data, low byte
#define LIS331_REG_OUT_X_H 0x29         //!< X-axis acceleration data, high byte
#define LIS331_REG_OUT_Y_L 0x2A         //!< Y-axis acceleration data, low byte
#define LIS331_REG_OUT_Y_H 0x2B         //!< Y-axis acceleration data, high byte
#define LIS331_REG_OUT_Z_L 0x2C         //!< Z-axis acceleration data, low byte
#define LIS331_REG_OUT_Z_H 0x2D         //!< Z-axis acceleration data, high byte
#define LIS331_REG_INT1CFG 0x30         //!< INT1 config. Enable on hi/low for each axis
#define LIS331_REG_INT1SRC 0x31         //!< INT1 source info
#define LIS331_REG_INT1THS 0x32         //!< INT1 acceleration threshold
#define LIS331_REG_INT1DUR 0x33         //!< INT1 duration threshold
#define LIS331_REG_INT2CFG 0x34         //!< INT2 config. Enable on hi/low for each axis
#define LIS331_REG_INT2SRC 0x35         //!< INT2 source info
#define LIS331_REG_INT2THS 0x36         //!< INT2 acceleration threshold
#define LIS331_REG_INT2DUR 0x37         //!< INT2 duration threshold

//! The high pass filter cutoff frequency
typedef enum
{
   LIS331_HPF_0_02_ODR,   //!< ODR/50
   LIS331_HPF_0_01_ODR,   //!< ODR/100
   LIS331_HPF_0_005_ODR,  //!< ODR/200
   LIS331_HPF_0_0025_ODR, //!< ODR/400
} lis331_hpf_cutoff_t;

//! The low pass filter cutoff frequency
typedef enum
{
   LIS331_LPF_37_HZ,
   LIS331_LPF_74_HZ,
   LIS331_LPF_292_HZ,
   LIS331_LPF_780_HZ,
} lis331_lpf_cutoff_t;

//! Used with LIS331_REG_CTRL1 to set power mode and bandwidth
typedef enum
{
   LIS331_DATARATE_POWERDOWN = 0,
   LIS331_DATARATE_50_HZ = 0x4,
   LIS331_DATARATE_100_HZ = 0x5,
   LIS331_DATARATE_400_HZ = 0x6,
   LIS331_DATARATE_1000_HZ = 0x7,
   LIS331_DATARATE_LOWPOWER_0_5_HZ = 0x8,
   LIS331_DATARATE_LOWPOWER_1_HZ = 0xC,
   LIS331_DATARATE_LOWPOWER_2_HZ = 0x10,
   LIS331_DATARATE_LOWPOWER_5_HZ = 0x14,
   LIS331_DATARATE_LOWPOWER_10_HZ = 0x18,
} lis331_data_rate_t;

//! Power mode options
typedef enum
{
   LIS331_MODE_SHUTDOWN,
   LIS331_MODE_NORMAL,
   LIS331_MODE_LOW_POWER // Low power is from 2-6 so checks against this should be 'mode >= LIS331_MODE_LOW_POWER'
} lis331_mode_t;

class LIS331
{
public:
   bool begin(i2c_inst_t *i2cInst = i2c0, uint8_t addr = LIS331_ADDR);
   uint8_t getDeviceID(void);

   void read(void);

   void configIntDataReady(uint8_t irqnum = 1, bool activelow = true, bool opendrain = true);

   void enableHighPassFilter(bool filter_enabled, lis331_hpf_cutoff_t cutoff = LIS331_HPF_0_0025_ODR, bool use_reference = false);
   void setHPFReference(int8_t reference);
   int8_t getHPFReference(void);
   void HPFReset(void);
   bool setLPFCutoff(lis331_lpf_cutoff_t cutoff);

   void setDataRate(lis331_data_rate_t dataRate);
   lis331_data_rate_t getDataRate(void);

   lis331_mode_t getMode(void);
   lis331_mode_t getMode(lis331_data_rate_t rate);

   int16_t x = 0; //!< raw x axis value
   int16_t y = 0; //!< raw y axis value
   int16_t z = 0; //!< raw z axis value

   float x_g = 0; //!< x axis acceleration in g, scaled by the selected range
   float y_g = 0; //!< y axis acceleration in g, scaled by the selected range
   float z_g = 0; //!< z axis acceleration in g, scaled by the selected range

protected:
   virtual void scaleValues(void);
   void writeRange(uint8_t range);
   uint8_t readRange(void);

   I2CDevice *i2c_dev = NULL;
};

#endif // __LIS331_H__
