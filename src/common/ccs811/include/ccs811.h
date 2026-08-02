/* Original Source Ported From https://github.com/adafruit/Adafruit_CCS811 */
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

#ifndef __CCS811_H__
#define __CCS811_H__

#include "hardware/i2c.h"
#include "i2c_device.h"

#define CCS811_ADDR 0x5A //!< Default I2C address

#define CCS811_REG_STATUS 0x00
#define CCS811_REG_MEAS_MODE 0x01
#define CCS811_REG_ALG_RESULT_DATA 0x02
#define CCS811_REG_ENV_DATA 0x05
#define CCS811_REG_THRESHOLDS 0x10
#define CCS811_REG_BASELINE 0x11
#define CCS811_REG_HW_ID 0x20
#define CCS811_REG_SW_RESET 0xFF

#define CCS811_BOOTLOADER_APP_START 0xF4

#define CCS811_HW_ID_CODE 0x81 //!< Expected value of CCS811_REG_HW_ID

//! Sample rate for readData()
typedef enum
{
   CCS811_DRIVE_MODE_IDLE = 0x00,
   CCS811_DRIVE_MODE_1SEC = 0x01,
   CCS811_DRIVE_MODE_10SEC = 0x02,
   CCS811_DRIVE_MODE_60SEC = 0x03,
   CCS811_DRIVE_MODE_250MS = 0x04,
} ccs811_drive_mode_t;

//! Driver for the CCS811 eCO2/TVOC air quality sensor
class CCS811
{
public:
   bool begin(i2c_inst_t *i2cInst = i2c0, uint8_t addr = CCS811_ADDR);

   void setEnvironmentalData(float humidity, float temperature);

   uint16_t getBaseline(void);
   void setBaseline(uint16_t baseline);

   void setThresholds(uint16_t low_med, uint16_t med_high, uint8_t hysteresis = 50);

   void SWReset(void);

   void setDriveMode(ccs811_drive_mode_t mode);
   void enableInterrupt(void);
   void disableInterrupt(void);

   uint16_t getTVOC(void) { return TVOC; }
   uint16_t geteCO2(void) { return eCO2; }
   uint16_t getCurrentSelected(void) { return currentSelected; } // uA
   uint16_t getRawADCreading(void) { return rawADCreading; }

   bool available(void);
   uint8_t readData(void);

   bool checkError(void);

private:
   I2CDevice *i2c_dev = NULL;

   uint16_t TVOC = 0;
   uint16_t eCO2 = 0;
   uint16_t currentSelected = 0;
   uint16_t rawADCreading = 0;

   uint8_t measMode = 0; // shadow of CCS811_REG_MEAS_MODE (DRIVE_MODE / INT_DATARDY / INT_THRESH bits)
   bool statusError = false;
   bool statusFwModeApp = false;

   void write8(uint8_t reg, uint8_t value);
   uint8_t read8(uint8_t reg);
   void read(uint8_t reg, uint8_t *buf, uint8_t num);
   void write(uint8_t reg, uint8_t *buf, uint8_t num);
};

#endif // __CCS811_H__
