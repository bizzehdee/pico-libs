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

#include "ccs811.h"
#include <pico/time.h>

bool CCS811::begin(i2c_inst_t *i2cInst, uint8_t addr)
{
   i2c_dev = new I2CDevice(addr, i2cInst);
   if (!i2c_dev->begin())
   {
      return false;
   }

   SWReset();
   sleep_ms(100);

   // check that the HW id is correct
   if (read8(CCS811_REG_HW_ID) != CCS811_HW_ID_CODE)
   {
      return false;
   }

   // try to start the app
   write(CCS811_BOOTLOADER_APP_START, NULL, 0);
   sleep_ms(100);

   // make sure there are no errors and we have entered application mode
   if (checkError())
   {
      return false;
   }

   uint8_t status = read8(CCS811_REG_STATUS);
   statusFwModeApp = (status >> 7) & 0x01;
   if (!statusFwModeApp)
   {
      return false;
   }

   disableInterrupt();

   // default to read every second
   setDriveMode(CCS811_DRIVE_MODE_1SEC);

   return true;
}

void CCS811::setDriveMode(ccs811_drive_mode_t mode)
{
   measMode = (measMode & ~0x70) | (mode << 4);
   write8(CCS811_REG_MEAS_MODE, measMode);
}

void CCS811::enableInterrupt(void)
{
   measMode |= (1 << 3);
   write8(CCS811_REG_MEAS_MODE, measMode);
}

void CCS811::disableInterrupt(void)
{
   measMode &= ~(1 << 3);
   write8(CCS811_REG_MEAS_MODE, measMode);
}

bool CCS811::available(void)
{
   uint8_t status = read8(CCS811_REG_STATUS);
   statusError = status & 0x01;
   return (status >> 3) & 0x01; // DATA_READY
}

uint8_t CCS811::readData(void)
{
   if (!available())
   {
      return 0;
   }

   uint8_t buf[8];
   read(CCS811_REG_ALG_RESULT_DATA, buf, 8);

   eCO2 = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
   TVOC = ((uint16_t)buf[2] << 8) | (uint16_t)buf[3];
   currentSelected = ((uint16_t)buf[6] >> 2);
   rawADCreading = ((uint16_t)(buf[6] & 3) << 8) | (uint16_t)buf[7];

   if (statusError)
   {
      return buf[5]; // error id
   }

   return 0;
}

void CCS811::setEnvironmentalData(float humidity, float temperature)
{
   /* Humidity is stored as an unsigned 16 bits in 1/512%RH. Temperature is
      stored as an unsigned 16 bits integer in 1/512 degrees, offset so that
      0 maps to -25C. */
   uint16_t hum_conv = humidity * 512.0f + 0.5f;
   uint16_t temp_conv = (temperature + 25.0f) * 512.0f + 0.5f;

   uint8_t buf[] = {
       (uint8_t)((hum_conv >> 8) & 0xFF), (uint8_t)(hum_conv & 0xFF),
       (uint8_t)((temp_conv >> 8) & 0xFF), (uint8_t)(temp_conv & 0xFF)};

   write(CCS811_REG_ENV_DATA, buf, 4);
}

uint16_t CCS811::getBaseline(void)
{
   uint8_t buf[2];
   read(CCS811_REG_BASELINE, buf, 2);
   return ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
}

void CCS811::setBaseline(uint16_t baseline)
{
   uint8_t buf[] = {(uint8_t)((baseline >> 8) & 0xFF), (uint8_t)(baseline & 0xFF)};
   write(CCS811_REG_BASELINE, buf, 2);
}

void CCS811::setThresholds(uint16_t low_med, uint16_t med_high, uint8_t hysteresis)
{
   uint8_t buf[] = {(uint8_t)((low_med >> 8) & 0xF), (uint8_t)(low_med & 0xF),
                    (uint8_t)((med_high >> 8) & 0xF), (uint8_t)(med_high & 0xF),
                    hysteresis};

   write(CCS811_REG_THRESHOLDS, buf, 5);
}

void CCS811::SWReset(void)
{
   // reset sequence from the datasheet
   uint8_t seq[] = {0x11, 0xE5, 0x72, 0x8A};
   write(CCS811_REG_SW_RESET, seq, 4);
}

bool CCS811::checkError(void)
{
   statusError = read8(CCS811_REG_STATUS) & 0x01;
   return statusError;
}

void CCS811::write8(uint8_t reg, uint8_t value)
{
   write(reg, &value, 1);
}

uint8_t CCS811::read8(uint8_t reg)
{
   uint8_t value = 0;
   read(reg, &value, 1);
   return value;
}

void CCS811::read(uint8_t reg, uint8_t *buf, uint8_t num)
{
   uint8_t buffer[1] = {reg};
   i2c_dev->write_then_read(buffer, 1, buf, num);
}

void CCS811::write(uint8_t reg, uint8_t *buf, uint8_t num)
{
   uint8_t prefix[1] = {reg};
   i2c_dev->write(buf, num, true, prefix, 1);
}
