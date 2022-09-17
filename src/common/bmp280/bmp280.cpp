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

#include <math.h>
#include "bmp280.h"

BMP280::BMP280(i2c_inst_t *i2cInst)
{
    this->i2cInst = i2cInst;
}

BMP280::~BMP280()
{
    this->i2cInst = NULL;
}

bool BMP280::begin(uint8_t addr, uint8_t chipid)
{
    this->addr = addr;
    _sensorID = read8(BMP280_REGISTER_CHIPID);
    if (_sensorID != chipid)
    {
        return false;
    }

    readCoefficients();
    setSampling();

    return true;
}

void BMP280::reset(void)
{
    write8(BMP280_REGISTER_SOFTRESET, MODE_SOFT_RESET_CODE);
}

uint8_t BMP280::getStatus(void)
{
    return read8(BMP280_REGISTER_STATUS);
}

uint8_t BMP280::sensorID(void)
{
    return _sensorID;
}

float BMP280::readTemperature()
{
    int32_t var1, var2;
    if (!_sensorID)
        return NAN; // begin() not called yet

    int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA);
    adc_T >>= 4;

    var1 = ((((adc_T >> 3) - ((int32_t)_bmp280_calib.dig_T1 << 1))) *
            ((int32_t)_bmp280_calib.dig_T2)) >>
           11;

    var2 = (((((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1))) >>
             12) *
            ((int32_t)_bmp280_calib.dig_T3)) >>
           14;

    t_fine = var1 + var2;

    float T = (t_fine * 5 + 128) >> 8;
    return T / 100;
}

float BMP280::readPressure(void)
{
    int64_t var1, var2, p;
    if (!_sensorID)
        return NAN; // begin() not called yet

    // Must be done first to get the t_fine variable set up
    readTemperature();

    int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
    adc_P >>= 4;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)_bmp280_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)_bmp280_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3) >> 8) +
           ((var1 * (int64_t)_bmp280_calib.dig_P2) << 12);
    var1 =
        (((((int64_t)1) << 47) + var1)) * ((int64_t)_bmp280_calib.dig_P1) >> 33;

    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)_bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7) << 4);
    return (float)p / 256;
}

float BMP280::readAltitude(float seaLevelhPa)
{
    float altitude;

    float pressure = readPressure(); // in Si units for Pascal
    pressure /= 100;

    altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

    return altitude;
}

float BMP280::seaLevelForAltitude(float altitude, float atmospheric)
{
    return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

float BMP280::waterBoilingPoint(float pressure)
{
    return (234.175 * log(pressure / 6.1078)) /
           (17.08085 - log(pressure / 6.1078));
}

bool BMP280::takeForcedMeasurement()
{
    return 0;
}

void BMP280::setSampling(sensor_mode mode,
                         sensor_sampling tempSampling,
                         sensor_sampling pressSampling,
                         sensor_filter filter,
                         standby_duration duration)
{
    if (!_sensorID)
        return; // begin() not called yet

    _measReg.mode = mode;
    _measReg.osrs_t = tempSampling;
    _measReg.osrs_p = pressSampling;

    _configReg.filter = filter;
    _configReg.t_sb = duration;

    write8(BMP280_REGISTER_CONFIG, _configReg.get());
    write8(BMP280_REGISTER_CONTROL, _measReg.get());
}

void BMP280::readCoefficients()
{
    _bmp280_calib.dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
    _bmp280_calib.dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
    _bmp280_calib.dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);

    _bmp280_calib.dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
    _bmp280_calib.dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
    _bmp280_calib.dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
    _bmp280_calib.dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
    _bmp280_calib.dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
    _bmp280_calib.dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
    _bmp280_calib.dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
    _bmp280_calib.dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
    _bmp280_calib.dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);
}

uint8_t BMP280::read8(unsigned char reg)
{
    uint8_t buffer[1];
    if (i2cInst)
    {
        buffer[0] = uint8_t(reg);

        i2c_write_blocking(i2cInst, addr, buffer, 1, false);
        i2c_read_blocking(i2cInst, addr, buffer, 1, false);
    }
    return buffer[0];
}

uint16_t BMP280::read16(unsigned char reg)
{
    uint8_t buffer[2];

    if (i2cInst)
    {
        buffer[0] = uint8_t(reg);

        i2c_write_blocking(i2cInst, addr, buffer, 1, false);
        i2c_read_blocking(i2cInst, addr, buffer, 2, false);
    }
    return uint16_t(buffer[0]) << 8 | uint16_t(buffer[1]);
}

uint16_t BMP280::read16_LE(unsigned char reg)
{
    uint16_t temp = read16(reg);
    return (temp >> 8) | (temp << 8);
}

int16_t BMP280::readS16_LE(unsigned char reg)
{
    return (int16_t)read16_LE(reg);
}

uint32_t BMP280::read24(unsigned char reg)
{
    uint8_t buffer[3];

    if (i2cInst)
    {
        buffer[0] = uint8_t(reg);
        i2c_write_blocking(i2cInst, addr, buffer, 1, false);
        i2c_read_blocking(i2cInst, addr, buffer, 3, false);
    }
    return uint32_t(buffer[0]) << 16 | uint32_t(buffer[1]) << 8 |
           uint32_t(buffer[2]);
}

void BMP280::write8(unsigned char reg, unsigned char value)
{
    uint8_t buffer[2];
    buffer[1] = value;
    if (i2cInst)
    {
        buffer[0] = reg;

        i2c_write_blocking(i2cInst, addr, buffer, 2, false);
    }
}