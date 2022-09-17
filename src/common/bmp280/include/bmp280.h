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

#ifndef __BMP280_H__
#define __BMP280_H__

#include "hardware/i2c.h"

/*!
 *  I2C ADDRESS/BITS/SETTINGS
 */
#define BMP280_ADDRESS (0x76)     /**< The default I2C address for the sensor. */
#define BMP280_ADDRESS_ALT (0x77) /**< Alternative I2C address for the sensor. */
#define BMP280_CHIPID (0x58)      /**< Default chip ID. */

enum
{
    BMP280_REGISTER_DIG_T1 = 0x88,
    BMP280_REGISTER_DIG_T2 = 0x8A,
    BMP280_REGISTER_DIG_T3 = 0x8C,
    BMP280_REGISTER_DIG_P1 = 0x8E,
    BMP280_REGISTER_DIG_P2 = 0x90,
    BMP280_REGISTER_DIG_P3 = 0x92,
    BMP280_REGISTER_DIG_P4 = 0x94,
    BMP280_REGISTER_DIG_P5 = 0x96,
    BMP280_REGISTER_DIG_P6 = 0x98,
    BMP280_REGISTER_DIG_P7 = 0x9A,
    BMP280_REGISTER_DIG_P8 = 0x9C,
    BMP280_REGISTER_DIG_P9 = 0x9E,
    BMP280_REGISTER_CHIPID = 0xD0,
    BMP280_REGISTER_VERSION = 0xD1,
    BMP280_REGISTER_SOFTRESET = 0xE0,
    BMP280_REGISTER_CAL26 = 0xE1, /**< R calibration = 0xE1-0xF0 */
    BMP280_REGISTER_STATUS = 0xF3,
    BMP280_REGISTER_CONTROL = 0xF4,
    BMP280_REGISTER_CONFIG = 0xF5,
    BMP280_REGISTER_PRESSUREDATA = 0xF7,
    BMP280_REGISTER_TEMPDATA = 0xFA,
};

typedef struct
{
    uint16_t dig_T1; /**< dig_T1 cal register. */
    int16_t dig_T2;  /**<  dig_T2 cal register. */
    int16_t dig_T3;  /**< dig_T3 cal register. */

    uint16_t dig_P1; /**< dig_P1 cal register. */
    int16_t dig_P2;  /**< dig_P2 cal register. */
    int16_t dig_P3;  /**< dig_P3 cal register. */
    int16_t dig_P4;  /**< dig_P4 cal register. */
    int16_t dig_P5;  /**< dig_P5 cal register. */
    int16_t dig_P6;  /**< dig_P6 cal register. */
    int16_t dig_P7;  /**< dig_P7 cal register. */
    int16_t dig_P8;  /**< dig_P8 cal register. */
    int16_t dig_P9;  /**< dig_P9 cal register. */
} bmp280_calib_data;

class BMP280
{
public:
    /** Oversampling rate for the sensor. */
    enum sensor_sampling
    {
        /** No over-sampling. */
        SAMPLING_NONE = 0x00,
        /** 1x over-sampling. */
        SAMPLING_X1 = 0x01,
        /** 2x over-sampling. */
        SAMPLING_X2 = 0x02,
        /** 4x over-sampling. */
        SAMPLING_X4 = 0x03,
        /** 8x over-sampling. */
        SAMPLING_X8 = 0x04,
        /** 16x over-sampling. */
        SAMPLING_X16 = 0x05
    };

    /** Operating mode for the sensor. */
    enum sensor_mode
    {
        /** Sleep mode. */
        MODE_SLEEP = 0x00,
        /** Forced mode. */
        MODE_FORCED = 0x01,
        /** Normal mode. */
        MODE_NORMAL = 0x03,
        /** Software reset. */
        MODE_SOFT_RESET_CODE = 0xB6
    };

    /** Filtering level for sensor data. */
    enum sensor_filter
    {
        /** No filtering. */
        FILTER_OFF = 0x00,
        /** 2x filtering. */
        FILTER_X2 = 0x01,
        /** 4x filtering. */
        FILTER_X4 = 0x02,
        /** 8x filtering. */
        FILTER_X8 = 0x03,
        /** 16x filtering. */
        FILTER_X16 = 0x04
    };

    /** Standby duration in ms */
    enum standby_duration
    {
        /** 1 ms standby. */
        STANDBY_MS_1 = 0x00,
        /** 62.5 ms standby. */
        STANDBY_MS_63 = 0x01,
        /** 125 ms standby. */
        STANDBY_MS_125 = 0x02,
        /** 250 ms standby. */
        STANDBY_MS_250 = 0x03,
        /** 500 ms standby. */
        STANDBY_MS_500 = 0x04,
        /** 1000 ms standby. */
        STANDBY_MS_1000 = 0x05,
        /** 2000 ms standby. */
        STANDBY_MS_2000 = 0x06,
        /** 4000 ms standby. */
        STANDBY_MS_4000 = 0x07
    };

    // default to i2c0, pins 4/5
    BMP280(i2c_inst_t *i2cInst = i2c0);
    ~BMP280();

    bool begin(uint8_t addr = BMP280_ADDRESS, uint8_t chipid = BMP280_CHIPID);
    void reset(void);
    uint8_t getStatus(void);
    uint8_t sensorID(void);

    float readTemperature();
    float readPressure(void);
    float readAltitude(float seaLevelhPa = 1013.25);
    float seaLevelForAltitude(float altitude, float atmospheric);
    float waterBoilingPoint(float pressure);
    bool takeForcedMeasurement();

    void setSampling(sensor_mode mode = MODE_NORMAL,
                     sensor_sampling tempSampling = SAMPLING_X16,
                     sensor_sampling pressSampling = SAMPLING_X16,
                     sensor_filter filter = FILTER_OFF,
                     standby_duration duration = STANDBY_MS_1);

private:
    struct config
    {
        /** Initialize to power-on-reset state */
        config() : t_sb(STANDBY_MS_1), filter(FILTER_OFF), none(0), spi3w_en(0) {}
        /** Inactive duration (standby time) in normal mode */
        unsigned int t_sb : 3;
        /** Filter settings */
        unsigned int filter : 3;
        /** Unused - don't set */
        unsigned int none : 1;
        /** Enables 3-wire SPI */
        unsigned int spi3w_en : 1;
        /** Used to retrieve the assembled config register's byte value. */
        unsigned int get() { return (t_sb << 5) | (filter << 2) | spi3w_en; }
    };

    /** Encapsulates trhe ctrl_meas register */
    struct ctrl_meas
    {
        /** Initialize to power-on-reset state */
        ctrl_meas()
            : osrs_t(SAMPLING_NONE), osrs_p(SAMPLING_NONE), mode(MODE_SLEEP) {}
        /** Temperature oversampling. */
        unsigned int osrs_t : 3;
        /** Pressure oversampling. */
        unsigned int osrs_p : 3;
        /** Device mode */
        unsigned int mode : 2;
        /** Used to retrieve the assembled ctrl_meas register's byte value. */
        unsigned int get() { return (osrs_t << 5) | (osrs_p << 2) | mode; }
    };

private:
    void readCoefficients();
    uint8_t read8(unsigned char reg);
    uint16_t read16(unsigned char reg);
    uint16_t read16_LE(unsigned char reg);
    int16_t readS16_LE(unsigned char reg);
    void write8(unsigned char reg, unsigned char value);
    uint32_t read24(unsigned char reg);
private:
    i2c_inst_t *i2cInst = NULL;
    uint8_t addr = BMP280_ADDRESS;
    int32_t _sensorID = 0;
    bmp280_calib_data _bmp280_calib;
    config _configReg;
    ctrl_meas _measReg;
    int32_t t_fine;
};

#endif // __BMP280_H__
