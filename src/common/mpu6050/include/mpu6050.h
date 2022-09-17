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

#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "hardware/i2c.h"
#include "i2c_device.h"
#include "sensor_defs.h"

#define MPU6050_I2CADDR_DEFAULT \
	0x68					   ///< MPU6050 default i2c address w/ AD0 high
#define MPU6050_DEVICE_ID 0x68 ///< The correct MPU6050_WHO_AM_I value

#define MPU6050_SELF_TEST_X \
	0x0D ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Y \
	0x0E ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Z \
	0x0F ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_A \
	0x10						 ///< Self test factory calibrated values register
#define MPU6050_SMPLRT_DIV 0x19	 ///< sample rate divisor register
#define MPU6050_CONFIG 0x1A		 ///< General configuration register
#define MPU6050_GYRO_CONFIG 0x1B ///< Gyro specfic configuration register
#define MPU6050_ACCEL_CONFIG \
	0x1C							   ///< Accelerometer specific configration register
#define MPU6050_INT_PIN_CONFIG 0x37	   ///< Interrupt pin configuration register
#define MPU6050_INT_ENABLE 0x38		   ///< Interrupt enable configuration register
#define MPU6050_INT_STATUS 0x3A		   ///< Interrupt status register
#define MPU6050_WHO_AM_I 0x75		   ///< Divice ID register
#define MPU6050_SIGNAL_PATH_RESET 0x68 ///< Signal path reset register
#define MPU6050_USER_CTRL 0x6A		   ///< FIFO and I2C Master control register
#define MPU6050_PWR_MGMT_1 0x6B		   ///< Primary power/sleep control register
#define MPU6050_PWR_MGMT_2 0x6C		   ///< Secondary power/sleep control register
#define MPU6050_TEMP_H 0x41			   ///< Temperature data high byte register
#define MPU6050_TEMP_L 0x42			   ///< Temperature data low byte register
#define MPU6050_ACCEL_OUT 0x3B		   ///< base address for sensor data reads
#define MPU6050_MOT_THR 0x1F		   ///< Motion detection threshold bits [7:0]
#define MPU6050_MOT_DUR \
	0x20 ///< Duration counter threshold for motion int. 1 kHz rate, LSB = 1 ms

/**
 * @brief FSYNC output values
 *
 * Allowed values for `setFsyncSampleOutput`.
 */
typedef enum fsync_out
{
	MPU6050_FSYNC_OUT_DISABLED,
	MPU6050_FSYNC_OUT_TEMP,
	MPU6050_FSYNC_OUT_GYROX,
	MPU6050_FSYNC_OUT_GYROY,
	MPU6050_FSYNC_OUT_GYROZ,
	MPU6050_FSYNC_OUT_ACCELX,
	MPU6050_FSYNC_OUT_ACCELY,
	MPU6050_FSYNC_OUT_ACCEL_Z,
} mpu6050_fsync_out_t;

/**
 * @brief Clock source options
 *
 * Allowed values for `setClock`.
 */
typedef enum clock_select
{
	MPU6050_INTR_8MHz,
	MPU6050_PLL_GYROX,
	MPU6050_PLL_GYROY,
	MPU6050_PLL_GYROZ,
	MPU6050_PLL_EXT_32K,
	MPU6050_PLL_EXT_19MHz,
	MPU6050_STOP = 7,
} mpu6050_clock_select_t;

/**
 * @brief Accelerometer range options
 *
 * Allowed values for `setAccelerometerRange`.
 */
typedef enum
{
	MPU6050_RANGE_2_G = 0b00,  ///< +/- 2g (default value)
	MPU6050_RANGE_4_G = 0b01,  ///< +/- 4g
	MPU6050_RANGE_8_G = 0b10,  ///< +/- 8g
	MPU6050_RANGE_16_G = 0b11, ///< +/- 16g
} mpu6050_accel_range_t;

/**
 * @brief Gyroscope range options
 *
 * Allowed values for `setGyroRange`.
 */
typedef enum
{
	MPU6050_RANGE_250_DEG,	///< +/- 250 deg/s (default value)
	MPU6050_RANGE_500_DEG,	///< +/- 500 deg/s
	MPU6050_RANGE_1000_DEG, ///< +/- 1000 deg/s
	MPU6050_RANGE_2000_DEG, ///< +/- 2000 deg/s
} mpu6050_gyro_range_t;

/**
 * @brief Digital low pass filter bandthwidth options
 *
 * Allowed values for `setFilterBandwidth`.
 */
typedef enum
{
	MPU6050_BAND_260_HZ, ///< Docs imply this disables the filter
	MPU6050_BAND_184_HZ, ///< 184 Hz
	MPU6050_BAND_94_HZ,	 ///< 94 Hz
	MPU6050_BAND_44_HZ,	 ///< 44 Hz
	MPU6050_BAND_21_HZ,	 ///< 21 Hz
	MPU6050_BAND_10_HZ,	 ///< 10 Hz
	MPU6050_BAND_5_HZ,	 ///< 5 Hz
} mpu6050_bandwidth_t;

/**
 * @brief Accelerometer high pass filter options
 *
 * Allowed values for `setHighPassFilter`.
 */
typedef enum
{
	MPU6050_HIGHPASS_DISABLE,
	MPU6050_HIGHPASS_5_HZ,
	MPU6050_HIGHPASS_2_5_HZ,
	MPU6050_HIGHPASS_1_25_HZ,
	MPU6050_HIGHPASS_0_63_HZ,
	MPU6050_HIGHPASS_UNUSED,
	MPU6050_HIGHPASS_HOLD,
} mpu6050_highpass_t;

/**
 * @brief Periodic measurement options
 *
 * Allowed values for `setCycleRate`.
 */
typedef enum
{
	MPU6050_CYCLE_1_25_HZ, ///< 1.25 Hz
	MPU6050_CYCLE_5_HZ,	   ///< 5 Hz
	MPU6050_CYCLE_20_HZ,   ///< 20 Hz
	MPU6050_CYCLE_40_HZ,   ///< 40 Hz
} mpu6050_cycle_rate_t;

class MPU6050
{
public:
	// default to i2c0, pins 4/5
	MPU6050();
	~MPU6050();

	bool begin(i2c_inst_t *i2cInst = i2c0, uint8_t i2c_addr = MPU6050_I2CADDR_DEFAULT, int32_t sensorID = 0);
	bool getEvent(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp);

	void reset(void);
	void setSampleRateDivisor(uint8_t divisor);
	void setFilterBandwidth(mpu6050_bandwidth_t bandwidth);

	mpu6050_gyro_range_t getGyroRange(void);
	void setGyroRange(mpu6050_gyro_range_t);
	mpu6050_accel_range_t getAccelerometerRange(void);
	void setAccelerometerRange(mpu6050_accel_range_t);

	void setInterruptPinPolarity(bool active_low);
	void setInterruptPinLatch(bool held);
	void setFsyncSampleOutput(mpu6050_fsync_out_t fsync_output);

	mpu6050_highpass_t getHighPassFilter(void);
	void setHighPassFilter(mpu6050_highpass_t bandwidth);

	void setMotionInterrupt(bool active);
	void setMotionDetectionThreshold(uint8_t thr);
	void setMotionDetectionDuration(uint8_t dur);
	bool getMotionInterruptStatus(void);

	mpu6050_fsync_out_t getFsyncSampleOutput(void);
	void setI2CBypass(bool bypass);

	void setClock(mpu6050_clock_select_t);
	mpu6050_clock_select_t getClock(void);

	mpu6050_bandwidth_t getFilterBandwidth(void);
	uint8_t getSampleRateDivisor(void);

	bool enableSleep(bool enable);
	bool enableCycle(bool enable);

	void setCycleRate(mpu6050_cycle_rate_t rate);
	mpu6050_cycle_rate_t getCycleRate(void);

	bool setGyroStandby(bool xAxisStandby, bool yAxisStandby, bool zAxisStandby);
	bool setAccelerometerStandby(bool xAxisStandby, bool yAxisStandby, bool zAxisStandby);
	bool setTemperatureStandby(bool enable);

private:
	virtual bool _init(int32_t sensor_id);
	void _read(void);
private:
	I2CDevice *i2c_dev = NULL;
	uint16_t _sensorid_accel, ///< ID number for accelerometer
		_sensorid_gyro,		  ///< ID number for gyro
		_sensorid_temp;		  ///< ID number for temperature

	int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
	float temperature, ///< Last reading's temperature (C)
		accX,		   ///< Last reading's accelerometer X axis m/s^2
		accY,		   ///< Last reading's accelerometer Y axis m/s^2
		accZ,		   ///< Last reading's accelerometer Z axis m/s^2
		gyroX,		   ///< Last reading's gyro X axis in rad/s
		gyroY,		   ///< Last reading's gyro Y axis in rad/s
		gyroZ;		   ///< Last reading's gyro Z axis in rad/s

	void fillTempEvent(sensors_event_t *temp, uint32_t timestamp);
	void fillAccelEvent(sensors_event_t *accel, uint32_t timestamp);
	void fillGyroEvent(sensors_event_t *gyro, uint32_t timestamp);
};

#endif // __MPU6050_H__
