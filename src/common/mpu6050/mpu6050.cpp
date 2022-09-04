#include "mpu6050.h"
#include "bus_register.h"
#include "bus_register_bits.h"
#include "hardware/timer.h"
#include <string.h>

MPU6050::MPU6050()
{
}

MPU6050::~MPU6050()
{
}

bool MPU6050::begin(i2c_inst_t *i2cInst, uint8_t i2c_addr, int32_t sensorID)
{
	i2c_dev = new I2CDevice(i2c_addr, i2cInst);

	if (!i2c_dev->begin())
	{
		return false;
	}

	BusRegister chipId = BusRegister(i2c_dev, MPU6050_WHO_AM_I, 1);

	// make sure we're talking to the right chip
	if (chipId.read() != MPU6050_DEVICE_ID)
	{
		return false;
	}

	return _init(sensorID);
}

void MPU6050::reset(void)
{
	BusRegister power_mgmt_1 =
		BusRegister(i2c_dev, MPU6050_PWR_MGMT_1, 1);
	BusRegister sig_path_reset =
		BusRegister(i2c_dev, MPU6050_SIGNAL_PATH_RESET, 1);
	BusRegisterBits device_reset =
		BusRegisterBits(&power_mgmt_1, 1, 7);

	device_reset.write(1); // reset
	while (device_reset.read() == 1)
	{ // check for the post reset value
		sleep_us(1);
	}
	sleep_us(100);

	sig_path_reset.write(0x7);
}

void MPU6050::setSampleRateDivisor(uint8_t divisor)
{
	BusRegister sample_rate_div =
		BusRegister(i2c_dev, MPU6050_SMPLRT_DIV, 1);
	sample_rate_div.write(divisor);
}

void MPU6050::setFilterBandwidth(mpu6050_bandwidth_t bandwidth)
{
	BusRegister config =
		BusRegister(i2c_dev, MPU6050_CONFIG, 1);

	BusRegisterBits filter_config =
		BusRegisterBits(&config, 3, 0);

	filter_config.write(bandwidth);
}

bool MPU6050::_init(int32_t sensor_id)
{
	_sensorid_accel = sensor_id;
	_sensorid_gyro = sensor_id + 1;
	_sensorid_temp = sensor_id + 2;

	reset();

	setSampleRateDivisor(0);

	setFilterBandwidth(MPU6050_BAND_260_HZ);

	setGyroRange(MPU6050_RANGE_500_DEG);

	setAccelerometerRange(MPU6050_RANGE_2_G);

	BusRegister power_mgmt_1 =
		BusRegister(i2c_dev, MPU6050_PWR_MGMT_1, 1);

	power_mgmt_1.write(0x01);

	return true;
}

mpu6050_gyro_range_t MPU6050::getGyroRange(void)
{
	BusRegister gyro_config =
		BusRegister(i2c_dev, MPU6050_GYRO_CONFIG, 1);
	BusRegisterBits gyro_range =
		BusRegisterBits(&gyro_config, 2, 3);

	return (mpu6050_gyro_range_t)gyro_range.read();
}

void MPU6050::setGyroRange(mpu6050_gyro_range_t new_range)
{
	BusRegister gyro_config =
		BusRegister(i2c_dev, MPU6050_GYRO_CONFIG, 1);
	BusRegisterBits gyro_range =
		BusRegisterBits(&gyro_config, 2, 3);

	gyro_range.write(new_range);
}

mpu6050_accel_range_t MPU6050::getAccelerometerRange(void)
{
	BusRegister accel_config =
		BusRegister(i2c_dev, MPU6050_ACCEL_CONFIG, 1);
	BusRegisterBits accel_range =
		BusRegisterBits(&accel_config, 2, 3);

	return (mpu6050_accel_range_t)accel_range.read();
}

void MPU6050::setAccelerometerRange(mpu6050_accel_range_t new_range)
{
	BusRegister accel_config =
		BusRegister(i2c_dev, MPU6050_ACCEL_CONFIG, 1);

	BusRegisterBits accel_range =
		BusRegisterBits(&accel_config, 2, 3);

	accel_range.write(new_range);
}

void MPU6050::setInterruptPinPolarity(bool active_low)
{
	BusRegister int_pin_config =
		BusRegister(i2c_dev, MPU6050_INT_PIN_CONFIG, 1);
	BusRegisterBits int_level =
		BusRegisterBits(&int_pin_config, 1, 7);
	int_level.write(active_low);
}

void MPU6050::setInterruptPinLatch(bool held)
{
	BusRegister int_pin_config =
		BusRegister(i2c_dev, MPU6050_INT_PIN_CONFIG, 1);
	BusRegisterBits int_latch =
		BusRegisterBits(&int_pin_config, 1, 5);
	int_latch.write(held);
}

void MPU6050::setFsyncSampleOutput(mpu6050_fsync_out_t fsync_output)
{
	BusRegister config =
		BusRegister(i2c_dev, MPU6050_CONFIG, 1);
	BusRegisterBits fsync_out =
		BusRegisterBits(&config, 3, 3);
	fsync_out.write(fsync_output);
}

mpu6050_highpass_t MPU6050::getHighPassFilter(void)
{
	BusRegister config =
		BusRegister(i2c_dev, MPU6050_ACCEL_CONFIG, 1);

	BusRegisterBits filter_config =
		BusRegisterBits(&config, 3, 0);
	return (mpu6050_highpass_t)filter_config.read();
}

void MPU6050::setHighPassFilter(mpu6050_highpass_t bandwidth)
{
	BusRegister config =
		BusRegister(i2c_dev, MPU6050_ACCEL_CONFIG, 1);

	BusRegisterBits filter_config =
		BusRegisterBits(&config, 3, 0);
	filter_config.write(bandwidth);
}

void MPU6050::setMotionInterrupt(bool active)
{
	BusRegister int_enable =
		BusRegister(i2c_dev, MPU6050_INT_ENABLE, 1);
	BusRegisterBits int_motion =
		BusRegisterBits(&int_enable, 1, 6);
	int_motion.write(active);
}

void MPU6050::setMotionDetectionThreshold(uint8_t thr)
{
	BusRegister threshold =
		BusRegister(i2c_dev, MPU6050_MOT_THR, 1);
	threshold.write(thr);
}

void MPU6050::setMotionDetectionDuration(uint8_t dur)
{
	BusRegister duration =
		BusRegister(i2c_dev, MPU6050_MOT_DUR, 1);
	duration.write(dur);
}

bool MPU6050::getMotionInterruptStatus(void)
{
	BusRegister status =
		BusRegister(i2c_dev, MPU6050_INT_STATUS, 1);

	BusRegisterBits motion =
		BusRegisterBits(&status, 1, 6);

	return (bool)motion.read();
}

mpu6050_fsync_out_t MPU6050::getFsyncSampleOutput(void)
{
	BusRegister config =
		BusRegister(i2c_dev, MPU6050_CONFIG, 1);
	BusRegisterBits fsync_out =
		BusRegisterBits(&config, 3, 3);
	return (mpu6050_fsync_out_t)fsync_out.read();
}

void MPU6050::setI2CBypass(bool bypass)
{
	BusRegister int_pin_config =
		BusRegister(i2c_dev, MPU6050_INT_PIN_CONFIG, 1);
	BusRegisterBits i2c_bypass =
		BusRegisterBits(&int_pin_config, 1, 1);

	BusRegister user_ctrl =
		BusRegister(i2c_dev, MPU6050_USER_CTRL, 1);
	BusRegisterBits i2c_master_enable =
		BusRegisterBits(&user_ctrl, 1, 5);

	i2c_bypass.write(bypass);
	i2c_master_enable.write(!bypass);
}

void MPU6050::setClock(mpu6050_clock_select_t new_clock)
{
	BusRegister pwr_mgmt =
		BusRegister(i2c_dev, MPU6050_PWR_MGMT_1, 1);

	BusRegisterBits clock_select =
		BusRegisterBits(&pwr_mgmt, 3, 0);
	clock_select.write(new_clock);
}

mpu6050_clock_select_t MPU6050::getClock(void)
{
	BusRegister pwr_mgmt =
		BusRegister(i2c_dev, MPU6050_PWR_MGMT_1, 1);

	BusRegisterBits clock_select =
		BusRegisterBits(&pwr_mgmt, 3, 0);
	return (mpu6050_clock_select_t)clock_select.read();
}

mpu6050_bandwidth_t MPU6050::getFilterBandwidth(void)
{
	BusRegister config =
		BusRegister(i2c_dev, MPU6050_CONFIG, 1);

	BusRegisterBits filter_config =
		BusRegisterBits(&config, 3, 0);
	return (mpu6050_bandwidth_t)filter_config.read();
}

uint8_t MPU6050::getSampleRateDivisor(void)
{
	BusRegister sample_rate_div =
		BusRegister(i2c_dev, MPU6050_SMPLRT_DIV, 1);
	return sample_rate_div.read();
}

bool MPU6050::enableSleep(bool enable)
{
	BusRegister pwr_mgmt =
		BusRegister(i2c_dev, MPU6050_PWR_MGMT_1, 1);

	BusRegisterBits sleep =
		BusRegisterBits(&pwr_mgmt, 1, 6);
	return sleep.write(enable);
}

bool MPU6050::enableCycle(bool enable)
{
	BusRegister pwr_mgmt =
		BusRegister(i2c_dev, MPU6050_PWR_MGMT_1, 1);

	BusRegisterBits cycle =
		BusRegisterBits(&pwr_mgmt, 1, 5);
	return cycle.write(enable);
}

void MPU6050::setCycleRate(mpu6050_cycle_rate_t rate)
{
	BusRegister pwr_mgmt_2 =
		BusRegister(i2c_dev, MPU6050_PWR_MGMT_2, 1);

	BusRegisterBits cycle_rate =
		BusRegisterBits(&pwr_mgmt_2, 2, 6);
	cycle_rate.write(rate);
}

mpu6050_cycle_rate_t MPU6050::getCycleRate(void)
{
	BusRegister pwr_mgmt_2 =
		BusRegister(i2c_dev, MPU6050_PWR_MGMT_2, 1);

	BusRegisterBits cycle_rate =
		BusRegisterBits(&pwr_mgmt_2, 2, 6);
	return (mpu6050_cycle_rate_t)cycle_rate.read();
}

bool MPU6050::setGyroStandby(bool xAxisStandby, bool yAxisStandby, bool zAxisStandby)
{
	BusRegister pwr_mgmt_2 =
		BusRegister(i2c_dev, MPU6050_PWR_MGMT_2, 1);

	BusRegisterBits gyro_stdby =
		BusRegisterBits(&pwr_mgmt_2, 3, 0);
	return gyro_stdby.write(xAxisStandby << 2 | yAxisStandby << 1 | zAxisStandby);
}

bool MPU6050::setAccelerometerStandby(bool xAxisStandby, bool yAxisStandby, bool zAxisStandby)
{
	BusRegister pwr_mgmt_2 =
		BusRegister(i2c_dev, MPU6050_PWR_MGMT_2, 1);

	BusRegisterBits accel_stdby =
		BusRegisterBits(&pwr_mgmt_2, 3, 3);
	return accel_stdby.write(xAxisStandby << 2 | yAxisStandby << 1 |
							 zAxisStandby);
}

bool MPU6050::setTemperatureStandby(bool enable)
{
	BusRegister pwr_mgmt =
		BusRegister(i2c_dev, MPU6050_PWR_MGMT_1, 1);

	BusRegisterBits temp_stdby =
		BusRegisterBits(&pwr_mgmt, 1, 3);
	return temp_stdby.write(1);
}

bool MPU6050::getEvent(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp)
{
	uint32_t timestamp = time_us_32();

	_read();

	fillTempEvent(temp, timestamp);
	fillAccelEvent(accel, timestamp);
	fillGyroEvent(gyro, timestamp);

	return true;
}

void MPU6050::_read(void)
{
	BusRegister data_reg =
		BusRegister(i2c_dev, MPU6050_ACCEL_OUT, 14);

	uint8_t buffer[14];
	data_reg.read(buffer, 14);

	rawAccX = buffer[0] << 8 | buffer[1];
	rawAccY = buffer[2] << 8 | buffer[3];
	rawAccZ = buffer[4] << 8 | buffer[5];

	rawTemp = buffer[6] << 8 | buffer[7];

	rawGyroX = buffer[8] << 8 | buffer[9];
	rawGyroY = buffer[10] << 8 | buffer[11];
	rawGyroZ = buffer[12] << 8 | buffer[13];

	temperature = (rawTemp / 340.0) + 36.53;

	mpu6050_accel_range_t accel_range = getAccelerometerRange();

	float accel_scale = 1;
	if (accel_range == MPU6050_RANGE_16_G)
		accel_scale = 2048;
	if (accel_range == MPU6050_RANGE_8_G)
		accel_scale = 4096;
	if (accel_range == MPU6050_RANGE_4_G)
		accel_scale = 8192;
	if (accel_range == MPU6050_RANGE_2_G)
		accel_scale = 16384;

	// setup range dependant scaling
	accX = ((float)rawAccX) / accel_scale;
	accY = ((float)rawAccY) / accel_scale;
	accZ = ((float)rawAccZ) / accel_scale;

	mpu6050_gyro_range_t gyro_range = getGyroRange();

	float gyro_scale = 1;
	if (gyro_range == MPU6050_RANGE_250_DEG)
		gyro_scale = 131;
	if (gyro_range == MPU6050_RANGE_500_DEG)
		gyro_scale = 65.5;
	if (gyro_range == MPU6050_RANGE_1000_DEG)
		gyro_scale = 32.8;
	if (gyro_range == MPU6050_RANGE_2000_DEG)
		gyro_scale = 16.4;

	gyroX = ((float)rawGyroX) / gyro_scale;
	gyroY = ((float)rawGyroY) / gyro_scale;
	gyroZ = ((float)rawGyroZ) / gyro_scale;
}

void MPU6050::fillTempEvent(sensors_event_t *temp, uint32_t timestamp)
{
	memset(temp, 0, sizeof(sensors_event_t));
	temp->version = sizeof(sensors_event_t);
	temp->sensor_id = _sensorid_temp;
	temp->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
	temp->timestamp = timestamp;
	temp->temperature = temperature;
}

void MPU6050::fillAccelEvent(sensors_event_t *accel, uint32_t timestamp)
{
	memset(accel, 0, sizeof(sensors_event_t));
	accel->version = 1;
	accel->sensor_id = _sensorid_accel;
	accel->type = SENSOR_TYPE_ACCELEROMETER;
	accel->timestamp = timestamp;
	accel->acceleration.x = accX * SENSORS_GRAVITY_STANDARD;
	accel->acceleration.y = accY * SENSORS_GRAVITY_STANDARD;
	accel->acceleration.z = accZ * SENSORS_GRAVITY_STANDARD;
}

void MPU6050::fillGyroEvent(sensors_event_t *gyro, uint32_t timestamp)
{
	memset(gyro, 0, sizeof(sensors_event_t));
	gyro->version = 1;
	gyro->sensor_id = _sensorid_gyro;
	gyro->type = SENSOR_TYPE_GYROSCOPE;
	gyro->timestamp = timestamp;
	gyro->gyro.x = gyroX * SENSORS_DPS_TO_RADS;
	gyro->gyro.y = gyroY * SENSORS_DPS_TO_RADS;
	gyro->gyro.z = gyroZ * SENSORS_DPS_TO_RADS;
}
