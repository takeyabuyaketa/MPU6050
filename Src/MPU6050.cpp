#include "MPU6050.hpp"
#include <math.h>

#define MPU_ADDRESS  0x68;
uint8_t I_AM = 0x68;
// mpu6050 register address
#define MPU_WHO_AM_I ((uint8_t)0x75);

#define MPU_ACC_X_H ((uint8_t) 0x3B);
#define MPU_ACC_X_L ((uint8_t) 0x3C);
#define MPU_ACC_Y_H ((uint8_t) 0x3D);
#define MPU_ACC_Y_L ((uint8_t) 0x3E);
#define MPU_ACC_Z_H ((uint8_t) 0x3F);
#define MPU_ACC_Z_L ((uint8_t) 0x40);
#define MPU_TEMP_H ((uint8_t) 0x41);
#define MPU_TEMP_L ((uint8_t) 0x42);
#define MPU_GYRO_X_H ((uint8_t) 0x43);
#define MPU_GYRO_X_L ((uint8_t) 0x44);
#define MPU_GYRO_Y_H ((uint8_t) 0x45);
#define MPU_GYRO_Y_L ((uint8_t) 0x46);
#define MPU_GYRO_Z_H ((uint8_t) 0x47);
#define MPU_GYRO_Z_L ((uint8_t) 0x48);

#define MPU_MGMT_1 ((uint8_t) 0x6B);
#define MPU_CONFIG ((uint8_t) 0x1A);
#define MPU_SMPRT_DIV ((uint8_t) 0x19);
#define MPU_ACCCEL_CONFIG ((uint8_t) 0x1c);
#define MPU_GYRO_CONFIG ((uint8_t) 0x1B);
//data
#define POW_CLK_SET ((uint8_t)0b00000011)//電源とかクロック周り
#define CONFIG ((uint8_t)0b00000011)//デジタルローパスフィルタとか gyroscope output rate = 1kHz
#define SMPRT_SET ((uint8_t)0b00000100)//Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
#define ACCEL_CONFIG ((uint8_t)0b00000000)//self testとか加速度のスケールとか
#define GYRO_CONFIG  ((uint8_t)0b00001000)//self testとか角速度のスケールとか +-500deg/sec

MPU6050::MPU6050(void) {
	this->raw_acc_x = 0.0;
	this->raw_acc_y = 0.0;
	this->raw_acc_z = 0.0;
	this->raw_mdps_x = 0.0;
	this->raw_mdps_y = 0.0;
	this->raw_mdps_z = 0.0;
	this->acc_x = 0.0;
	this->acc_y = 0.0;
	this->acc_z = 0.0;
	this->vel_x = 0.0;
	this->vel_y = 0.0;
	this->vel_z = 0.0;
	this->angv_x = 0.0;
	this->angv_y = 0.0;
	this->angv_z = 0.0;
	this->roll = 0.0;
	this->pitch = 0.0;
	this->yaw = 0.0;
	this->temp = 0.0;
}

//void MPU6050::GetGyroBias(float * const avg, float * const stdev) const {
//	static constexpr int NumOfTrial = 256;
//
//	float _avg = 0.0f;
//	float _stdev = 0.0f;
//
//	for (int i = 0; i < NumOfTrial; i++) {
//		float reading = HAL_I2C_Master_Transmit(hi2c, this->device_address << 1, &reg, 1,
//				1000) ;
//
//		_avg += reading;
//		_stdev += reading * reading;
//
//		Timer::sleep(5);
//	}
//
//	_avg /= NumOfTrial;
//
//	_stdev -= NumOfTrial * _avg * _avg;
//	_stdev /= NumOfTrial - 1;
//	_stdev = sqrtf(_stdev);
//
//	*avg = _avg;
//	*stdev = _stdev;
//}

bool MPU6050::Init(I2C_HandleTypeDef* hi2c) {

	uint8_t reg;
	uint8_t temp;
	uint8_t d[2];

	if (HAL_I2C_IsDeviceReady(hi2c, this->device_address << 1, 32, 5)
			!= HAL_OK) {
		return false;
	}
	reg = MPU_WHO_AM_I
	;
	if (HAL_I2C_Master_Transmit(hi2c, this->device_address << 1, &reg, 1, 1000)
			!= HAL_OK) {
		return false;
	}

	if (HAL_I2C_Master_Receive(hi2c, this->device_address << 1, &temp, 1, 1000)
			!= HAL_OK) {
		return false;
	}

	if (temp != I_AM) { //whoami
		return false;
	}

	//power and clock
	d[0] = MPU_MGMT_1
	;
	d[1] = POW_CLK_SET;
	if (HAL_I2C_Master_Transmit(hi2c, this->device_address << 1, d, 2, 1000)
			!= HAL_OK) {
		return false;
	}
	//
	d[0] = MPU_CONFIG
	;
	d[1] = CONFIG;
	if (HAL_I2C_Master_Transmit(hi2c, this->device_address << 1, d, 2, 1000)
			!= HAL_OK) {
		return false;
	}
	d[0] = MPU_SMPRT_DIV
	;
	d[1] = SMPRT_SET;
	if (HAL_I2C_Master_Transmit(hi2c, this->device_address << 1, d, 2, 1000)
			!= HAL_OK) {
		return false;
	}
	d[0] = MPU_ACCCEL_CONFIG
	;
	d[1] = ACCEL_CONFIG;
	while (HAL_I2C_Master_Transmit(hi2c, this->device_address << 1, d, 1, 1000)
			!= HAL_OK) {
		return false;
	}
	d[0] = MPU_ACCCEL_CONFIG
	;
	d[1] = ACCEL_CONFIG;
	while (HAL_I2C_Master_Transmit(hi2c, this->device_address << 1, d, 1, 1000)
			!= HAL_OK) {
		return false;
	}

	//yskさんのやつパクりました ドリフトとオフセットの補正方法はなんとかしたい
//	HAL_Delay(100);
//
//	float avg = 0.0f;
//	float stdev = 1000.0f;
//
//	for (int i = 0; i < 10; i++) {
//		this->GetGyroBias(&avg, &stdev);
//
//		if (stdev < 700) {
//			movavg = (int32_t) avg;
//
//			return true;
//		}
//	}

	return true;
}

void MPU6050::ReadAccGyro(I2C_HandleTypeDef* hi2c) {
	uint8_t data[14];
	int16_t temp;
	uint8_t reg = MPU_ACC_X_H
	;

	/* Read full raw data, 14bytes */
//	while (HAL_I2C_Master_Transmit(hi2c, this->device_address << 1, &reg, 1,
//			1000) != HAL_OK)
//		;

	while (HAL_I2C_Master_Receive(hi2c, this->device_address << 1, data, 14,
			1000) != HAL_OK)
		;

	/* Format accelerometer data */
	this->raw_acc_x = (int16_t) (data[0] << 8 | data[1] / 16384);
	this->raw_acc_y = (int16_t) (data[2] << 8 | data[3] / 16384);
	this->raw_acc_z = (int16_t) (data[4] << 8 | data[5] / 16384);

	/* Format temperature */
	temp = (data[6] << 8 | data[7]);
	this->temp = (float) ((float) ((int16_t) temp) / (float) 340.0
			+ (float) 36.53);

	/* Format gyroscope data */
	this->raw_mdps_x = ((int16_t) (data[8] << 8 | data[9])*1000.0) / 65.5;
	this->raw_mdps_y = ((int16_t) (data[10] << 8 | data[11])*1000.0) / 65.5;
	this->raw_mdps_z = ((int16_t) (data[12] << 8 | data[13])*1000.0) / 65.5;

	this->Calc();
	/* Return OK */
}

void MPU6050::Calc(void) {
	static constexpr float RadPerMilliDeg = M_PI / 180000.0;
	static constexpr float RadPerMilliDegPerSec = RadPerMilliDeg / SamplingFrequency;
//	static constexpr float halfPi = M_PI / 2.0;

	yaw += (float) raw_mdps_z * RadPerMilliDegPerSec;

	if (yaw > (float) M_PI) {
		yaw -= (2.0f * (float) M_PI);
	} else if (yaw < -(float) M_PI) {
		yaw += (2.0f * (float) M_PI);
	}

}
