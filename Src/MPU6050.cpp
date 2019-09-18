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
#define ACCEL_CONFIG ((uint8_t)0b00001000)//self testとか加速度のスケールとか +-4g
#define GYRO_CONFIG  ((uint8_t)0b00001000)//self testとか角速度のスケールとか +-500deg/sec

MPU6050::MPU6050(void) {
	this->raw_ma_x = 0.0;
	this->raw_ma_y = 0.0;
	this->raw_ma_z = 0.0;
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

void MPU6050::Sample(I2C_HandleTypeDef* hi2c) {
	ReadAccGyro(hi2c);
	Calc_Ang();
	Calc_Vel();
}

void MPU6050::GetGyroBias(I2C_HandleTypeDef* hi2c, float * const avg,
		float * const stdev) {
	static I2C_HandleTypeDef* handle = hi2c;
	static constexpr int NumOfTrial = 256;

	float _avg[3] = { };
	float _stdev[3] = { };

	for (int i = 0; i < NumOfTrial; i++) {

		this->ReadAccGyro(handle);

		_avg[0] += this->raw_ma_x;
		_avg[1] += this->raw_ma_y;
		_avg[2] += this->raw_mdps_z;

		_stdev[0] += this->raw_ma_x * raw_ma_x;
		_stdev[1] += this->raw_ma_y * raw_ma_y;
		_stdev[2] += this->raw_mdps_z * raw_mdps_z;
		HAL_Delay(5);
	}

	for (int k = 0; k < 3; k++) {
		_avg[k] /= NumOfTrial;

		_stdev[k] -= NumOfTrial * _avg[k] * _avg[k];
		_stdev[k] /= NumOfTrial - 1;
		_stdev[k] = sqrtf(_stdev[k]);

		avg[k] = _avg[k];
		stdev[k] = _stdev[k];
	}
}

bool MPU6050::Init(I2C_HandleTypeDef* hi2c) {
	static I2C_HandleTypeDef* handle = hi2c;
	uint8_t reg;
	uint8_t temp;
	uint8_t d[2];

	if (HAL_I2C_IsDeviceReady(handle, this->device_address << 1, 32, 5)
			!= HAL_OK) {
		return false;
	}
	reg = MPU_WHO_AM_I
	;
	if (HAL_I2C_Master_Transmit(handle, this->device_address << 1, &reg, 1,
			1000) != HAL_OK) {
		return false;
	}

	if (HAL_I2C_Master_Receive(handle, this->device_address << 1, &temp, 1,
			1000) != HAL_OK) {
		return false;
	}

	if (temp != I_AM) { //whoami
		return false;
	}

	//power and clock
	d[0] = MPU_MGMT_1
	;
	d[1] = POW_CLK_SET;
	if (HAL_I2C_Master_Transmit(handle, this->device_address << 1, d, 2, 1000)
			!= HAL_OK) {
		return false;
	}
	//
	d[0] = MPU_CONFIG
	;
	d[1] = CONFIG;
	if (HAL_I2C_Master_Transmit(handle, this->device_address << 1, d, 2, 1000)
			!= HAL_OK) {
		return false;
	}
	d[0] = MPU_SMPRT_DIV
	;
	d[1] = SMPRT_SET;
	if (HAL_I2C_Master_Transmit(handle, this->device_address << 1, d, 2, 1000)
			!= HAL_OK) {
		return false;
	}
	d[0] = MPU_ACCCEL_CONFIG
	;
	d[1] = ACCEL_CONFIG;
	while (HAL_I2C_Master_Transmit(handle, this->device_address << 1, d, 2,
			1000) != HAL_OK) {
		return false;
	}
	d[0] = MPU_GYRO_CONFIG
	;
	d[1] = GYRO_CONFIG;
	while (HAL_I2C_Master_Transmit(handle, this->device_address << 1, d, 2,
			1000) != HAL_OK) {
		return false;
	}

	//yskさんのやつパクりました ドリフトとオフセットの補正方法はなんとかしたい
	HAL_Delay(100);

	float avg[3] = { };
	float stdev[3] = { 1000, 1000, 1000 }; //acc_x acc_y mdps_z この辺の値はいじらんとあかん気がする

	for (int i = 0; i < 10; i++) {
		this->GetGyroBias(hi2c, avg, stdev);

		if (stdev[0] < 700 && stdev[1] < 700 && stdev[2] < 700) { //もうちょいいい書き方あると思う
			this->movavg_acc_x = avg[0];
			this->movavg_acc_y = avg[1];
			this->movavg_ang_z = avg[2];
			return true;
		}
	}

	return false;
}

void MPU6050::ReadAccGyro(I2C_HandleTypeDef* hi2c) {
	static I2C_HandleTypeDef* handle = hi2c;
	uint8_t data[14];
	int16_t temp;
	uint8_t reg = MPU_ACC_X_H
	;

	/* Read full raw data, 14bytes */

	while (HAL_I2C_Master_Transmit(handle, this->device_address << 1, &reg, 1,
			1000) != HAL_OK)
		;
	while (HAL_I2C_Master_Receive(handle, this->device_address << 1, data, 14,
			1000) != HAL_OK)
		;

	/* Format accelerometer data */
	this->raw_ma_x = ((int16_t) (data[0] << 8 | data[1]) * 9.80665 * 1000)
			/ 8192.0;
	this->raw_ma_y = ((int16_t) (data[2] << 8 | data[3]) * 9.80665 * 1000)
			/ 8192.0;
	this->raw_ma_z = ((int16_t) (data[4] << 8 | data[5]) * 9.80665 * 1000)
			/ -8192.0;

	/* Format temperature */
	temp = (data[6] << 8 | data[7]);
	this->temp = (float) ((float) ((int16_t) temp) / (float) 340.0
			+ (float) 36.53);

	/* Format gyroscope data */
	this->raw_mdps_x = ((int16_t) (data[8] << 8 | data[9]) * 1000) / 65.5;
	this->raw_mdps_y = ((int16_t) (data[10] << 8 | data[11]) * 1000) / 65.5;
	this->raw_mdps_z = ((int16_t) (data[12] << 8 | data[13]) * 1000) / 65.5;

//	this->Calc();
	/* Return OK */
}

void MPU6050::Calc_Ang(void) {
	static constexpr float movband = 100.0;
	static constexpr float RadPerMilliDeg = M_PI / 180000.0;
	static constexpr float RadPerMilliDegPerSec = RadPerMilliDeg
			/ SamplingFrequency;
	static constexpr float w = 0.01f;
	static float dt = 0;
	static uint32_t last_time = HAL_GetTick();
	static float old = 0;

//	dt = (HAL_GetTick() - last_time) / 1000.0;
	last_time = HAL_GetTick();

	float dy_biased_mdps = raw_mdps_z - movavg_ang_z;

	if (dy_biased_mdps < -movband || movband < dy_biased_mdps) {
		// yaw is in radian, so, convert from mdps to radian.
//		yaw += dy_biased_mdps * RadPerMilliDeg * dt;
		yaw += this->Integral(old, dy_biased_mdps) * RadPerMilliDegPerSec;

		if (yaw > (float) M_PI) {
			yaw -= (2.0f * (float) M_PI);
		} else if (yaw < -(float) M_PI) {
			yaw += (2.0f * (float) M_PI);
		}
	} else {
		movavg_ang_z = (((movavg_ang_z * (1 - w)) + (raw_mdps_z * w)));
	}

	old = dy_biased_mdps;
}

void MPU6050::Calc_Vel(void) {
	static constexpr float movband_x = 200.0;
	static constexpr float movband_y = 200.0;
	static constexpr float AccPerMilliG = 1.0 / 1000;
	static constexpr float AccPerMilliGPerSec = AccPerMilliG
			/ SamplingFrequency;
	static constexpr float w_x = 0.01f;
	static constexpr float w_y = 0.01f;
	static float dt = 0;
	static uint32_t last_time = HAL_GetTick();
	static float old_x = 0;
	static float old_y = 0;
	static float lowpassValue_x = movavg_acc_x;
	static float lowpassValue_y = movavg_acc_y;

//	dt = (HAL_GetTick() - last_time) / 1000.0;
	last_time = HAL_GetTick();



	float dy_biased_ma_x = raw_ma_x - movavg_acc_x;
	float dy_biased_ma_y = raw_ma_y - movavg_acc_y;

	if (dy_biased_ma_x < -movband_x || movband_x < dy_biased_ma_x) {
		vel_x += this->Integral(old_x, dy_biased_ma_x) * AccPerMilliGPerSec;
	} else {
		movavg_acc_x = (((movavg_acc_x * (1 - w_x)) + (raw_ma_x * w_x)));
	}

	if (dy_biased_ma_y < -movband_y || movband_y < dy_biased_ma_y) {
		vel_y += this->Integral(old_y, dy_biased_ma_y) * AccPerMilliGPerSec;
	} else {
		movavg_acc_y = (((movavg_acc_y * (1 - w_y)) + (raw_ma_y * w_y)));
	}

	old_x = dy_biased_ma_x;
	old_y = dy_biased_ma_y;
}

float MPU6050::Integral(float old, float current) {
	return ((old + current)) / 2.0;
}
