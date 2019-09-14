#include "MPU6050.hpp"

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
#define CONFIG ((uint8_t)0b00000000)//デジタルローパスフィルタとか gyroscope output rate = 8kHz
#define SMPRT_SET ((uint8_t)0b00000111)//Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
#define ACCEL_CONFIG ((uint8_t)0b00000000)//self testとか加速度のスケールとか
#define GYRO_CONFIG  ((uint8_t)0b00001000)//self testとか角速度のスケールとか +-500deg/sec

MPU6050::MPU6050(void){
	this->acc_x = 0;
	this->acc_y = 0;
	this->acc_z = 0;
	this->gyro_x= 0;
	this->gyro_y= 0;
	this->gyro_z= 0;
	this->temp  = 0;
}

bool MPU6050::MPU6050_Init(I2C_HandleTypeDef* hi2c) {

	uint8_t reg;
	uint8_t temp;
	uint8_t d[2];

	if (HAL_I2C_IsDeviceReady(hi2c, this->device_address << 1, 32, 5) != HAL_OK) {
		return false;
	}
	reg=MPU_WHO_AM_I;
	if (HAL_I2C_Master_Transmit(hi2c, this->device_address << 1, &reg, 1,
			1000) != HAL_OK) {
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
	d[0] = MPU_MGMT_1;
	d[1] = POW_CLK_SET;
	if (HAL_I2C_Master_Transmit(hi2c, this->device_address << 1, d, 2, 1000)
			!= HAL_OK) {
		return false;
	}
	//
	d[0] = MPU_CONFIG;
	d[1] = CONFIG;
	if (HAL_I2C_Master_Transmit(hi2c, this->device_address << 1, d, 2, 1000)
			!= HAL_OK) {
		return false;
	}
	d[0] = MPU_SMPRT_DIV;
	d[1] = SMPRT_SET;
	if (HAL_I2C_Master_Transmit(hi2c, this->device_address << 1, d, 2, 1000)
			!= HAL_OK) {
		return false;
	}
	d[0] = MPU_ACCCEL_CONFIG;
	d[1] = ACCEL_CONFIG;
	while (HAL_I2C_Master_Transmit(hi2c, this->device_address << 1, d, 1, 1000)
			!= HAL_OK) {
		return false;
	}
	d[0] = MPU_ACCCEL_CONFIG;
	d[1] = ACCEL_CONFIG;
	while (HAL_I2C_Master_Transmit(hi2c, this->device_address << 1, d, 1, 1000)
			!= HAL_OK) {
		return false;
	}
	return true;
}

void MPU6050::MPU6050_ReadAccGyro(I2C_HandleTypeDef* hi2c) {
	uint8_t data[14];
	int16_t temp;
	uint8_t reg = MPU_ACC_X_H;

	/* Read full raw data, 14bytes */
	while (HAL_I2C_Master_Transmit(hi2c, this->device_address<<1, &reg, 1, 1000)
			!= HAL_OK)
		;

	while (HAL_I2C_Master_Receive(hi2c, this->device_address<<1, data, 14, 1000)
			!= HAL_OK)
		;

	/* Format accelerometer data */
	this->acc_x = (int16_t) (data[0] << 8 | data[1]);
	this->acc_y = (int16_t) (data[2] << 8 | data[3]);
    this->acc_z = (int16_t) (data[4] << 8 | data[5]);

	/* Format temperature */
	temp = (data[6] << 8 | data[7]);
	this->temp = (float) ((float) ((int16_t) temp) / (float) 340.0
			+ (float) 36.53);

	/* Format gyroscope data */
	this->gyro_x = (int16_t) (data[8]  << 8 | data[9]);
	this->gyro_y = (int16_t) (data[10] << 8 | data[11]);
	this->gyro_z = (int16_t) (data[12] << 8 | data[13]);

	/* Return OK */
}
