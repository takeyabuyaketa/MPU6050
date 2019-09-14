#ifndef INCLUDE_MPU9250_HPP_
#define INCLUDE_MPU9250_HPP_

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_i2c.h"
#include <stdbool.h>

class MPU6050
{
private:
	uint8_t device_address=0x68;
	float Gyro_Mult;
	float Acce_Mult;

public:
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	float   temp;
	MPU6050(void);
	bool MPU6050_Init(I2C_HandleTypeDef* hi2c);
	void MPU6050_ReadAccGyro(I2C_HandleTypeDef* hi2c);

};

#endif /* INCLUDE_MPU9250_HPP_ */
