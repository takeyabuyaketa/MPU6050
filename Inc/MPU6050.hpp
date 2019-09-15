#ifndef INCLUDE_MPU9250_HPP_
#define INCLUDE_MPU9250_HPP_

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_i2c.h"
#include <stdbool.h>

class MPU6050 {
private:
	uint8_t device_address = 0x68;
	float raw_acc_x;
	float raw_acc_y;
	float raw_acc_z;
	int raw_mdps_x;
	int raw_mdps_y;
	int raw_mdps_z;
//	void  ReadAccGyro(I2C_HandleTypeDef* hi2c);
	void GetGyroBias(I2C_HandleTypeDef* hi2c, float * const avg, float * const stdev) const;
	static constexpr uint16_t SamplingFrequency = 200;
	int movavg;

	void  Calc(void);
public:
	float acc_x;
	float acc_y;
	float acc_z;
	float vel_x;
	float vel_y;
	float vel_z;
	float angv_x;
	float angv_y;
	float angv_z;
	float roll;
	float pitch;
	float yaw;
	float temp;
	MPU6050(void);
	bool Init(I2C_HandleTypeDef* hi2c);
	void  ReadAccGyro(I2C_HandleTypeDef* hi2c);

};

#endif /* INCLUDE_MPU9250_HPP_ */
