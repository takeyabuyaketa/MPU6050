#ifndef INCLUDE_MPU9250_HPP_
#define INCLUDE_MPU9250_HPP_

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_i2c.h"
#include <stdbool.h>

class MPU6050 {
private:
	uint8_t device_address = 0x68;
//	float raw_ma_x;
//	float raw_ma_y;
//	float raw_ma_z;
//	float raw_mdps_x;
//	float raw_mdps_y;
//	float raw_mdps_z;
	void GetGyroBias(I2C_HandleTypeDef* hi2c, float * const avg, float * const stdev);//constではReadの使い回しができないのでやめた
	static constexpr uint16_t SamplingFrequency = 200;
//	float movavg;
	void  ReadAccGyro(I2C_HandleTypeDef* hi2c);
	void  Calc_Ang(void);
	void  Calc_Vel(void);
	float Integral(float old,float current);
public:
	float raw_ma_x;
	float raw_ma_y;
	float raw_ma_z;
	float movavg_acc_x;
	float movavg_acc_y;
	float movavg_ang_z;
	float raw_mdps_x;
	float raw_mdps_y;
	float raw_mdps_z;
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
	void Sample(I2C_HandleTypeDef* hi2c);
	float dt=0;

};

#endif /* INCLUDE_MPU9250_HPP_ */
