/*
 * mpu6050.h
 *
 *  Created on: Apr 13, 2025
 *      Author: SergALLy
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stdbool.h"
#include <stdint.h>
#include "main.h"

typedef struct{
	double X;
	double Y;
	double Z;
} gyro_handle_t;

typedef struct{
	double X;
	double Y;
	double Z;
} accel_handle_t;

typedef struct{
	I2C_HandleTypeDef *i2c_handle;
	uint8_t device_address;
	gyro_handle_t gyro;
	accel_handle_t accel;
	float temp;
}mpu6050_handle_t;

#define MPU6050_I2C_DEFAULT_DEVICE_ADDRESS 0xD0
/*
 *  FS_SEL		   LSB
 *    0			  131.0f
 *    1			   65.5f
 *    2			   32.8f
 *    3			   16.4f
 */
#define MPU_GYRO_LSB 	131.0f
/*
 * AFS_SEL		   LSB
 * 	  0			16384.0f
 * 	  1			 8192.0f
 * 	  2			 4096.0f
 * 	  3			 2048.0f
 */
#define MPU_ACCEL_LSB	16384.0f

#define INIT_ALL	0x7F
#define ACCEL_X		0x01
#define ACCEL_Y		0x02
#define ACCEL_Z		0x04
#define ACCEL		0x07
#define TEMP		0x08
#define GYRO_X		0x10
#define GYRO_Y		0x20
#define GYRO_Z		0x40
#define GYRO		0x70

bool mpu6050_init(mpu6050_handle_t *handle, uint8_t setup);
bool mpu6050_read_gyro(mpu6050_handle_t *handle);
bool mpu6050_read_accel(mpu6050_handle_t *handle);
bool mpu6050_read_temp(mpu6050_handle_t *handle);
bool mpu6050_read_all(mpu6050_handle_t *handle);

#endif /* INC_MPU6050_H_ */
