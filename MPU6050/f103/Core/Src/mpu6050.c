/*
 * mpu6050.c
 *
 *  Created on: Apr 13, 2025
 *      Author: SergALLy
 */
#include "mpu6050.h"

typedef enum {
	MPU_SMPRT_DIV =	0x19,		// Делитель частоты дискретизации
	MPU_CONFIG = 0x1a,			// Конфигурация
	MPU_GYRO_CONFIG	= 0x1b,		// Конфигурация гироскопа
	MPU_ACCEL_CONFIG = 0x1c,	// Конфигурация акселерометра
	MPU_FIFO_EN	= 0x23,			// Настройка выходных данных
	MPU_INT_PIN_CFG = 0x37,		// Конфигурация включения пин INT
	MPU_INT_ENABLE = 0x38,		// Разрешения прерываний
	MPU_INT_STATUS = 0x3a,		// Состояние прерывания
	MPU_ACCEL_XOUT_H = 0x3b,	// Измерения акселерометра
	MPU_TEMP_OUT_H = 0x41,		// Измерение температуры
	MPU_GYRO_XOUT_H = 0x43,		// Гироскопические измерения
	MPU_USER_CTRL = 0x6a,		// Контроль пользователя
	MPU_PWR_MGMT_1 = 0x6c,
	MPU_PWR_MGMT_2 = 0x72,
	MPU_FIFO_R_W = 0x74,
	MPU_WHO_AM_I = 0x75
} mpu6050_register_t;

static bool mpu6050_read(mpu6050_handle_t *handle, uint8_t address, uint8_t *buff, uint8_t size)
{
	return HAL_I2C_Mem_Read(handle->i2c_handle, handle->device_address, address, I2C_MEMADD_SIZE_8BIT, buff, size, 100) == HAL_OK;
}

static bool mpu6050_write_u8(mpu6050_handle_t *handle, uint8_t address, uint8_t *data)
{
	return HAL_I2C_Mem_Write(handle->i2c_handle, handle->device_address, address, I2C_MEMADD_SIZE_8BIT, data, 1, 100) == HAL_OK;
};

bool mpu6050_init(mpu6050_handle_t *handle, uint8_t setup)
{
	bool success = true;
	uint8_t data =0;

	success &= mpu6050_read(handle, MPU_WHO_AM_I, &data, 1);
	if (data == 0x68)
	{
		data = 0x00; // Внутренний генератор
		success &= mpu6050_write_u8(handle, MPU_PWR_MGMT_1, &data);

		data = 0x07; // Частота дискретизации 1кГц
		success &= mpu6050_write_u8(handle, MPU_SMPRT_DIV, &data);

		data = 0x00; // Без самопроверок, Диапазон +-250
		success &= mpu6050_write_u8(handle, MPU_GYRO_CONFIG, &data);

		data = 0x00; // Без самопроверок, диапазон +-2g
		success &= mpu6050_write_u8(handle, MPU_ACCEL_CONFIG, &data);

		return success;
	}
	else
		return false;
}

bool mpu6050_read_gyro(mpu6050_handle_t *handle)
{
	uint8_t data[6]={0};
	int16_t buff = 0;
	bool success = true;

	success &= mpu6050_read(handle, MPU_GYRO_XOUT_H, data, 6);

	buff = (int16_t)(data[0]<<8 | data[1]);
	handle -> gyro.X = buff / MPU_GYRO_LSB;

	buff = (int16_t)(data[2]<<8 | data[3]);
	handle -> gyro.Y = buff / MPU_GYRO_LSB;

	buff = (int16_t)(data[4]<<8 | data[5]);
	handle -> gyro.Z = buff / MPU_GYRO_LSB;

	return success;
}

bool mpu6050_read_accel(mpu6050_handle_t *handle)
{
	uint8_t data[6]={0};
	int16_t buff = 0;
	bool success = true;

	success &= mpu6050_read(handle, MPU_ACCEL_XOUT_H, data, 6);

	buff = (int16_t)(data[0]<<8 | data[1]);
	handle -> accel.X = buff / MPU_ACCEL_LSB;

	buff = (int16_t)(data[2]<<8 | data[3]);
	handle -> accel.Y = buff / MPU_ACCEL_LSB;

	buff = (int16_t)(data[4]<<8 | data[5]);
	handle -> accel.Z = buff / MPU_ACCEL_LSB;

	return success;
}

bool mpu6050_read_temp(mpu6050_handle_t *handle)
{
	uint8_t data[2]={0};
	int16_t buff = 0;
	bool success = true;

	success &= mpu6050_read(handle, MPU_TEMP_OUT_H, data, 2);

	buff = (int16_t)(data[0]<<8 | data[1]);
	handle -> temp = buff/340.0f + 36.53f;

	return success;
}

bool mpu6050_read_all(mpu6050_handle_t *handle)
{
	uint8_t data[14]={0};
		int16_t buff = 0;
		bool success = true;

		success &= mpu6050_read(handle, MPU_ACCEL_XOUT_H, data, 14);

		buff = (int16_t)(data[0]<<8 | data[1]);
		handle -> accel.X = buff / MPU_ACCEL_LSB;

		buff = (int16_t)(data[2]<<8 | data[3]);
		handle -> accel.Y = buff / MPU_ACCEL_LSB;

		buff = (int16_t)(data[4]<<8 | data[5]);
		handle -> accel.Z = buff / MPU_ACCEL_LSB;

		buff = (int16_t)(data[6]<<8 | data[7]);
		handle -> temp = buff/340.0f + 36.53f;

		buff = (int16_t)(data[8]<<8 | data[9]);
		handle -> gyro.X = buff / MPU_GYRO_LSB;

		buff = (int16_t)(data[10]<<8 | data[11]);
		handle -> gyro.Y = buff / MPU_GYRO_LSB;

		buff = (int16_t)(data[12]<<8 | data[13]);
		handle -> gyro.Z = buff / MPU_GYRO_LSB;

		return success;
}

