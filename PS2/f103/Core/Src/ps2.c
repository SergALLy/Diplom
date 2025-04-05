/*
 * ps2.c
 *
 *  Created on: Mar 31, 2025
 *      Author: SergALLy
 */
#include "ps2.h"

static 	uint8_t cmd_read[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

bool PS2_Cmd(ps2_handle_t *handle, uint8_t* TxData, uint8_t size)
{
	bool success = true;

	CS_H; CS_L;
	//HAL_SPI_Transmit(&hspi1, TxData, size, 100);
	success &= (HAL_SPI_Transmit(handle->spi_handle, TxData, size, 100) == HAL_OK);
	CS_H;
	return success;
}

bool PS2_Init(ps2_handle_t *handle)
{ // Возможно не нужна
	bool success = true;
	/*uint8_t ShortPoll[5] = {0x01, 0x42, 0x00, 0x00, 0x00};
	uint8_t EnterConfig[9] = {0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t Setup[9] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
	uint8_t VibrationMode[5] = {0x01, 0x4D, 0x00, 0x00, 0x01};
	uint8_t ExitConfig[9] = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};

	PS2_Cmd(ShortPoll, 5);
	PS2_Cmd(ShortPoll, 5);
	PS2_Cmd(ShortPoll, 5);
	PS2_Cmd(EnterConfig, 9);
	PS2_Cmd(Setup, 9);
	PS2_Cmd(VibrationMode, 5);
	PS2_Cmd(ExitConfig, 9);*/

	uint8_t ShortPoll[5] = {0x01, 0x42, 0x00, 0xff, 0xff};
	uint8_t EnterConfig[5] = {0x01, 0x43, 0x00, 0x01, 0x00};
	uint8_t Setup[9] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
	uint8_t VibrationMode[9] = {0x01, 0x4D, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff};
	uint8_t Push[9] = {0x01, 0x4F, 0x00, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00};
	uint8_t ExitConfig[9] = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};

	success &= PS2_Cmd(handle, ShortPoll, 5);
	success &= PS2_Cmd(handle, EnterConfig, 5);
	success &= PS2_Cmd(handle, Setup, 9);
	success &= PS2_Cmd(handle, VibrationMode, 9);
	success &= PS2_Cmd(handle, Push, 9);
	success &= PS2_Cmd(handle, ExitConfig, 9);
	return success;
}

bool PS2_ReadData(ps2_handle_t *handle)
{
	bool success = true;
	CS_H; CS_L;
	success &= (HAL_SPI_TransmitReceive(handle -> spi_handle, cmd_read, handle -> data, 9, 100) == HAL_OK);
	CS_H;
	success &= ((handle -> data[1] == 0x41) || (handle -> data[1] == 0x73)) && (handle -> data[2] == 0x5A);
	return success;
}

void PS2_Vibration(ps2_handle_t *handle,uint8_t motor1, uint8_t motor2)
{
	// Не рабочая
	uint8_t buff[9] = {0x01, 0x42, 0x00, motor1, motor2, 0x00, 0x00, 0x00, 0x00};
	PS2_Cmd(handle, buff, 9);
}
