/*
 * ps2.c
 *
 *  Created on: Mar 31, 2025
 *      Author: SergALLy
 */
#include "ps2.h"

static uint8_t cmd_read[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t RxData[9] = {0};

static float map(float min_1, float max_1, float value, float min_2, float max_2)
{
	/*
	 *  Назначение: перерасчёт значения из 1 интервала в значение из 2 интервала
	 *  Входные параматеры:
	 *  	min_1: Нижняя граница исходного интервала
	 *  	max_1: Верхняя граница исходного интервала
	 *  	value: Значение в исходном интервале
	 *  	min_2: Нижняя граница нового интервала
	 *  	max_2^ Верхняя граница нового интервала
	 *  Return:
	 *  	Значение в новом интервале
	 */
	if (value <= min_1) return min_2;
    if (value == 0) return (max_2 - min_2)/2+min_2;
	if (value >= max_1) return max_2;
	return (value-min_1)/(max_1-min_1)*(max_2-min_2) + min_2;
}

static bool PS2_Cmd(ps2_handle_t *handle, uint8_t* TxData, uint8_t size)
{
	bool success = true;

	CS_H; CS_L;
	success &= (HAL_SPI_Transmit(handle->spi_handle, TxData, size, 100) == HAL_OK);
	CS_H;
	return success;
}

bool PS2_Init(ps2_handle_t *handle)
{ // Не робит
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
	/*
	 * Чтения данных с джойстика PS2
	 * Входные данные:
	 * 			handle - Дескриптор джойстика PS2
	 * return:
	 * 			True - успешно, False - иначе
	 */
	bool success = true;
	CS_H; CS_L; // Чтение данных с джойстика
	success &= (HAL_SPI_TransmitReceive(handle -> spi_handle, cmd_read, RxData, 9, 100) == HAL_OK);
	CS_H;
	handle -> ID = RxData[1];
	// Проверка корректности данных
	success &= ((handle -> ID == PS2_GREEN_MODE) || (handle -> ID == PS2_RED_MODE)) && (RxData[2] == PS2_READY);
	handle -> buttons = ~(RxData[3] | RxData[4]<<8);
	if (handle -> ID == PS2_RED_MODE) {
		handle -> right_stick.X = map(0, 255, RxData[5], -127, 127);
		handle -> right_stick.Y = map(0, 255, RxData[6], 127, -127);
		handle -> left_stick.X = map(0, 255, RxData[7], -127, 127);
		handle -> left_stick.Y = -map(0, 255, RxData[8], -127, 127);
	}
	return success;
}

void PS2_Vibration(ps2_handle_t *handle,uint8_t motor1, uint8_t motor2)
{
	// Не рабочая
	uint8_t buff[9] = {0x01, 0x42, 0x00, motor1, motor2, 0x00, 0x00, 0x00, 0x00};
	PS2_Cmd(handle, buff, 9);
}
