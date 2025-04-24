/*
 * ps2.h
 *
 *  Created on: Mar 31, 2025
 *      Author: SergALLy
 */
#ifndef PS2_H_
#define PS2_H_

#include "main.h"
#include <stdbool.h>

typedef struct{
	/*
	 * Значение стика по оси X:
	 * -128 - крайнее левое
	 * 0 - среднее положение
	 * 127 - крайнее правое
	 */
	int8_t X;
	/*
	 * Значение стика по оси Y:
	 * -128 - крайнее нижнее
	 * 0 - среднее положение
	 * 127 - крайнее верхнее
	 */
	int8_t Y;
} stick;
/*
 * Структура, определяющая дескриптор, описывающий джойстик PS2.
 */
typedef struct {
	/*
	 * Дескриптор шины SPI для PS2 джойстика
	 */
	SPI_HandleTypeDef *spi_handle;
	/*
	 * Режим работы PS2 джойстика
	 */
	uint8_t ID;
	/*
	 * Нажатые кнопки:
	 * 1 - кнопка нажата;
	 * 0 - кнопка не нажата
	 */
	uint16_t buttons;
	/*
	 * Структура для правого стика
	 */
	stick right_stick;
	/*
	 * Структура для левого стика
	 */
	stick left_stick;
} ps2_handle_t;

#define CS_H			HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET)
#define CS_L			HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET)

#define PS2_READY		0x5A
#define PS2_GREEN_MODE	0x41
#define PS2_RED_MODE	0x73

#define BUTTON_SELECT   (1 << 0)
#define BUTTON_L3       (1 << 1)
#define BUTTON_R3       (1 << 2)
#define BUTTON_START    (1 << 3)
#define BUTTON_UP       (1 << 4)
#define BUTTON_RIGHT    (1 << 5)
#define BUTTON_DOWN     (1 << 6)
#define BUTTON_LEFT     (1 << 7)
#define BUTTON_L2       (1 << 8)
#define BUTTON_R2       (1 << 9)
#define BUTTON_L1       (1 << 10)
#define BUTTON_R1       (1 << 11)
#define BUTTON_TRIANGLE (1 << 12)
#define BUTTON_CIRCLE   (1 << 13)
#define BUTTON_CROSS    (1 << 14)
#define BUTTON_SQUARE   (1 << 15)

#define PS2_READ_BUTTON(BYTE,MASK)	((BYTE) & (uint16_t)(MASK))

/*
 * Чтения данных с джойстика PS2
 * Входные данные:
 * 			handle - Дескриптор джойстика PS2
 * return:
 * 			True - успешно, False - иначе
 */
bool PS2_ReadData(ps2_handle_t *handle);

#endif /* INC_PS2_H_ */
