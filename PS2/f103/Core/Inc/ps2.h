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

typedef struct {
	SPI_HandleTypeDef *spi_handle;
	uint8_t data[9];
} ps2_handle_t;

#define CS_H	HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET)
#define CS_L	HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET)

bool PS2_ReadData(ps2_handle_t *handle);

#endif /* INC_PS2_H_ */
