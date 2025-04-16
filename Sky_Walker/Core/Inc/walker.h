/*
 * walker.h
 *
 *  Created on: Apr 14, 2025
 *      Author: SergALLy
 */

#ifndef INC_WALKER_H_
#define INC_WALKER_H_

#include <stdint.h>
#include "stdbool.h"
#include "pca9685.h"
#include "ps2.h"
//#include "stm32l4xx_hal.h"
//#include "main.h"

typedef struct {
	pca9685_handle_t *pca_handle;
	uint8_t thigh; // Бедро, канал сервопривода
	uint8_t knee; // Колено, канал сервопривода
	uint8_t foot; // Стопа, канал сервопривода
} leg_handle_t;

typedef struct {
	leg_handle_t *leg_1;
	leg_handle_t *leg_2;
	leg_handle_t *leg_3;
	leg_handle_t *leg_4;
	leg_handle_t *leg_5;
	leg_handle_t *leg_6;
} walker_handle_t;

#define MAX_ANGLE 	30
#define MIN_ANGLE	-30

/*
 * Назначение: Инициализация драйверов pca9685
 * Входные параметры:
 * 		pca_1, pca_2: дескрипторы драйверов pca9685
 * Return:
 * 		True - инициализация успешно
 * 		False - иначе
 */
bool walker_init(pca9685_handle_t* pca_1, pca9685_handle_t* pca_2);

/*
 * Назначение: Вывод всех сервоприводов в нейтральное положение
 * Входные параметры:
 * 		handle: дескриптор робота
 * Return:
 * 		True - успешно
 * 		False - иначе
 */
bool walker_servo_neultral(walker_handle_t *handle);

bool walker_run(walker_handle_t *walker, stick *stick);

#endif /* INC_WALKER_H_ */
