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

typedef struct {
	pca9685_handle_t *pca_handle;
	uint8_t coxa; 	// Серво, ближайший к корпусу
	uint8_t femur; 	// Среднее серво
	uint8_t tibia; 	// Серво, ближайший к концу конечности
	float X;
	float Y;
	float Z;
} leg_handle_t;

typedef leg_handle_t walker_handle_t[6];

#include "walker_config.h"

/*
 * Назначение: Инициализация драйверов pca9685
 * Входные параметры:
 * 		pca_1, pca_2: дескрипторы драйверов pca9685
 * Return:
 * 		True - инициализация успешно
 * 		False - иначе
 */
bool walker_init(pca9685_handle_t *pca_1, pca9685_handle_t *pca_2);

/*
 * Назначение: Вывод всех сервоприводов в нейтральное положение
 * Входные параметры:
 * 		handle: дескриптор робота
 * Return:
 * 		True - успешно
 * 		False - иначе
 */
bool walker_servo_nelrtal(walker_handle_t *walker);

bool walker_calc_ik(walker_handle_t *walker, uint8_t leg_number, float x,
		float y, float z);

void walker_tripod_mode(ps2_handle_t *ps, walker_handle_t *walker);

#endif /* INC_WALKER_H_ */
