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
#include "main.h"

typedef struct {
	uint8_t channel;
	float angle;
} servo_handle_t;

typedef struct {
	pca9685_handle_t *pca_handle;
	servo_handle_t *thigh; // Бедро
	servo_handle_t *knee; // Колено
	servo_handle_t *foot; // Стопа
} leg_handle_t;

typedef struct {
	leg_handle_t *leg1;
	leg_handle_t *leg2;
	leg_handle_t *leg3;
	leg_handle_t *leg4;
	leg_handle_t *leg5;
	leg_handle_t *leg6;
} walker_handle_t;

#define MAX_ANGLE 	270
#define MIN_ANGLE	0

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



#endif /* INC_WALKER_H_ */
