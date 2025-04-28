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
	float X;		// Положение ноги по оси X
	float Y;		// Положение ноги по оси Y
	float Z;		// Положение ноги по оси Z
} leg_handle_t;

typedef leg_handle_t walker_handle_t[6]; // Массив структур с ногами

#include "walker_config.h"		// Файл с конфигурацией робота

/*
 * Назначение: Инициализация драйверов pca9685, и конфигируции ног робота
 * Входные параметры:
 * 		pca_1, pca_2: дескрипторы драйверов pca9685
 * Return:
 * 		True - инициализация успешно
 * 		False - иначе
 */
bool walker_init(pca9685_handle_t *pca_1, pca9685_handle_t *pca_2);

/*
 * Назначение: Вывод всех сервоприводов в нейтральное положение
 * Return:
 * 		True - успешно
 * 		False - иначе
 */
bool walker_servo_nelrtal();

/*
 *  Назначение: Расчёт задачи обратной кинематики (ОК) для 1 ноги и перенос ноги в положение с координатами (x,y,z)
 *  Входные параметры:
 *  	 leg_number: номер ноги робота, для которой необходимо решить задаи ОК
 *  	 x, y, z: координаты положения ноги, куда её надо перенести
 *  return:
 * 		True - успешно
 * 		False - иначе
 */
bool walker_calc_ik(uint8_t leg_number, float x,
		float y, float z);

/*
 *  Назначение: Реализация режима шага - треугольник ( 3 - идут, 3 - стоят)
 *  Входные параметры:
 *  	ps - дескриптор джойстика
 */
void walker_tripod_mode(ps2_handle_t *ps);

#endif /* INC_WALKER_H_ */
