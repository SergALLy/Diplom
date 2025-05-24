/*
 * walker_config.h
 *
 *  Created on: Apr 15, 2025
 *      Author: SergALLy
 */

#ifndef INC_WALKER_CONFIG_H_
#define INC_WALKER_CONFIG_H_

#include "walker.h"
#include "i2c.h"
#include "spi.h"

#define PERIOD_MS		3.0f		// Период ШИМа

#define COXA_LENGTH 	61.0f		// Длина от оси серво coxa до оси серво femur
#define FEMUR_LENGTH	75.3f		// Длина от оси серво femur до оси серво tibia
#define TIBIA_LENGTH	119.3f		// Длина от оси серво tibia до кончика ноги

extern ps2_handle_t ps;				// Дескриптор джойстика PS2
extern pca9685_handle_t pca_left;	// Дескриптор левого драйвера pca9685
extern pca9685_handle_t pca_right;	// Дескриптор правого драйвера pca9685

// Дескрипторы ног шагохода
extern leg_handle_t leg1; // Правая передняя
extern leg_handle_t leg2; // Правая задняя
extern leg_handle_t leg3; // Правая задняя
extern leg_handle_t leg4; // Левая задняя
extern leg_handle_t leg5; // Левая средняя
extern leg_handle_t leg6; // Левая передняя
// Описание шагохода
extern walker_handle_t sky_walker;

// Исходного положение
extern const float home_x[6];
extern const float home_y[6];
extern const float home_z[6];

// Расстояние от центра туловища до оси серво coxa
extern const float body_x[6];
extern const float body_y[6];
extern const float body_z[6];

// Калибровка сервоприводов
extern const int8_t coxa_cal[6];
extern const int8_t femur_cal[6];
extern const int8_t tibia_cal[6];

extern uint8_t mode;
extern uint8_t gait;

#endif /* INC_WALKER_CONFIG_H_ */
