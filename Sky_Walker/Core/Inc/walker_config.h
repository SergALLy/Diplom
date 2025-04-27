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

#define PERIOD_MS		3.0f
#define DURATION		618

#define COXA_LENGTH 	61.0f	// Длина, ближайщего к корпусу сегмента
#define FEMUR_LENGTH	75.3f	// Длина средне
#define TIBIA_LENGTH	119.3f	// Длина голени (последний сегмент)

extern pca9685_handle_t pca_left;
extern pca9685_handle_t pca_right;

extern const leg_handle_t leg1;
extern const leg_handle_t leg2;
extern const leg_handle_t leg3;
extern const leg_handle_t leg4;
extern const leg_handle_t leg5;
extern leg_handle_t leg6;
extern walker_handle_t sky_walker;

// Задание исходного положения
extern const float home_x[6];
extern const float home_y[6];
extern const float home_z[6];

// Расстояние от центра туловища до серво coxa
extern const float body_x[6];
extern const float body_y[6];
extern const float body_z[6];

// калибровка сервоприводов
extern const int8_t coxa_cal[6];
extern const int8_t femur_cal[6];
extern const int8_t tibia_cal[6];

#endif /* INC_WALKER_CONFIG_H_ */
