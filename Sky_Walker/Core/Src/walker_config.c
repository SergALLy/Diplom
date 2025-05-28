/*
 * walker_config.c
 *
 *  Created on: Apr 15, 2025
 *      Author: SergALLy
 */
#include "walker_config.h"

// Исходного положение
const float home_x[6] = {  96.4,    0.0,  -96.4,  -96.4,    0.0,   96.4};
const float home_y[6] = {  96.4,  136.3,   96.4,  -96.4, -136.3,  -96.4};
const float home_z[6] = {  -120,   -120,   -120,   -120,   -120,   -120};

// Расстояние от центра туловища до оси серво coxa
const float body_x[6] = {134.0, 0.0, -134.0, -134.0, 0.0, 134.0};
const float body_y[6] = {81.5, 109.6, 81.5, -81.5, -109.6, -81.5};
const float body_z[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

/* Калибровка сервоприводов
 *  coxa:
 * 		левый:	-angle	-> вперед;	angle	-> назад
 * 		правый:	angle	-> вперед;	-angle	-> назад
 * 	femur:
 * 		anlge	-> вверх;	-angle	-> вниз
 * 	tibia:
 * 		angle	-> вверх;	-angle	-> вниз
 */
const float coxa_cal[6]  = {0, 2, 0, -3, -5, 1};
const float femur_cal[6]  = {0, 6, 1.5, 10, 6, -1};
const float tibia_cal[6] = {0, -3, 3, 10, 19, 9};

// Дескриптор джойстика PS2
ps2_handle_t ps = {
		.spi_handle = &hspi2
};

// Дескриптор левого драйвера pca9685
pca9685_handle_t pca_left = {
		.i2c_handle = &hi2c3,
		.device_address = PCA9685_I2C_DEFAULT_DEVICE_ADDRESS,
		.inverted = false
};

// Дескриптор правого драйвера pca9685
pca9685_handle_t pca_right = {
		.i2c_handle = &hi2c2,
		.device_address = PCA9685_I2C_DEFAULT_DEVICE_ADDRESS,
		.inverted = false
};

// Дескрипторы ног шагохода
leg_handle_t leg1 ={ // Правая передняя
		.pca_handle = &pca_right,
		.tibia =  15,
		.femur =  14,
		.coxa = 13,
		.X = home_x[0],
		.Y = home_y[0],
		.Z = home_z[0]
};

leg_handle_t leg2 ={ // Правая задняя
		.pca_handle = &pca_right,
		.tibia =  7,
		.femur =  6,
		.coxa = 5,
		.X = home_x[1],
		.Y = home_y[1],
		.Z = home_z[1]
};

leg_handle_t leg3 ={ // Правая задняя
		.pca_handle = &pca_right,
		.tibia =  2,
		.femur =  1,
		.coxa = 0,
		.X = home_x[2],
		.Y = home_y[2],
		.Z = home_z[2]
};

leg_handle_t leg4 ={ // Левая задняя
		.pca_handle = &pca_left,
		.tibia =  2,
		.femur =  1,
		.coxa = 0,
		.X = home_x[3],
		.Y = home_y[3],
		.Z = home_z[3]
};

leg_handle_t leg5 ={ // Левая средняя
		.pca_handle = &pca_left,
		.tibia =  7,
		.femur =  6,
		.coxa = 5,
		.X = home_x[4],
		.Y = home_y[4],
		.Z = home_z[4]
};

leg_handle_t leg6 ={ // Левая передняя
		.pca_handle = &pca_left,
		.tibia =  15,
		.femur =  14,
		.coxa = 13,
		.X = home_x[5],
		.Y = home_y[5],
		.Z = home_z[5]
};
// Описание шагохода
walker_handle_t sky_walker = {};

// Флаги
uint8_t mode; // Выбор режима
uint8_t gait; // Выбор походки

// Приращения для изменения положения корпуса
float offset_x[6] = {0, 0, 0, 0, 0, 0};
float offset_y[6] = {0, 0, 0, 0, 0, 0};
float offset_z[6] = {0, 0, 0, 0, 0, 0};
