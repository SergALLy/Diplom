/*
 * walker_config.c
 *
 *  Created on: Apr 15, 2025
 *      Author: SergALLy
 */
#include "walker_config.h"

const float home_x[6] = {  96.4,    0.0,  -96.4,  -96.4,    0.0,   96.4};
//const float home_x[6] = {  96.4,    0.0,  -96.4,  -96.4,    0.0,   96.4};
const float home_y[6] = {  96.4,  136.3,   96.4,  -96.4, -136.3,  -96.4};
//const float home_z[6] = {  -150,   -150,   -150,   -150,   -150,   -150};
const float home_z[6] = {  -115,   -115,   -115,   -115,   -115,   -115};
// Расстояние от центра корпуса до серва coxa
const float body_x[6] = {134.0, 0.0, -134.0, -134.0, 0.0, 134.0};
const float body_y[6] = {81.5, 109.6, 81.5, -81.5, -109.6, -81.5};
const float body_z[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

const int8_t coxa_cal[6]  = {0, 1, 0, -1, -1, 1};
const int8_t femur_cal[6]  = {0, 0, 0, 0, 0, 0};
const int8_t tibia_cal[6] = {0, 0, 0, 0, 0, 0};

pca9685_handle_t pca_left = {
		.i2c_handle = &hi2c3,
		.device_address = PCA9685_I2C_DEFAULT_DEVICE_ADDRESS,
		.inverted = false
};

pca9685_handle_t pca_right = {
		.i2c_handle = &hi2c2,
		.device_address = PCA9685_I2C_DEFAULT_DEVICE_ADDRESS,
		.inverted = false
};

const leg_handle_t leg1 ={
		.pca_handle = &pca_right,
		.tibia =  15,
		.femur =  14,
		.coxa = 13,
		.X = home_x[0],
		.Y = home_y[0],
		.Z = home_z[0]
};

const leg_handle_t leg2 ={
		.pca_handle = &pca_right,
		.tibia =  7,
		.femur =  6,
		.coxa = 5,
		.X = home_x[1],
		.Y = home_y[1],
		.Z = home_z[1]
};

const leg_handle_t leg3 ={
		.pca_handle = &pca_right,
		.tibia =  2,
		.femur =  1,
		.coxa = 0,
		.X = home_x[2],
		.Y = home_y[2],
		.Z = home_z[2]
};

const leg_handle_t leg4 ={
		.pca_handle = &pca_left,
		.tibia =  2,
		.femur =  1,
		.coxa = 0,
		.X = home_x[3],
		.Y = home_y[3],
		.Z = home_z[3]
};

const leg_handle_t leg5 ={
		.pca_handle = &pca_left,
		.tibia =  7,
		.femur =  6,
		.coxa = 5,
		.X = home_x[4],
		.Y = home_y[4],
		.Z = home_z[4]
};

leg_handle_t leg6 ={
		.pca_handle = &pca_left,
		.tibia =  15,
		.femur =  14,
		.coxa = 13,
		.X = home_x[5],
		.Y = home_y[5],
		.Z = home_z[5]
};

walker_handle_t sky_walker = {};

