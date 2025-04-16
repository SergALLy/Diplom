/*
 * walker_config.c
 *
 *  Created on: Apr 15, 2025
 *      Author: SergALLy
 */
#include "walker_config.h"

pca9685_handle_t pca_left = {
		.i2c_handle = &hi2c2,
		.device_address = PCA9685_I2C_DEFAULT_DEVICE_ADDRESS,
		.inverted = false
};

pca9685_handle_t pca_right = {
		.i2c_handle = &hi2c3,
		.device_address = PCA9685_I2C_DEFAULT_DEVICE_ADDRESS,
		.inverted = false
};

leg_handle_t leg1 ={
		.pca_handle = &pca_right,
		.foot =  0,
		.knee =  0,
		.thigh = 0,
};

leg_handle_t leg2 ={
		.pca_handle = &pca_right,
		.foot =  0,
		.knee =  0,
		.thigh = 0,
};

leg_handle_t leg3 ={
		.pca_handle = &pca_right,
		.foot =  0,
		.knee =  0,
		.thigh = 0,
};

leg_handle_t leg4 ={
		.pca_handle = &pca_right,
		.foot =  0,
		.knee =  0,
		.thigh = 0,
};

leg_handle_t leg5 ={
		.pca_handle = &pca_right,
		.foot =  0,
		.knee =  0,
		.thigh = 0,
};

leg_handle_t leg6 ={
		.pca_handle = &pca_right,
		.foot =  0,
		.knee =  0,
		.thigh = 0,
};
