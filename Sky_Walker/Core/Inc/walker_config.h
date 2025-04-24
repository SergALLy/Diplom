/*
 * walker_config.h
 *
 *  Created on: Apr 15, 2025
 *      Author: SergALLy
 */

#ifndef INC_WALKER_CONFIG_H_
#define INC_WALKER_CONFIG_H_

#include "walker.h"
#include "stdbool.h"
#include "i2c.h"

extern pca9685_handle_t pca_left;
extern pca9685_handle_t pca_right;
extern leg_handle_t leg1;
extern leg_handle_t leg2;
extern leg_handle_t leg3;
extern leg_handle_t leg4;
extern leg_handle_t leg5;
extern leg_handle_t leg6;

#endif /* INC_WALKER_CONFIG_H_ */
