/*
 * walker.c
 *
 *  Created on: Apr 14, 2025
 *      Author: SergALLy
 */
#include "walker.h"
#include "assert.h"
#include "math.h"
#include "stdlib.h"
/*
 *  270 -> 2.5 мс -> 3413
 * 135 - 1.5 мс -> 2048
 * 0 -> 0.5 мс -> 683
 */
#define MAX_SERVO_POS		2958 // Зачение ШИМа в положении серва 225deg
#define MIN_SERVO_POS		1138  // Зачение ШИМа в положении серва 45deg

#define RAD_TO_DEG	(180.0 / M_PI)
#define DEG_TO_RAD	(M_PI / 180.0)

static uint16_t tick = 0;
static float step_len_X, step_len_Y, sin_rot_Z, cos_rot_Z;

uint8_t tripod_mode[6] = { 1, 2, 1, 2, 1, 2 };

static float map(float min_1, float max_1, float value, float min_2,
		float max_2) {
	/*
	 *  Назначение: перерасчёт значения из 1 интервала в значение из 2 интервала
	 *  Входные параматеры:
	 *  	min_1: Нижняя граница исходного интервала
	 *  	max_1: Верхняя граница исходного интервала
	 *  	value: Значение в исходном интервале
	 *  	min_2: Нижняя граница нового интервала
	 *  	max_2^ Верхняя граница нового интервала
	 *  Return:
	 *  	Значение в новом интервале
	 */
	if (value <= min_1)
		return min_2;
	if (value == 0)
		return (max_2 - min_2) / 2 + min_2;
	if (value >= max_1)
		return max_2;
	return (value - min_1) / (max_1 - min_1) * (max_2 - min_2) + min_2;
}

static float constrain(float x, float min, float max) {

	/*
	 * Ограничивает значение x в диапазоне [min, max]
	 * @param x   - входное значение
	 * @param min - минимальное допустимое значение
	 * @param max - максимальное допустимое значение
	 * @return    - значение x, "зажатое" между min и max
	 */

	if (x < min)
		return min;
	if (x > max)
		return max;
	return x;
}

static uint16_t angle_2_u16(float angle) {
	/*
	 * Назначение: Перевод угла в градусах в ШИМ
	 * Входные параметры:
	 * 		angle: Углов в градусах в пределах от [0; 180]
	 * Return:
	 * 		Значение ШИМ (time off)
	 */
	assert(angle >= 0);
	assert(angle <= 180);

	return map(0, 180, angle, MIN_SERVO_POS, MAX_SERVO_POS);
}

static void calc_step_len(int8_t x, int8_t y, int8_t z) {
	step_len_X = 90 * x / 127;  // перемещения по оси X
	step_len_Y = 90 * y / 127;  // перемещения по оси Y
	float Z = 35 * z / 127;  // перемещения по оси Z

	sin_rot_Z = sin(Z * DEG_TO_RAD);
	cos_rot_Z = cos(Z * DEG_TO_RAD);
}

static void calc_ampl(uint8_t leg_num, float *amp) {
	float total_X = home_x[leg_num] + body_x[leg_num];
	float total_Y = home_x[leg_num] + body_x[leg_num];

	float rot_offset_x = total_X * sin_rot_Z + total_X * cos_rot_Z - total_X;
	float rot_offset_y = total_Y * cos_rot_Z - total_Y * sin_rot_Z - total_Y;

	amp[0] = (step_len_X + rot_offset_x) / 2.0;
	amp[1] = (step_len_Y + rot_offset_y) / 2.0;
	amp[0] = constrain(amp[0], -50, 50);
	amp[1] = constrain(amp[1], -50, 50);

	if (fabs(step_len_X + rot_offset_x) > fabs(step_len_Y + rot_offset_y))
		amp[2] = (step_len_X + rot_offset_x) / 4.0;
	else
		amp[2] = (step_len_Y + rot_offset_y) / 4.0;
}

static bool walker_servo_write(leg_handle_t *leg, float coxa_angle,
		float femur_angle, float tibia_angle) {
	bool success = true;

	success &= pca9685_set_channel_pwm_times(leg->pca_handle, leg->coxa, 0,
			angle_2_u16(coxa_angle));
	success &= pca9685_set_channel_pwm_times(leg->pca_handle, leg->femur, 0,
			angle_2_u16(femur_angle));
	success &= pca9685_set_channel_pwm_times(leg->pca_handle, leg->tibia, 0,
			angle_2_u16(tibia_angle));
	return success;
}

/*
 *  coxa:
 * 		левый:	-angle	-> вперед;	angle	-> назад
 * 		правый:	angle	-> вперед;	-angle	-> назад
 * 	femur:
 * 		anlge	-> вверх;	-angle	-> вниз
 * 	tibia:
 * 		angle	-> вверх;	-angle	-> вниз
 */

bool walker_calc_ik(walker_handle_t *walker, uint8_t leg_number, float x,
		float y, float z) {
	/*
	 * Назначение: Расчёт углов сервоприводов
	 * Входные параметры:
	 * 		x: Значение скорости по оси x (движение прямо)
	 * 		y: Значение скорости по оси y (движение вбок)
	 * 		angle: Массив значений углов
	 * Return:
	 * 		Массив значений ШИМа = {
	 * 			ШИМ бедра,
	 * 			ШИМ колена,
	 * 			ШИМ стопы
	 * 		}
	 */
	bool success = false;
	float L0, L3;
	float phi_tibia, gamma_femur, phi_femur;
	float coxa_angle, femur_angle, tibia_angle;

	L0 = sqrt(pow(x, 2) + pow(y, 2)) - COXA_LENGTH;
	L3 = sqrt(pow(L0, 2) + pow(z, 2));

	// Проверка достижимости
	if ((L3 < (TIBIA_LENGTH + FEMUR_LENGTH))
			&& (L3 > (TIBIA_LENGTH - FEMUR_LENGTH))) {
		success = true;
		// расчёт угла для серво coxa
		coxa_angle = atan2(x, y) * RAD_TO_DEG + coxa_cal[leg_number];
		// расчёт угла для серво femur
		gamma_femur = atan2(z, L0);
		phi_femur = acos(
				(pow(FEMUR_LENGTH, 1) + pow(L3, 1) - pow(TIBIA_LENGTH, 1))
						/ (2 * FEMUR_LENGTH * L3));
		femur_angle = (phi_femur + gamma_femur) * RAD_TO_DEG + 14.0 + 90.0
				+ femur_cal[leg_number];
		femur_angle = constrain(femur_angle, 0.0, 180.0);
		// расчёт угла для серво tibia (последний)
		phi_tibia = acos(
				(pow(FEMUR_LENGTH, 2) + pow(TIBIA_LENGTH, 2) - pow(L3, 2))
						/ (2 * FEMUR_LENGTH * TIBIA_LENGTH));
		tibia_angle = phi_tibia * RAD_TO_DEG - 23.0 + tibia_cal[leg_number];
		tibia_angle = constrain(tibia_angle, 0.0, 180.0);
		//output to the appropriate leg
		switch (leg_number) {
		case 0:
			coxa_angle = coxa_angle + 45.0;        //compensate for leg mounting
			coxa_angle = constrain(coxa_angle, 0.0, 180.0);
			success &= walker_servo_write(walker[leg_number], coxa_angle,
					femur_angle, tibia_angle);
			break;
		case 1:
			coxa_angle = coxa_angle + 90.0;        //compensate for leg mounting
			coxa_angle = constrain(coxa_angle, 0.0, 180.0);
			success &= walker_servo_write(walker[leg_number], coxa_angle,
					femur_angle, tibia_angle);
			break;
		case 2:
			coxa_angle = coxa_angle + 135.0;       //compensate for leg mounting
			coxa_angle = constrain(coxa_angle, 0.0, 180.0);
			success &= walker_servo_write(walker[leg_number], coxa_angle,
					femur_angle, tibia_angle);
			break;
		case 3:
			if (coxa_angle < 0)                    //compensate for leg mounting
				coxa_angle = coxa_angle + 225.0;       // (need to use different
			else
				//  positive and negative offsets
				coxa_angle = coxa_angle - 135.0; //  due to atan2 results above!)
			coxa_angle = constrain(coxa_angle, 0.0, 180.0);
			success &= walker_servo_write(walker[leg_number], coxa_angle,
					femur_angle, tibia_angle);
			break;
		case 4:
			if (coxa_angle < 0)                    //compensate for leg mounting
				coxa_angle = coxa_angle + 270.0;       // (need to use different
			else
				//  positive and negative offsets
				coxa_angle = coxa_angle - 90.0; //  due to atan2 results above!)
			coxa_angle = constrain(coxa_angle, 0.0, 180.0);
			success &= walker_servo_write(walker[leg_number], coxa_angle,
					femur_angle, tibia_angle);
			break;
		case 5:
			if (coxa_angle < 0)                    //compensate for leg mounting
				coxa_angle = coxa_angle + 315.0;       // (need to use different
			else
				//  positive and negative offsets
				coxa_angle = coxa_angle - 45.0; //  due to atan2 results above!)
			coxa_angle = constrain(coxa_angle, 0.0, 180.0);
			success &= walker_servo_write(walker[leg_number], coxa_angle,
					femur_angle, tibia_angle);
			break;
		}
	}
	return success;
}

bool walker_init(pca9685_handle_t *pca_1, pca9685_handle_t *pca_2) {
	/*
	 * Назначение: Инициализация драйверов pca9685
	 * Входные параметры:
	 * 		pca_1, pca_2: дескрипторы драйверов pca9685
	 * Return:
	 * 		True - инициализация успешно
	 * 		False - иначе
	 */
	sky_walker[0] = leg1;
	sky_walker[1] = leg2;
	sky_walker[2] = leg3;
	sky_walker[3] = leg4;
	sky_walker[4] = leg5;
	sky_walker[5] = leg6;

	bool success = true;

	success &= pca9685_init(pca_1);
	success &= pca9685_set_pwm_frequency(pca_1, 333.3f);
	success &= pca9685_init(pca_2);
	success &= pca9685_set_pwm_frequency(pca_2, 333.3f);

	return success;
}

bool walker_servo_nelrtal(walker_handle_t *walker) {
	/*
	 * Назначение: Вывод всех сервоприводов в нейтральное положение
	 * Входные параметры:
	 * 		walker: дескриптор робота
	 * Return:
	 * 		True - успешно
	 * 		False - иначе
	 */
	bool success = true;

	for (uint8_t i = 0; i < 6; i++) {
		success &= walker_servo_write(walker[i], 90 + coxa_cal[i],
				90 + femur_cal[i], 90 + tibia_cal[i]);
	}
	return success;
}

void walker_tripod_mode(ps2_handle_t *ps, walker_handle_t *walker) {
	float amplitudes[3] = { 0, 0, 0 };
	int8_t RX = ps->right_stick.X;
	int8_t RY = ps->right_stick.Y;
	int8_t LX = ps->left_stick.X;

	//if commands more than deadband then process
	if ((abs(RX) > 15) || (abs(RY) > 15) || (abs(LX) > 15) || (tick > 0)) {
		calc_step_len(RX, RY, LX);
		uint16_t numTicks = round(DURATION / PERIOD_MS / 2.0); //total ticks divided into the two cases
		float phi = M_PI * tick / numTicks;
		for (uint8_t leg_num = 0; leg_num < 6; leg_num++) {
			calc_ampl(leg_num, amplitudes);
			switch (tripod_mode[leg_num]) {
			case 1:                        //move foot forward (raise and lower)
				walker[leg_num]->X = home_x[leg_num] - amplitudes[0] * cos(phi);
				walker[leg_num]->Y = home_y[leg_num] - amplitudes[1] * cos(phi);
				walker[leg_num]->Z = home_z[leg_num]
						+ fabs(amplitudes[2]) * sin(phi);
				if (tick >= numTicks - 1)
					tripod_mode[leg_num] = 2; // смена режима
				break;
			case 2:                             //move foot back (on the ground)
				walker[leg_num]->X = home_x[leg_num] + amplitudes[0] * cos(phi);
				walker[leg_num]->Y = home_y[leg_num] + amplitudes[1] * cos(phi);
				walker[leg_num]->Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					tripod_mode[leg_num] = 1; // смена режима
				break;
			}
		}
		//increment tick
		if (tick < numTicks - 1)
			tick++;
		else
			tick = 0;
	}
}
