/*
 * walker.c
 *
 *  Created on: Apr 14, 2025
 *      Author: SergALLy
 */
#include "walker.h"

 /*
  *  270 -> 2.5 мс -> 3413
 * 135 - 1.5 мс -> 2048
 * 0 -> 0.5 мс -> 683
 */
#define MAX_SERVO_POS		3413 // 4096*2.5/3
#define NEULTRAL_SERVO_POS	2048 // 4096*1.5/3
#define MIN_SERVO_POS		683  // 4096*0.5/3

#define KNEE_ANGLE_HIGH		10

#define THIGH		0
#define KNEE_HIGH	1
#define KNEE_LOW	2
#define FOOT		3

static float map(float min_1, float max_1, float value, float min_2, float max_2)
{
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
	if (value <= min_1) return min_2;
    if (value == 0) return (max_2 - min_2)/2+min_2;
	if (value >= max_1) return max_2;
	return (value-min_1)/(max_1-min_1)*(max_2-min_2) + min_2;
}

static uint16_t angle_2_u16(float angle)
{
	/*
	 * Назначение: Перевод угла в градусах в ШИМ
	 * Входные параметры:
	 * 		pwm: Массив значений ШИМ
	 * 		angle: Массив углов в градусах
	 * Return:
	 * 		Массив значений ШИМ (time off)
	 */
	return map(-135, 135, angle, MIN_SERVO_POS,MAX_SERVO_POS);
}

static void walker_angle(int8_t x, int8_t y, float *angle)
{
	/*
	 * Назначение: Расчёт углов сервоприводов
	 * Входные параметры:
	 * 		x: Значение скорости по оси x
	 * 		y: Значение скорости по оси y
	 * 		angle: Массив значений углов
	 * Return:
	 * 		Массив значений ШИМа = {
	 * 			ШИМ бедра,
	 * 			ШИМ колена,
	 * 			ШИМ стопы
	 * 		}
	 */

	angle[THIGH] = -map(-128, 127, y, MIN_ANGLE, MAX_ANGLE); // Угол бедра
	angle[KNEE_HIGH] = KNEE_ANGLE_HIGH; // Угол колена
	angle[KNEE_LOW] = 0;
	angle[FOOT] = 0; // Угол стопы
}

bool walker_init(pca9685_handle_t* pca_1, pca9685_handle_t* pca_2)
{
	/*
	 * Назначение: Инициализация драйверов pca9685
	 * Входные параметры:
	 * 		pca_1, pca_2: дескрипторы драйверов pca9685
	 * Return:
	 * 		True - инициализация успешно
	 * 		False - иначе
	 */
	bool success = true;

	success &= pca9685_init(pca_1);
	success &= pca9685_set_pwm_frequency(pca_1, 333.3f);
	success &= pca9685_init(pca_2);
	success &= pca9685_set_pwm_frequency(pca_2, 333.3f);

	return success;
}

bool walker_servo_neultral(walker_handle_t *handle)
{
	/*
	 * Назначение: Вывод всех сервоприводов в нейтральное положение
	 * Входные параметры:
	 * 		handle: дескриптор робота
	 * Return:
	 * 		True - успешно
	 * 		False - иначе
	 */
	bool success = true;

	success &= pca9685_set_channel_pwm_times(handle->leg_1->pca_handle, handle->leg_1->foot, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg_1->pca_handle, handle->leg_1->knee, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg_1->pca_handle, handle->leg_1->thigh, 0, NEULTRAL_SERVO_POS);

	success &= pca9685_set_channel_pwm_times(handle->leg_2->pca_handle, handle->leg_2->foot, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg_2->pca_handle, handle->leg_2->knee, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg_2->pca_handle, handle->leg_2->thigh, 0, NEULTRAL_SERVO_POS);

	success &= pca9685_set_channel_pwm_times(handle->leg_3->pca_handle, handle->leg_3->foot, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg_3->pca_handle, handle->leg_3->knee, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg_3->pca_handle, handle->leg_3->thigh, 0, NEULTRAL_SERVO_POS);

	success &= pca9685_set_channel_pwm_times(handle->leg_4->pca_handle, handle->leg_4->foot, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg_4->pca_handle, handle->leg_4->knee, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg_4->pca_handle, handle->leg_4->thigh, 0, NEULTRAL_SERVO_POS);

	success &= pca9685_set_channel_pwm_times(handle->leg_5->pca_handle, handle->leg_5->foot, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg_5->pca_handle, handle->leg_5->knee, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg_5->pca_handle, handle->leg_5->thigh, 0, NEULTRAL_SERVO_POS);

	success &= pca9685_set_channel_pwm_times(handle->leg_6->pca_handle, handle->leg_6->foot, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg_6->pca_handle, handle->leg_6->knee, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg_6->pca_handle, handle->leg_6->thigh, 0, NEULTRAL_SERVO_POS);

	return success;
}

/* 					Алгоритм "Шаг вперед"
 *
 * 1) нога 1, 2, 3 - колено angle_knee_high (подъём)
 * 2) нога 1, 2, 3 - бедро +angle_thigh (перенос ног)
 * 3) нога 1, 2, 3 - колено angle_knee_low (опускание)
 * 4) нога 1, 2, 3 - бедро 0 (перенос тела)
 *    нога 4, 5, 6 - бедро -angle_thigh
 *
 * 5) нога 4, 5, 6 - колено angle_knee_high (подъём)
 * 6) нога 4, 5, 6 - бедро +angle_thigh (перенос ног)
 * 7) нога 4, 5, 6 - колено angle_knee_low (опускание)
 * 8) нога 1, 2, 3 - бедро -angle_thigh (перенос тела)
 * 	  нога 4, 5, 6 - бедро 0 (перенос тела)
 */
bool walker_run(walker_handle_t *walker, stick *stick)
{
	bool success= true;
	float angle[4] = {0};
	uint16_t pwm=0;
	walker_angle(stick->X, stick->Y, angle);

	// Первая Фаза (ШАГ 1,2,3 ногами)
	// нога 1, 2, 3 - колено angle_knee_high (подъём)
	pwm = angle_2_u16(angle[KNEE_HIGH]);
	success &= pca9685_set_channel_pwm_times(walker->leg_1->pca_handle, walker->leg_1->knee, 0, pwm);
	success &= pca9685_set_channel_pwm_times(walker->leg_2->pca_handle, walker->leg_2->knee, 0, pwm);
	success &= pca9685_set_channel_pwm_times(walker->leg_3->pca_handle, walker->leg_3->knee, 0, pwm);
	HAL_Delay(20);
	// нога 1, 2, 3 - бедро +angle_thigh (перенос ног)
	pwm = angle_2_u16(angle[THIGH]);
	success &= pca9685_set_channel_pwm_times(walker->leg_1->pca_handle, walker->leg_1->thigh, 0, pwm);
	success &= pca9685_set_channel_pwm_times(walker->leg_2->pca_handle, walker->leg_2->thigh, 0, pwm);
	success &= pca9685_set_channel_pwm_times(walker->leg_3->pca_handle, walker->leg_3->thigh, 0, pwm);
	HAL_Delay(20);
	// нога 1, 2, 3 - колено angle_knee_low (опускание)
	pwm = angle_2_u16(angle[KNEE_LOW]);
	success &= pca9685_set_channel_pwm_times(walker->leg_1->pca_handle, walker->leg_1->knee, 0, pwm);
	success &= pca9685_set_channel_pwm_times(walker->leg_2->pca_handle, walker->leg_2->knee, 0, pwm);
	success &= pca9685_set_channel_pwm_times(walker->leg_3->pca_handle, walker->leg_3->knee, 0, pwm);
	HAL_Delay(20);

	// Вторая фаза (перенос тела)
	// нога 1, 2, 3 - бедро 0
	success &= pca9685_set_channel_pwm_times(walker->leg_1->pca_handle, walker->leg_1->thigh, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(walker->leg_2->pca_handle, walker->leg_2->thigh, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(walker->leg_3->pca_handle, walker->leg_3->thigh, 0, NEULTRAL_SERVO_POS);
	// нога 4, 5, 6 - бедро -angle_thigh
	pwm = angle_2_u16(-angle[THIGH]);
	success &= pca9685_set_channel_pwm_times(walker->leg_4->pca_handle, walker->leg_4->thigh, 0, pwm);
	success &= pca9685_set_channel_pwm_times(walker->leg_5->pca_handle, walker->leg_5->thigh, 0, pwm);
	success &= pca9685_set_channel_pwm_times(walker->leg_6->pca_handle, walker->leg_6->thigh, 0, pwm);
	HAL_Delay(20);

	walker_angle(stick->X, stick->Y, angle);

	// Третья Фаза (перенос ШАГ ногами 4,5,6)
	// нога 4, 5, 6 - колено angle_knee_high (подъём)
	pwm = angle_2_u16(angle[KNEE_HIGH]);
	success &= pca9685_set_channel_pwm_times(walker->leg_4->pca_handle, walker->leg_4->knee, 0, pwm);
	success &= pca9685_set_channel_pwm_times(walker->leg_5->pca_handle, walker->leg_5->knee, 0, pwm);
	success &= pca9685_set_channel_pwm_times(walker->leg_6->pca_handle, walker->leg_6->knee, 0, pwm);
	HAL_Delay(20);
	// нога 4, 5, 6 - бедро +angle_thigh (перенос ног)
	pwm = angle_2_u16(angle[THIGH]);
	success &= pca9685_set_channel_pwm_times(walker->leg_4->pca_handle, walker->leg_4->thigh, 0, pwm);
	success &= pca9685_set_channel_pwm_times(walker->leg_5->pca_handle, walker->leg_5->thigh, 0, pwm);
	success &= pca9685_set_channel_pwm_times(walker->leg_6->pca_handle, walker->leg_6->thigh, 0, pwm);
	HAL_Delay(20);
	// нога 4, 5, 6 - колено angle_knee_low (опускание)
	pwm = angle_2_u16(angle[KNEE_LOW]);
	success &= pca9685_set_channel_pwm_times(walker->leg_4->pca_handle, walker->leg_4->knee, 0, pwm);
	success &= pca9685_set_channel_pwm_times(walker->leg_5->pca_handle, walker->leg_5->knee, 0, pwm);
	success &= pca9685_set_channel_pwm_times(walker->leg_6->pca_handle, walker->leg_6->knee, 0, pwm);
	HAL_Delay(20);

	// Четвертая фаза (Переенос тела)
	// нога 1, 2, 3 - бедро -angle_thigh
	pwm = angle_2_u16(-angle[THIGH]);
	success &= pca9685_set_channel_pwm_times(walker->leg_1->pca_handle, walker->leg_1->thigh, 0, pwm);
	success &= pca9685_set_channel_pwm_times(walker->leg_2->pca_handle, walker->leg_2->thigh, 0, pwm);
	success &= pca9685_set_channel_pwm_times(walker->leg_3->pca_handle, walker->leg_3->thigh, 0, pwm);
	// нога 4, 5, 6 - бедро 0
	success &= pca9685_set_channel_pwm_times(walker->leg_4->pca_handle, walker->leg_4->thigh, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(walker->leg_5->pca_handle, walker->leg_5->thigh, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(walker->leg_6->pca_handle, walker->leg_6->thigh, 0, NEULTRAL_SERVO_POS);
	HAL_Delay(20);

	return success;
}

