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

static uint16_t angle_2_u16(float angle)
{
	/*
	 * Назначение: Перевод угла в градусах в ШИМ
	 * Входные параметры:
	 * 		angle: Угол в градусах
	 * Return:
	 * 		Значение ШИМ (time off)
	 */
	if (angle <= -135) return MIN_SERVO_POS;
	if (angle >= 135) return MAX_SERVO_POS;
	return (uint16_t)( ((MAX_SERVO_POS - MIN_SERVO_POS) * (angle+135) / 270) + MIN_SERVO_POS );
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

	success &= pca9685_set_channel_pwm_times(handle->leg1->pca_handle, handle->leg1->foot->channel, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg1->pca_handle, handle->leg1->knee->channel, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg1->pca_handle, handle->leg1->thigh->channel, 0, NEULTRAL_SERVO_POS);

	success &= pca9685_set_channel_pwm_times(handle->leg2->pca_handle, handle->leg2->foot->channel, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg2->pca_handle, handle->leg2->knee->channel, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg2->pca_handle, handle->leg2->thigh->channel, 0, NEULTRAL_SERVO_POS);

	success &= pca9685_set_channel_pwm_times(handle->leg3->pca_handle, handle->leg3->foot->channel, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg3->pca_handle, handle->leg3->knee->channel, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg3->pca_handle, handle->leg3->thigh->channel, 0, NEULTRAL_SERVO_POS);

	success &= pca9685_set_channel_pwm_times(handle->leg4->pca_handle, handle->leg4->foot->channel, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg4->pca_handle, handle->leg4->knee->channel, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg4->pca_handle, handle->leg4->thigh->channel, 0, NEULTRAL_SERVO_POS);

	success &= pca9685_set_channel_pwm_times(handle->leg5->pca_handle, handle->leg5->foot->channel, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg5->pca_handle, handle->leg5->knee->channel, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg5->pca_handle, handle->leg5->thigh->channel, 0, NEULTRAL_SERVO_POS);

	success &= pca9685_set_channel_pwm_times(handle->leg6->pca_handle, handle->leg6->foot->channel, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg6->pca_handle, handle->leg6->knee->channel, 0, NEULTRAL_SERVO_POS);
	success &= pca9685_set_channel_pwm_times(handle->leg6->pca_handle, handle->leg6->thigh->channel, 0, NEULTRAL_SERVO_POS);

	return success;
}
