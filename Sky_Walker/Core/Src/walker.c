/*
 * walker.c
 *
 *  Created on: Apr 14, 2025
 *      Author: SergALLy
 */
#include "walker.h"
#include "math.h"
#include "stdlib.h"
#include "usart.h"

#define MAX_SERVO_POS		2958 						// Зачение ШИМа в положении серва 225deg
#define MIN_SERVO_POS		1138  						// Зачение ШИМа в положении серва 45deg
#define A12DEG				209440L						// 12 градусов в радианах * 1000000
#define A30DEG				523599L						// 30 градусов в радианах * 1000000

#define RAD_TO_DEG	(180.0 / M_PI)						// Перевод из радиан в градусы
#define DEG_TO_RAD	(M_PI / 180.0)						// Перевод из градусов в радианы

#define MAX_MOVING_BODY		30							// Максимальное перемещение корпуса

static uint8_t flag = 0;								// Флаг

static uint16_t tick = 0;								// Вспомогательная, для плавного шага
static uint16_t duration = 314;							// Продолжительность шага

static float step_len_X, step_len_Y;					// Вспомогательные, длина шага
static float sin_rot_Z, cos_rot_Z;						// Вспомогательные, поворот вокруг оси z
static float step_hight_coef = 1.0;						// Коэффициент высоты шага

// Значение углов ног для управления передними
float leg1_coxa, leg1_femur, leg1_tibia;
float leg6_coxa, leg6_femur, leg6_tibia;

static uint8_t tripod_mode[6] = { 1, 2, 1, 2, 1, 2 };	// Порядок фаз для режима "треугольник"
static uint8_t wave_mode[6] = { 1, 2, 3, 4, 5, 6 };		// Порядок фаз для режима "волна"
static uint8_t tetrapod_mode[6] = { 1, 2, 3, 1, 3, 2 };	// Порядок фаз для режима "тетрапод"
static uint8_t ripple_mode[6] = { 2, 6, 4, 1, 3, 5 };	// Порядок фаз для режима "насекомое"

static float map(float min_1, float max_1, float value, float min_2,
		float max_2) {
	/*
	 *  Назначение: перерасчёт значения из 1 интервала в значение из 2 интервала
	 *  Входные параматеры:
	 *  	min_1: Нижняя граница исходного интервала
	 *  	max_1: Верхняя граница исходного интервала
	 *  	value: Значение в исходном интервале
	 *  	min_2: Нижняя граница нового интервала
	 *  	max_2: Верхняя граница нового интервала
	 *  Return:
	 *  	Значение в интервале 2
	 */
	if (value <= min_1)
		return min_2;
	if (value >= max_1)
		return max_2;
	if (value == 0)
		return (max_2 - min_2) / 2 + min_2;
	return (value - min_1) / (max_1 - min_1) * (max_2 - min_2) + min_2;
}

static float constrain(float x, float min, float max) {

	/*
	 * Назначение: Ограничивает значение x в диапазоне [min, max]
	 * Входные параметры:
	 * 		x: входное значение
	 * 		min: минимальное допустимое значение
	 * 		max: - максимальное допустимое значение
	 * Return:
	 * 		значение x, "зажатое" между min и max
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
	if (angle < 0)
		angle = 0;
	if (angle > 180)
		angle = 180;

	return map(0, 180, angle, MIN_SERVO_POS, MAX_SERVO_POS);
}

static void calc_step_len(int8_t x, int8_t y, int8_t z) {
	/*
	 * Назначение:расчёт длины шага и угла поворота по оси Z
	 * Входные параметры:
	 * 		x, y, z: прирошения по оси x, y, z
	 */
	// Вычисление длина шага
	step_len_X = 90 * x / 127;
	step_len_Y = 90 * y / 127;
	float Z = 35 * z / 127;
	// Вычисление тригонометрии поворта
	sin_rot_Z = sin(Z * DEG_TO_RAD);
	cos_rot_Z = cos(Z * DEG_TO_RAD);
}

static void calc_ampl(uint8_t leg_num, float *amp) {
	/*
	 * Назначение расчёт амплитуд шага
	 * Входные параметры:
	 * 		leg_num: номер ноги, для которой высчитается амплитуды шага
	 * 		amp: массив амплитуд шаг по осям x, y z
	 */
	// Расчёт положения ноги в исходном положении относительно системы координат корпуса
	float total_X = home_x[leg_num] + body_x[leg_num];
	float total_Y = home_y[leg_num] + body_y[leg_num];
	// Расчёт смешений по осям x, y при повороте вокруг оси z
	float rot_offset_x = total_Y * sin_rot_Z + total_X * cos_rot_Z - total_X;
	float rot_offset_y = total_Y * cos_rot_Z - total_X * sin_rot_Z - total_Y;
	// Расчёт амплитуд
	amp[0] = (step_len_X + rot_offset_x) / 2.0;
	amp[1] = (step_len_Y + rot_offset_y) / 2.0;
	// Ограничение амплитуд
	amp[0] = constrain(amp[0], -50, 50);
	amp[1] = constrain(amp[1], -50, 50);
	// Амплитуда по оси z как максимум из амплитуд по осям x, y
	if (fabs(step_len_X + rot_offset_x) > fabs(step_len_Y + rot_offset_y))
		amp[2] = step_hight_coef * (step_len_X + rot_offset_x) / 4.0;
	else
		amp[2] = step_hight_coef * (step_len_Y + rot_offset_y) / 4.0;
}

static bool walker_servo_write(uint8_t leg_num, float coxa_angle,
		float femur_angle, float tibia_angle) {
	/*
	 * Назначение: Поворот сервоприводов ноги на заданные углы
	 * Входные параметры:
	 * 		leg_num: ноги ноги
	 * 		coxa_angle: угол поворота серво coxa
	 * 		femur_angle: угол поворота серво femur
	 * 		tibia_angle: угол поворота серво tibia
	 * 	Return:
	 * 		True - успешно
	 * 		False - иначе
	 */
	bool success = true;

	success &= pca9685_set_channel_pwm_times(sky_walker[leg_num].pca_handle,
			sky_walker[leg_num].coxa, 0, angle_2_u16(coxa_angle)); // Поворот серво coxa
	success &= pca9685_set_channel_pwm_times(sky_walker[leg_num].pca_handle,
			sky_walker[leg_num].femur, 0, angle_2_u16(femur_angle)); // Поворот серво femur
	success &= pca9685_set_channel_pwm_times(sky_walker[leg_num].pca_handle,
			sky_walker[leg_num].tibia, 0, angle_2_u16(tibia_angle)); // Поворот серво tibia
	return success;
}

bool walker_calc_ik(uint8_t leg_number, float x, float y, float z) {
	/*
	 *  Назначение: Расчёт задачи обратной кинематики (ОК) для 1 ноги и перенос ноги в положение с координатами (x,y,z)
	 *  Входные параметры:
	 *  	 leg_number: номер ноги робота, для которой необходимо решить задаи ОК
	 *  	 x, y, z: координаты положения ноги, куда её надо перенести
	 *  return:
	 * 		True - успешно
	 * 		False - иначе
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
		// Расчёт угла для серво coxa
		coxa_angle = atan2(x, y) * RAD_TO_DEG + coxa_cal[leg_number];
		// Расчёт угла для серво femur
		gamma_femur = atan2(z, L0);
		phi_femur = acos(
				(pow(FEMUR_LENGTH, 2) + pow(L3, 2) - pow(TIBIA_LENGTH, 2))
						/ (2 * FEMUR_LENGTH * L3));
		femur_angle = (phi_femur + gamma_femur) * RAD_TO_DEG + 14.0 + 90.0
				+ femur_cal[leg_number];
		femur_angle = constrain(femur_angle, 0.0, 180.0);
		// Расчёт угла для серво tibia (последний)
		phi_tibia = acos(
				(pow(FEMUR_LENGTH, 2) + pow(TIBIA_LENGTH, 2) - pow(L3, 2))
						/ (2 * FEMUR_LENGTH * TIBIA_LENGTH));
		tibia_angle = phi_tibia * RAD_TO_DEG - 23.0 + tibia_cal[leg_number];
		tibia_angle = constrain(tibia_angle, 0.0, 180.0);
		// Перенос ноги в положение (x,y,z)
		switch (leg_number) {
		case 0:
			if (flag_IK_control_leg & (1 << 0)) {
				coxa_angle = coxa_angle + 45.0;     // Компенсация крепления ноги
				coxa_angle = constrain(coxa_angle, 0.0, 180.0);
				success &= walker_servo_write(leg_number, coxa_angle,
						femur_angle, tibia_angle);
			}
			break;
		case 1:
			coxa_angle = coxa_angle + 90.0;        // Компенсация крепления ноги
			coxa_angle = constrain(coxa_angle, 0.0, 180.0);
			success &= walker_servo_write(leg_number, coxa_angle, femur_angle,
					tibia_angle);
			break;
		case 2:
			coxa_angle = coxa_angle + 135.0;       // Компенсация крепления ноги
			coxa_angle = constrain(coxa_angle, 0.0, 180.0);
			success &= walker_servo_write(leg_number, coxa_angle, femur_angle,
					tibia_angle);
			break;
		case 3:
			if (coxa_angle < 0)                    // Компенсация крепления ноги
				coxa_angle = coxa_angle + 225.0;
			else
				coxa_angle = coxa_angle - 135.0;
			coxa_angle = constrain(coxa_angle, 0.0, 180.0);
			success &= walker_servo_write(leg_number, coxa_angle, femur_angle,
					tibia_angle);
			break;
		case 4:
			if (coxa_angle < 0)                    // Компенсация крепления ноги
				coxa_angle = coxa_angle + 270.0;
			else
				coxa_angle = coxa_angle - 90.0;
			coxa_angle = constrain(coxa_angle, 0.0, 180.0);
			success &= walker_servo_write(leg_number, coxa_angle, femur_angle,
					tibia_angle);
			break;
		case 5:
			if (flag_IK_control_leg & (1 << 5)) {
				if (coxa_angle < 0)                 // Компенсация крепления ноги
					coxa_angle = coxa_angle + 315.0;
				else
					coxa_angle = coxa_angle - 45.0;
				coxa_angle = constrain(coxa_angle, 0.0, 180.0);
				success &= walker_servo_write(leg_number, coxa_angle,
						femur_angle, tibia_angle);
			}
			break;
		}
	}
	return success;
}

bool walker_init(pca9685_handle_t *pca_1, pca9685_handle_t *pca_2) {
	/*
	 * Назначение: Инициализация драйверов pca9685, и конфигируции ног робота
	 * Входные параметры:
	 * 		pca_1, pca_2: дескрипторы драйверов pca9685
	 * Return:
	 * 		True - инициализация успешно
	 * 		False - иначе
	 */
	// Конфигурация ног робота
	sky_walker[0] = leg1;
	sky_walker[1] = leg2;
	sky_walker[2] = leg3;
	sky_walker[3] = leg4;
	sky_walker[4] = leg5;
	sky_walker[5] = leg6;

	bool success = true;
	float frequence = 1 / PERIOD_MS * 1000;
	// Установка периода ШИМа 3 мс
	success &= pca9685_init(pca_1);
	success &= pca9685_set_pwm_frequency(pca_1, frequence);
	success &= pca9685_init(pca_2);
	success &= pca9685_set_pwm_frequency(pca_2, frequence);

	return success;
}

bool walker_servo_nelrtal() {
	/*
	 * Назначение: Вывод всех сервоприводов в нейтральное положение
	 * Return:
	 * 		True - успешно
	 * 		False - иначе
	 */
	bool success = true;

	for (uint8_t i = 0; i < 6; i++) {
		success &= walker_servo_write(i, 90 + coxa_cal[i], 90 + femur_cal[i],
				90 + tibia_cal[i]);
	}

	return success;
}

void walker_tripod_mode(ps2_handle_t *ps) {
	/*
	 *  Назначение: Реализация режима шага - треугольник ( 3 - идут, 3 - стоят)
	 *  Входные параметры:
	 *  	ps - дескриптор джойстика
	 */
	float amplitudes[3] = { 0, 0, 0 }; // Массив амплитуд шага
	// Приращения по осям x, y, z
	int8_t RX = ps->right_stick.Y;
	int8_t RY = ps->right_stick.X;
	int8_t LX = -ps->left_stick.X;

	// Если процесс уже запущен или стики не в мертвой зоне
	if ((abs(RX) > 15) || (abs(RY) > 15) || (abs(LX) > 15) || (tick > 0)) {
		calc_step_len(RX, RY, LX); 	// Расчет длин шага
		uint16_t numTicks = round(duration / PERIOD_MS / 2.0); // Расчёт периода 1 шага
		float phi = M_PI * tick / numTicks;	// Текущий момент шага
		for (uint8_t leg_num = 0; leg_num < 6; leg_num++) {
			calc_ampl(leg_num, amplitudes);	// Расчёт амплитуд шага ноги
			switch (tripod_mode[leg_num]) {
			case 1:
				sky_walker[leg_num].X = home_x[leg_num]
						- amplitudes[0] * cos(phi);
				sky_walker[leg_num].Y = home_y[leg_num]
						- amplitudes[1] * cos(phi);
				sky_walker[leg_num].Z = home_z[leg_num]
						+ fabs(amplitudes[2]) * sin(phi);
				if (tick >= numTicks - 1)
					tripod_mode[leg_num] = 2; // Смена фазы
				break;
			case 2:
				sky_walker[leg_num].X = home_x[leg_num]
						+ amplitudes[0] * cos(phi);
				sky_walker[leg_num].Y = home_y[leg_num]
						+ amplitudes[1] * cos(phi);
				sky_walker[leg_num].Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					tripod_mode[leg_num] = 1; // Смена фазы
				break;
			}
		}
		// Увеличение tick
		if (tick < numTicks - 1)
			tick++;
		else
			tick = 0;
	}
}

void walker_wave_mode(ps2_handle_t *ps) {
	/*
	 *  Назначение: Реализация режима шага - волна ( 1 - идет, 5 - стоят)
	 *  Входные параметры:
	 *  	ps - дескриптор джойстика
	 */
	float amplitudes[3] = { 0, 0, 0 }; // Массив амплитуд шага
	// Приращения по осям x, y, z
	int8_t RX = ps->right_stick.Y;
	int8_t RY = ps->right_stick.X;
	int8_t LX = -ps->left_stick.X;

	// Если процесс уже запущен или стики не в мертвой зоне
	if ((abs(RX) > 15) || (abs(RY) > 15) || (abs(LX) > 15) || (tick > 0)) {
		calc_step_len(RX, RY, LX); 	// Расчет длин шага
		uint16_t numTicks = round(duration / PERIOD_MS / 6.0); // Расчёт периода 1 шага
		float phi = M_PI * tick / numTicks;	// Текущий момент шага
		for (uint8_t leg_num = 0; leg_num < 6; leg_num++) {
			calc_ampl(leg_num, amplitudes);	// Расчёт амплитуд шага ноги
			switch (wave_mode[leg_num]) {
			case 1:
				sky_walker[leg_num].X = home_x[leg_num]
						- amplitudes[0] * cos(phi);
				sky_walker[leg_num].Y = home_y[leg_num]
						- amplitudes[1] * cos(phi);
				sky_walker[leg_num].Z = home_z[leg_num]
						+ fabs(amplitudes[2]) * sin(phi);
				if (tick >= numTicks - 1)
					wave_mode[leg_num] = 6; // Смена фазы
				break;
			case 2:
				sky_walker[leg_num].X = sky_walker[leg_num].X
						- amplitudes[0] / numTicks / 2.5;
				sky_walker[leg_num].Y = sky_walker[leg_num].Y
						- amplitudes[1] / numTicks / 2.5;
				sky_walker[leg_num].Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					wave_mode[leg_num] = 1; // Смена фазы
				break;
			case 3:
				sky_walker[leg_num].X = sky_walker[leg_num].X
						- amplitudes[0] / numTicks / 2.5;
				sky_walker[leg_num].Y = sky_walker[leg_num].Y
						- amplitudes[1] / numTicks / 2.5;
				sky_walker[leg_num].Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					wave_mode[leg_num] = 2; // Смена фазы
				break;
			case 4:
				sky_walker[leg_num].X = sky_walker[leg_num].X
						- amplitudes[0] / numTicks / 2.5;
				sky_walker[leg_num].Y = sky_walker[leg_num].Y
						- amplitudes[1] / numTicks / 2.5;
				sky_walker[leg_num].Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					wave_mode[leg_num] = 3; // Смена фазы
				break;
			case 5:
				sky_walker[leg_num].X = sky_walker[leg_num].X
						- amplitudes[0] / numTicks / 2.5;
				sky_walker[leg_num].Y = sky_walker[leg_num].Y
						- amplitudes[1] / numTicks / 2.5;
				sky_walker[leg_num].Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					wave_mode[leg_num] = 4; // Смена фазы
				break;
			case 6:
				sky_walker[leg_num].X = sky_walker[leg_num].X
						- amplitudes[0] / numTicks / 2.5;
				sky_walker[leg_num].Y = sky_walker[leg_num].Y
						- amplitudes[1] / numTicks / 2.5;
				sky_walker[leg_num].Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					wave_mode[leg_num] = 5; // Смена фазы
				break;
			}
		}
		// Увеличение tick
		if (tick < numTicks - 1)
			tick++;
		else
			tick = 0;
	}
}

void walker_tetrapod_mode(ps2_handle_t *ps) {
	/*
	 *  Назначение: Реализация режима шага - тетрапод ( 2 - идут, 4 - стоят)
	 *  Входные параметры:
	 *  	ps - дескриптор джойстика
	 */
	float amplitudes[3] = { 0, 0, 0 }; // Массив амплитуд шага
	// Приращения по осям x, y, z
	int8_t RX = ps->right_stick.Y;
	int8_t RY = ps->right_stick.X;
	int8_t LX = -ps->left_stick.X;

	// Если процесс уже запущен или стики не в мертвой зоне
	if ((abs(RX) > 15) || (abs(RY) > 15) || (abs(LX) > 15) || (tick > 0)) {
		calc_step_len(RX, RY, LX); 	// Расчет длин шага
		uint16_t numTicks = round(duration / PERIOD_MS / 3.0); // Расчёт периода 1 шага
		float phi = M_PI * tick / numTicks;	// Текущий момент шага
		for (uint8_t leg_num = 0; leg_num < 6; leg_num++) {
			calc_ampl(leg_num, amplitudes);	// Расчёт амплитуд шага ноги
			switch (tetrapod_mode[leg_num]) {
			case 1:
				sky_walker[leg_num].X = home_x[leg_num]
						- amplitudes[0] * cos(phi);
				sky_walker[leg_num].Y = home_y[leg_num]
						- amplitudes[1] * cos(phi);
				sky_walker[leg_num].Z = home_z[leg_num]
						+ fabs(amplitudes[2]) * sin(phi);
				if (tick >= numTicks - 1)
					tetrapod_mode[leg_num] = 2; // Смена фазы
				break;
			case 2:
				sky_walker[leg_num].X = sky_walker[leg_num].X
						- amplitudes[0] / numTicks;
				sky_walker[leg_num].Y = sky_walker[leg_num].Y
						- amplitudes[1] / numTicks;
				sky_walker[leg_num].Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					tetrapod_mode[leg_num] = 3; // Смена фазы
				break;
			case 3:
				sky_walker[leg_num].X = sky_walker[leg_num].X
						- amplitudes[0] / numTicks;
				sky_walker[leg_num].Y = sky_walker[leg_num].Y
						- amplitudes[1] / numTicks;
				sky_walker[leg_num].Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					tetrapod_mode[leg_num] = 1; // Смена фазы
				break;
			}
		}
		// Увеличение tick
		if (tick < numTicks - 1)
			tick++;
		else
			tick = 0;
	}
}

void walker_ripple_mode(ps2_handle_t *ps) {
	/*
	 *  Назначение: Режим ходьбы насекомое
	 *  Входные параметры:
	 *  	ps - дескриптор джойстика
	 */
	float amplitudes[3] = { 0, 0, 0 }; // Массив амплитуд шага
	// Приращения по осям x, y, z
	int8_t RX = ps->right_stick.Y;
	int8_t RY = ps->right_stick.X;
	int8_t LX = -ps->left_stick.X;

	// Если процесс уже запущен или стики не в мертвой зоне
	if ((abs(RX) > 15) || (abs(RY) > 15) || (abs(LX) > 15) || (tick > 0)) {
		calc_step_len(RX, RY, LX); 	// Расчет длин шага
		uint16_t numTicks = round(duration / PERIOD_MS / 6.0); // Расчёт периода 1 шага
		float phi = M_PI * tick / (numTicks * 2);	// Текущий момент шага
		for (uint8_t leg_num = 0; leg_num < 6; leg_num++) {
			calc_ampl(leg_num, amplitudes);	// Расчёт амплитуд шага ноги
			switch (ripple_mode[leg_num]) { // Выбор фазы шага
			case 1:
				sky_walker[leg_num].X = home_x[leg_num]
						- amplitudes[0] * cos(phi);
				sky_walker[leg_num].Y = home_y[leg_num]
						- amplitudes[1] * cos(phi);
				sky_walker[leg_num].Z = home_z[leg_num]
						+ fabs(amplitudes[2]) * sin(phi);
				if (tick >= numTicks - 1)
					ripple_mode[leg_num] = 2; // Смена фазы
				break;
			case 2:
				sky_walker[leg_num].X = home_x[leg_num]
						- amplitudes[0] * cos(phi);
				sky_walker[leg_num].Y = home_y[leg_num]
						- amplitudes[1] * cos(phi);
				sky_walker[leg_num].Z = home_z[leg_num]
						+ fabs(amplitudes[2]) * sin(phi + M_PI_2);
				if (tick >= numTicks - 1)
					ripple_mode[leg_num] = 3; // Смена фазы
				break;
			case 3:
				sky_walker[leg_num].X = sky_walker[leg_num].X
						- amplitudes[0] / numTicks / 2.0;
				sky_walker[leg_num].Y = sky_walker[leg_num].Y
						- amplitudes[1] / numTicks / 2.0;
				sky_walker[leg_num].Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					ripple_mode[leg_num] = 4; // Смена фазы
				break;
			case 4:
				sky_walker[leg_num].X = sky_walker[leg_num].X
						- amplitudes[0] / numTicks / 2.0;
				sky_walker[leg_num].Y = sky_walker[leg_num].Y
						- amplitudes[1] / numTicks / 2.0;
				sky_walker[leg_num].Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					ripple_mode[leg_num] = 5; // Смена фазы
				break;
			case 5:
				sky_walker[leg_num].X = sky_walker[leg_num].X
						- amplitudes[0] / numTicks / 2.0;
				sky_walker[leg_num].Y = sky_walker[leg_num].Y
						- amplitudes[1] / numTicks / 2.0;
				sky_walker[leg_num].Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					ripple_mode[leg_num] = 6; // Смена фазы
				break;
			case 6:
				sky_walker[leg_num].X = sky_walker[leg_num].X
						- amplitudes[0] / numTicks / 2.0;
				sky_walker[leg_num].Y = sky_walker[leg_num].Y
						- amplitudes[1] / numTicks / 2.0;
				sky_walker[leg_num].Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					ripple_mode[leg_num] = 1; // Смена фазы
				break;
			}
		}
		// Увеличение tick
		if (tick < numTicks - 1)
			tick++;
		else
			tick = 0;
	}
}

void walker_move_body(ps2_handle_t *ps) {
	/*
	 * Назначение: движение корпуса по осям
	 * Входные параметры:
	 *  	ps - дескриптор джойстика
	 */
	float RX = map(-127, 127, (ps->right_stick.Y), 2 * MAX_MOVING_BODY,
			-2 * MAX_MOVING_BODY); // Перемещение по оси X
	float RY = map(-127, 127, (ps->right_stick.X), -2 * MAX_MOVING_BODY,
			2 * MAX_MOVING_BODY); // Перемещение по оси Y
	float RZ = map(-127, 127, (ps->left_stick.Y), 2 * MAX_MOVING_BODY,
			-2 * MAX_MOVING_BODY); // Перемещение по оси Z

	for (uint8_t leg_num = 0; leg_num < 6; leg_num++) {	// Перемещение корпуса
		sky_walker[leg_num].X = home_x[leg_num] + RX;
		sky_walker[leg_num].Y = home_y[leg_num] + RY;
		sky_walker[leg_num].Z = home_z[leg_num] + RZ;
	}

	if (flag & 0x02) // Зафиксировать положение?
			{
		for (uint8_t leg_num = 0; leg_num < 6; leg_num++) { // Фиксация пермещения
			offset_x[leg_num] += RX;
			offset_y[leg_num] += RY;
			offset_z[leg_num] += RZ;

			sky_walker[leg_num].X = home_x[leg_num];
			sky_walker[leg_num].Y = home_y[leg_num];
			sky_walker[leg_num].Z = home_z[leg_num];
		}
		flag &= ~0x02; // Сброс флага
		mode = 0;	   // Сброс режима
	}
}

void walker_rotate_body(ps2_handle_t *ps) {
	/*
	 *  Назначение: Поворот корпуса вокруг осей
	 *  Входные параметры:
	 *  	 ps - дескриптор джойстика
	 */
	float rotOffsetX[6], rotOffsetY[6], rotOffsetZ[6]; // Поворот корпуса вокруг осей

	// Расчёт тригонометрии
	float sinX = sin(
			map(-127, 127, ps->right_stick.X, A12DEG, -A12DEG) / 1000000.0);
	float cosX = cos(
			map(-127, 127, ps->right_stick.X, A12DEG, -A12DEG) / 1000000.0);
	float sinY = sin(
			map(-127, 127, ps->right_stick.Y, A12DEG, -A12DEG) / 1000000.0);
	float cosY = cos(
			map(-127, 127, ps->right_stick.Y, A12DEG, -A12DEG) / 1000000.0);
	float sinZ = sin(
			map(-127, 127, ps->left_stick.X, -A30DEG, A30DEG) / 1000000.0);
	float cosZ = cos(
			map(-127, 127, ps->left_stick.X, -A30DEG, A30DEG) / 1000000.0);

	// Пермешение ро оси Z
	float RZ = ps->left_stick.Y;
	if (RZ <= 0)
		RZ = map(-127, 0, RZ, MAX_MOVING_BODY, 0);
	else
		RZ = map(0, 127, RZ, 0, -2 * MAX_MOVING_BODY);

	for (uint8_t leg_num = 0; leg_num < 6; leg_num++) {
		// Расчёт полных координат
		float totalX = home_x[leg_num] + body_x[leg_num];
		float totalY = home_y[leg_num] + body_y[leg_num];
		float totalZ = home_z[leg_num] + body_z[leg_num];

		// Расчёт поворота корпуса
		rotOffsetX[leg_num] = totalX * cosY * cosZ + totalY * sinX * sinY * cosZ
				+ totalY * cosX * sinZ - totalZ * cosX * sinY * cosZ
				+ totalZ * sinX * sinZ - totalX; // Поворот корпуса вокруг оси X
		rotOffsetY[leg_num] = -totalX * cosY * sinZ
				- totalY * sinX * sinY * sinZ + totalY * cosX * cosZ
				+ totalZ * cosX * sinY * sinZ + totalZ * sinX * cosZ - totalY; // Поворот корпуса вокруг оси Y
		rotOffsetZ[leg_num] = totalX * sinY - totalY * sinX * cosY
				+ totalZ * cosX * cosY - totalZ; // Поворот корпуса вокруг оси Z

		// Поворот корпуса
		sky_walker[leg_num].X = home_x[leg_num] + rotOffsetX[leg_num];
		sky_walker[leg_num].Y = home_y[leg_num] + rotOffsetY[leg_num];
		sky_walker[leg_num].Z = home_z[leg_num] + rotOffsetZ[leg_num] + RZ;
	}
	if (flag & 0x02) // Зафиксировать положение
			{
		for (uint8_t leg_num = 0; leg_num < 6; leg_num++) { // Фиксация пермещения
			offset_x[leg_num] += rotOffsetX[leg_num];
			offset_y[leg_num] += rotOffsetY[leg_num];
			offset_z[leg_num] += rotOffsetZ[leg_num] + RZ;

			sky_walker[leg_num].X = home_x[leg_num];
			sky_walker[leg_num].Y = home_y[leg_num];
			sky_walker[leg_num].Z = home_z[leg_num];
		}
		flag &= ~0x02; // Сброс флага
		mode = 0;	   // Сброс режима
	}
}

bool walker_lift_leg(ps2_handle_t *ps) {
	/*
	 *  Назначение: Подъем передних ног робота
	 *  Входные параметры:
	 *  	 ps - дескриптор джойстика
	 *  return:
	 * 		True - успешно
	 * 		False - иначе
	 */
	bool success = true;

	uint16_t pwm = 0;	// Значение ШИМ на канале
	float temp = 0;		// Значение с джойстика
	float z_left = 1.0;	float z_right = 1.0; // Высота шага

	if (flag_IK_control_leg & (1 << 0)) // Проверка на управление ОК первой ногой
			{
		success &= pca9685_read_channel_pwm(sky_walker[0].pca_handle,
				sky_walker[0].coxa, &pwm); // Чтение ШИМ с канала coxa
		leg1_coxa = map(MIN_SERVO_POS, MAX_SERVO_POS, pwm, 0, 180); // Перевод ШИМ в угол
		success &= pca9685_read_channel_pwm(sky_walker[0].pca_handle,
				sky_walker[0].femur, &pwm); // Чтение ШИМ с канала femur
		leg1_femur = map(MIN_SERVO_POS, MAX_SERVO_POS, pwm, 0, 180); // Перевод ШИМ в угол
		success &= pca9685_read_channel_pwm(sky_walker[0].pca_handle,
				sky_walker[0].tibia, &pwm); // Чтение ШИМ с канала tibia
		leg1_tibia = map(MIN_SERVO_POS, MAX_SERVO_POS, pwm, 0, 180); // Перевод ШИМ в угол

		offset_x[1] += 100; // Перенос второй ноги вперед
		flag_IK_control_leg &= ~(1 << 0); // Запрет на управление ОК первой ногой
	}

	temp = ps->right_stick.X; // Чтение с джойстика оси X
	temp = map(-127, 127, temp, 45, -45); // Перевод значений
	success &= pca9685_set_channel_pwm_times(sky_walker[0].pca_handle,
			sky_walker[0].coxa, 0,
			angle_2_u16(constrain(leg1_coxa + temp, 45, 135))); // Поворот серво coxa

	temp = ps->right_stick.Y; // Чтение с джойстика оси Y
	if (temp > 0) { // Джойстик вверх
		temp = map(0, 127, temp, 0, 24); // Перевод значений
		success &= pca9685_set_channel_pwm_times(sky_walker[0].pca_handle,
				sky_walker[0].femur, 0,
				angle_2_u16(constrain(leg1_femur + temp, 0, 170))); // Поворот серво femur
		success &= pca9685_set_channel_pwm_times(sky_walker[0].pca_handle,
				sky_walker[0].tibia, 0,
				angle_2_u16(constrain(leg1_tibia + 4 * temp, 0, 170))); // Поворот серво tibia
	} else { // Джойстик вниз
		z_right = map(-127, 0, temp, 8, 1); // Изменение высоты шага
	}

	if (flag_IK_control_leg & (1 << 5)) { // Проверка на управление ОК шестой ноги
		success &= pca9685_read_channel_pwm(sky_walker[5].pca_handle,
				sky_walker[5].coxa, &pwm); // Чтение ШИМ с канала coxa
		leg6_coxa = map(MIN_SERVO_POS, MAX_SERVO_POS, pwm, 0, 180); // Перевод ШИМ в угол
		success &= pca9685_read_channel_pwm(sky_walker[5].pca_handle,
				sky_walker[5].femur, &pwm); // Чтение ШИМ с канала femur
		leg6_femur = map(MIN_SERVO_POS, MAX_SERVO_POS, pwm, 0, 180); // Перевод ШИМ в угол
		success &= pca9685_read_channel_pwm(sky_walker[5].pca_handle,
				sky_walker[5].tibia, &pwm); // Чтение ШИМ с канала tibia
		leg6_tibia = map(MIN_SERVO_POS, MAX_SERVO_POS, pwm, 0, 180); // Перевод ШИМ в угол

		offset_x[4] += 100; // Перенос пятой ноги вперед
		flag_IK_control_leg &= ~(1 << 5); // Запрет на управление ОК первой ногой
	}

	temp = ps->left_stick.X; // Чтение с джойстика оси X
	temp = map(-127, 127, temp, 45, -45); // Перевод значений
	success &= pca9685_set_channel_pwm_times(sky_walker[5].pca_handle,
			sky_walker[5].coxa, 0,
			angle_2_u16(constrain(leg6_coxa + temp, 45, 135))); // Поворот серво coxa

	temp = ps->left_stick.Y;  // Чтение с джойстика оси Y
	if (temp > 0) { // Джойстик вверх
		temp = map(0, 127, temp, 0, 24); // Перевод значений
		success &= pca9685_set_channel_pwm_times(sky_walker[5].pca_handle,
				sky_walker[5].femur, 0,
				angle_2_u16(constrain(leg6_femur + temp, 0, 170))); // Поворот серво femur
		success &= pca9685_set_channel_pwm_times(sky_walker[5].pca_handle,
				sky_walker[5].tibia, 0,
				angle_2_u16(constrain(leg6_tibia + 4 * temp, 0, 170))); // Поворот серво tibia
	} else { // Джойстик вниз
		z_left = map(-127, 0, temp, 8, 1); // Изменение высоты шага
	}

	if (z_left > z_right) z_right = z_left; // Выбор макс
	if (flag & 0x02) { // Закрепить высоту шага
		step_hight_coef = 1.0 + ((z_right - 1.0) / 3.0); // Пересчёт высоты шага
		flag &= ~0x02; // Сбром флага
	}

	return success;
}

void walker_read_mode(ps2_handle_t *ps) {
	/*
	 *  Назначение: Выбор режима работы робота
	 *  Входные параметры:
	 *  	ps - дескриптор джойстика
	 */

	if (ps->ID == PS2_RED_MODE) {
		if (PS2_READ_BUTTON(ps->buttons, BUTTON_UP)) {
			mode = 0; // Остновка движения
			gait = 0; // Режим ходьбы "Треугольник"
			HAL_UART_Transmit(&huart2,
					(uint8_t*) "Режим ходьбы \"Треугольник\"\n", 27, 100);
		}
		if (PS2_READ_BUTTON(ps->buttons, BUTTON_DOWN)) {
			mode = 0; // Остановка движения
			gait = 1; // Режим ходьбы "Волна"
			HAL_UART_Transmit(&huart2, (uint8_t*) "Режим ходьбы \"Волна\"\n",
					21, 100);
		}
		if (PS2_READ_BUTTON(ps->buttons, BUTTON_LEFT)) {
			mode = 0; // Остановка движения
			gait = 2; // Режим ходьбы "Тетрапод"
			HAL_UART_Transmit(&huart2, (uint8_t*) "Режим ходьбы \"Тетрапод\"\n",
					24, 100);
		}
		if (PS2_READ_BUTTON(ps->buttons, BUTTON_RIGHT)) {
			mode = 0; // Остановка движения
			gait = 3; // Режим ходьбы "Насекомое"
			HAL_UART_Transmit(&huart2,
					(uint8_t*) "Режим ходьбы \"Насекомое\"\n", 25, 100);
		}

		if (PS2_READ_BUTTON(ps->buttons, BUTTON_SELECT)) { // Режим скорости
			if (flag & 0x01) {
				duration = 314; // Медленный режим
				flag &= ~0x01;  // Сброс флага
				HAL_UART_Transmit(&huart2, (uint8_t*) "Быстрый режим\n", 14,
						100);
			} else {
				duration = 157; // Быстрый режим
				flag |= 0x01;   // Сброс флага
				HAL_UART_Transmit(&huart2, (uint8_t*) "Медленный режим\n", 16,
						100);
			}
		}

		if (PS2_READ_BUTTON(ps->buttons, BUTTON_TRIANGLE)) {
			mode = 1; // Разрешить движение
			HAL_UART_Transmit(&huart2, (uint8_t*) "Старт движения\n", 15, 100);
		}
		if (PS2_READ_BUTTON(ps->buttons, BUTTON_SQUARE)) {
			mode = 2; // Перемещение корпуса
			HAL_UART_Transmit(&huart2, (uint8_t*) "Перемещение корпуса\n", 20,
					100);
		}
		if (PS2_READ_BUTTON(ps->buttons, BUTTON_CIRCLE)) {
			mode = 3; // Поворот корпуса
			HAL_UART_Transmit(&huart2, (uint8_t*) "Поворот корпуса\n", 16, 100);
		}
		if (PS2_READ_BUTTON(ps->buttons, BUTTON_CROSS)) {
			mode = 4; // Подъем передних ног
			HAL_UART_Transmit(&huart2, (uint8_t*) "Подъём передних ног\n", 20,
					100);
		}

		if (PS2_READ_BUTTON(ps->buttons,
				BUTTON_L1) || PS2_READ_BUTTON(ps->buttons, BUTTON_R1)) {
			flag |= 0x02; // Зафиксировать перемещение
		}
		if (PS2_READ_BUTTON(ps->buttons,
				BUTTON_L2) || PS2_READ_BUTTON(ps->buttons, BUTTON_R2)) {
			flag &= ~0x02; // Сброс перемщения корпуса
			for (uint8_t leg_num = 0; leg_num < 6; leg_num++) {
				offset_x[leg_num] = 0;
				offset_y[leg_num] = 0;
				offset_z[leg_num] = 0;
			}
			flag_IK_control_leg = (1 << 0) | (1 << 5); // Резрешить управление передними ногами
		}
		if (PS2_READ_BUTTON(ps->buttons, BUTTON_START)) { // Сброс всех настроек
			for (uint8_t leg_num = 0; leg_num < 6; leg_num++) {
				sky_walker[leg_num].X = home_x[leg_num];
				sky_walker[leg_num].Y = home_y[leg_num];
				sky_walker[leg_num].Z = home_z[leg_num];

				offset_x[leg_num] = 0;
				offset_y[leg_num] = 0;
				offset_z[leg_num] = 0;
			}
			flag_IK_control_leg = (1 << 0) | (1 << 5); // Разрешить управление передними ногами
			mode = 0; 				 // Остановка движения
			step_hight_coef = 1.0;	 // Сброс высоты шага
			flag &= ~0x02;			 // Сброс флага
		}
	}

	if (PS2_READ_BUTTON(ps->buttons, BUTTON_START) && ps->ID == PS2_GREEN_MODE) {
		mode = 99; // Режим установки серво в нейтральное положение
	}
}
