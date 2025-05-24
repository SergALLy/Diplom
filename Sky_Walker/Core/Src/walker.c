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

#define MAX_SERVO_POS		2958 						// Зачение ШИМа в положении серва 225deg
#define MIN_SERVO_POS		1138  						// Зачение ШИМа в положении серва 45deg

#define RAD_TO_DEG	(180.0 / M_PI)						// Перевод из радиан в градусы
#define DEG_TO_RAD	(M_PI / 180.0)						// Перевод из градусов в радианы

static uint16_t tick = 0;								// Вспомогательная, для плавного шага
static uint16_t duration = 314;								// Продолжительность шага

static float step_len_X, step_len_Y;					// Вспомогательные, длина шага
static float sin_rot_Z, cos_rot_Z;						// Вспомогательные, поворот вокруг оси z

static uint8_t tripod_mode[6] = {1, 2, 1, 2, 1, 2};		// Порядок шага для режима "треугольник"
static uint8_t wave_mode[6] = {1, 2, 3, 4, 5, 6};		// Порядок шага для режима "волна"

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
	if (value == 0)
		return (max_2 - min_2) / 2 + min_2;
	if (value >= max_1)
		return max_2;
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
	assert(angle >= 0);
	assert(angle <= 180);

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
		amp[2] = (step_len_X + rot_offset_x) / 4.0;
	else
		amp[2] = (step_len_Y + rot_offset_y) / 4.0;
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

	success &= pca9685_set_channel_pwm_times(sky_walker[leg_num].pca_handle, sky_walker[leg_num].coxa, 0,
			angle_2_u16(coxa_angle)); // поворот серво coxa
	success &= pca9685_set_channel_pwm_times(sky_walker[leg_num].pca_handle, sky_walker[leg_num].femur, 0,
			angle_2_u16(femur_angle)); // поворот серво femur
	success &= pca9685_set_channel_pwm_times(sky_walker[leg_num].pca_handle, sky_walker[leg_num].tibia, 0,
			angle_2_u16(tibia_angle)); // поворот серво tibia
	return success;
}

bool walker_calc_ik(uint8_t leg_number, float x,
		float y, float z) {
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
		// расчёт угла для серво coxa
		coxa_angle = atan2(x, y) * RAD_TO_DEG + coxa_cal[leg_number];
		// расчёт угла для серво femur
		gamma_femur = atan2(z, L0);
		phi_femur = acos(
				(pow(FEMUR_LENGTH, 2) + pow(L3, 2) - pow(TIBIA_LENGTH, 2))
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
		// Перенос ноги в положение (x,y,z)
		switch (leg_number) {
		case 0:
			coxa_angle = coxa_angle + 45.0;        //компенсация крепления ноги
			coxa_angle = constrain(coxa_angle, 0.0, 180.0);
			success &= walker_servo_write(leg_number, coxa_angle,
					femur_angle, tibia_angle);
			break;
		case 1:
			coxa_angle = coxa_angle + 90.0;        //компенсация крепления ноги
			coxa_angle = constrain(coxa_angle, 0.0, 180.0);
			success &= walker_servo_write(leg_number, coxa_angle,
					femur_angle, tibia_angle);
			break;
		case 2:
			coxa_angle = coxa_angle + 135.0;       //компенсация крепления ноги
			coxa_angle = constrain(coxa_angle, 0.0, 180.0);
			success &= walker_servo_write(leg_number, coxa_angle,
					femur_angle, tibia_angle);
			break;
		case 3:
			if (coxa_angle < 0)                    //компенсация крепления ноги
				coxa_angle = coxa_angle + 225.0;
			else
				coxa_angle = coxa_angle - 135.0;
			coxa_angle = constrain(coxa_angle, 0.0, 180.0);
			success &= walker_servo_write(leg_number, coxa_angle,
					femur_angle, tibia_angle);
			break;
		case 4:
			if (coxa_angle < 0)                    //компенсация крепления ноги
				coxa_angle = coxa_angle + 270.0;
			else
				coxa_angle = coxa_angle - 90.0;
			coxa_angle = constrain(coxa_angle, 0.0, 180.0);
			success &= walker_servo_write(leg_number, coxa_angle,
					femur_angle, tibia_angle);
			break;
		case 5:
			if (coxa_angle < 0)                    //компенсация крепления ноги
				coxa_angle = coxa_angle + 315.0;
			else
				coxa_angle = coxa_angle - 45.0;
			coxa_angle = constrain(coxa_angle, 0.0, 180.0);
			success &= walker_servo_write(leg_number, coxa_angle,
					femur_angle, tibia_angle);
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
	sky_walker[0] = leg1;	sky_walker[1] = leg2;
	sky_walker[2] = leg3;	sky_walker[3] = leg4;
	sky_walker[4] = leg5;	sky_walker[5] = leg6;

	bool success = true;
	float frequence = 1/PERIOD_MS * 1000;
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
		success &= walker_servo_write(i, 90 + coxa_cal[i],
				90 + femur_cal[i], 90 + tibia_cal[i]);
	}
	return success;
}

void walker_tripod_mode(ps2_handle_t *ps) {
	/*
	 *  Назначение: Реализация режима шага - треугольник ( 3 - идут, 3 - стоят)
	 *  Входные параметры:
	 *  	ps - дескриптор джойстика
	 */
	float amplitudes[3] = { 0, 0, 0 }; // массив амплитуд шага
	// Приращения по осям x, y, z
	int8_t RX = ps->right_stick.Y;
	int8_t RY = ps->right_stick.X;
	int8_t LX = ps->left_stick.X;

	// Если процесс уже запущен или стики не в мертвой зоне
	if ((abs(RX) > 15) || (abs(RY) > 15) || (abs(LX) > 15) || (tick > 0)) {
		calc_step_len(RX, RY, LX); 	// Расчет длин шага
		uint16_t numTicks = round(duration / PERIOD_MS / 2.0); // расчёт периода 1 шага
		float phi = M_PI * tick / numTicks;	// текущая фаза шага
		for (uint8_t leg_num = 0; leg_num < 6; leg_num++) {
			calc_ampl(leg_num, amplitudes);	// расчёт амплитуд шага ноги
			switch (tripod_mode[leg_num]) {
			case 1:                        //подъем и перемещение ноги
				sky_walker[leg_num].X = home_x[leg_num] - amplitudes[0] * cos(phi);
				sky_walker[leg_num].Y = home_y[leg_num] - amplitudes[1] * cos(phi);
				sky_walker[leg_num].Z = home_z[leg_num]
						+ fabs(amplitudes[2]) * sin(phi);
				if (tick >= numTicks - 1)
					tripod_mode[leg_num] = 2; // смена режима
				break;
			case 2:                             // передвижение без подъема
				sky_walker[leg_num].X = home_x[leg_num] + amplitudes[0] * cos(phi);
				sky_walker[leg_num].Y = home_y[leg_num] + amplitudes[1] * cos(phi);
				sky_walker[leg_num].Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					tripod_mode[leg_num] = 1; // смена режима
				break;
			}
		}
		// увеличение tick
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
	float amplitudes[3] = { 0, 0, 0 }; // массив амплитуд шага
	// Приращения по осям x, y, z
	int8_t RX = ps->right_stick.Y;
	int8_t RY = ps->right_stick.X;
	int8_t LX = ps->left_stick.X;

	// Если процесс уже запущен или стики не в мертвой зоне
	if ((abs(RX) > 15) || (abs(RY) > 15) || (abs(LX) > 15) || (tick > 0)) {
		calc_step_len(RX, RY, LX); 	// Расчет длин шага
		uint16_t numTicks = round(duration / PERIOD_MS / 6.0); // расчёт периода 1 шага
		float phi = M_PI * tick / numTicks;	// текущая фаза шага
		for (uint8_t leg_num = 0; leg_num < 6; leg_num++) {
			calc_ampl(leg_num, amplitudes);	// расчёт амплитуд шага ноги
			switch (wave_mode[leg_num]) {
			case 1:                        //подъем и перемещение ноги
				sky_walker[leg_num].X = home_x[leg_num] - amplitudes[0] * cos(phi);
				sky_walker[leg_num].Y = home_y[leg_num] - amplitudes[1] * cos(phi);
				sky_walker[leg_num].Z = home_z[leg_num] + fabs(amplitudes[2]) * sin(phi);
				if (tick >= numTicks - 1)
					wave_mode[leg_num] = 6; // смена режима
				break;
			case 2:                             // передвижение без подъема
				sky_walker[leg_num].X = sky_walker[leg_num].X - amplitudes[0] / numTicks / 2.5;
				sky_walker[leg_num].Y = sky_walker[leg_num].Y - amplitudes[1] / numTicks / 2.5;
				sky_walker[leg_num].Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					wave_mode[leg_num] = 1; // смена режима
				break;
			case 3:                             // передвижение без подъема
				sky_walker[leg_num].X = sky_walker[leg_num].X - amplitudes[0] / numTicks / 2.5;
				sky_walker[leg_num].Y = sky_walker[leg_num].Y - amplitudes[1] / numTicks / 2.5;
				sky_walker[leg_num].Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					wave_mode[leg_num] = 2; // смена режима
				break;
			case 4:                             // передвижение без подъема
				sky_walker[leg_num].X = sky_walker[leg_num].X - amplitudes[0] / numTicks / 2.5;
				sky_walker[leg_num].Y = sky_walker[leg_num].Y - amplitudes[1] / numTicks / 2.5;
				sky_walker[leg_num].Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					wave_mode[leg_num] = 3; // смена режима
				break;
			case 5:                             // передвижение без подъема
				sky_walker[leg_num].X = sky_walker[leg_num].X - amplitudes[0] / numTicks / 2.5;
				sky_walker[leg_num].Y = sky_walker[leg_num].Y - amplitudes[1] / numTicks / 2.5;
				sky_walker[leg_num].Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					wave_mode[leg_num] = 4; // смена режима
				break;
			case 6:                             // передвижение без подъема
				sky_walker[leg_num].X = sky_walker[leg_num].X - amplitudes[0] / numTicks / 2.5;
				sky_walker[leg_num].Y = sky_walker[leg_num].Y - amplitudes[1] / numTicks / 2.5;
				sky_walker[leg_num].Z = home_z[leg_num];
				if (tick >= numTicks - 1)
					wave_mode[leg_num] = 5; // смена режима
				break;
			}
		}
		// увеличение tick
		if (tick < numTicks - 1)
			tick++;
		else
			tick = 0;
	}
}

bool walker_read_mode(ps2_handle_t *ps) {
	/*
	 *  Назначение: Выбор режима работы робота
	 *  Входные параметры:
	 *  	ps - дескриптор джойстика
	 * Return:
	 * 		True - успешно чтение данных с джойстика
	 * 		False - ошибка при чтении данных с джойстика
	 */
	bool success = true;
	 uint8_t flag = 0;

	success &= PS2_ReadData(ps); // Чтение данных с джойстика
	if (ps->ID == PS2_RED_MODE) {
		if (PS2_READ_BUTTON(ps->buttons, BUTTON_UP)) {
			mode = 0; // Остновка движения
			gait = 0; // Смена редима ходьбы
		}
		if (PS2_READ_BUTTON(ps->buttons, BUTTON_DOWN)) {
			mode = 0; // Остановка движения
			gait = 1; // Смена режима ходьба
		}
		if (PS2_READ_BUTTON(ps->buttons, BUTTON_TRIANGLE)) {
			mode = 1; // Подтвердить выбор режима и разрешить движение
		}
		if (PS2_READ_BUTTON(ps->buttons, BUTTON_SELECT)){
			if (flag == 0) {duration = 314; flag = 1;}
			else {duration = 157; flag = 0;}
		}
		//if (PS2_READ_BUTTON(ps->buttons, BUTTON_L1)) duration = 157;
	}

	if (PS2_READ_BUTTON(ps->buttons, BUTTON_START) && ps->ID == PS2_GREEN_MODE) {
		mode = 99; // Режим установки серво в нейтральное положение
	}

	return success;
}
