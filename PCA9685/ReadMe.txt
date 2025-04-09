Библиотека: https://github.com/henriheimann/stm32-hal-pca9685

1.Используется библиотека HAL и протокол I2C

2.Настройки I2C:
	I2C Speed Mode: Standart Mode
	I2C Speed Frequency: 100KHz
	Primary Address Lenght selection: 7-bit

3.Подключение:
	SCL -> SCL
	SDA -> SDA

4.Для работы необходимо определить переменную типа pca9685_handle_t и задать поля:
	.i2c_handle - дескритор протокола I2C устройства;
	.device_address - адрес устройсва;
	.inverted - необходимость инвертирования (true/false)

5.Функция bool pca9685_init(pca9685_handle_t *handle) служит для инициализации устройства.
 Входные параметры:
	handle - дескриптор устройства
 return:
	True - успешно, False - иначе
6.Функция bool pca9685_set_pwm_frequency(pca9685_handle_t *handle, float frequency) служит для установки частоты ШИМа.
 Входные параметры:
	handle - дескриптор устройства
	frequency - частота ШИМа
 return:
	True - успешно, False - иначе
7.Функция bool pca9685_set_channel_pwm_times(pca9685_handle_t *handle, unsigned channel, unsigned on_time, unsigned off_time) служит для установки 1 и 0 ШИМа.
 Входные параметры:
	handle - дескриптор устройства
	channel - канал выхода
	on_time - начало 1
	off_time - начало 0
 return:
	True - успешно, False - иначе 
8.Функция bool pca9685_set_channel_duty_cycle(pca9685_handle_t *handle, unsigned channel, float duty_cycle, bool logarithmic) служит для установки скважности ШИМа.
 Входные параметры:
	handle - дескриптор устройства
	channel - канад выхода
	duty_cycle - скважность
	logarithmic - true для применения логарифмической функции
 return:
	True - успешно, False - иначе
9.Литература:
	Даташит: Documents/PCA9685.pdf
	Даташит: Documents/PCA9685_ru.pdf