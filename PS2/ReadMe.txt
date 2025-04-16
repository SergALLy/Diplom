1.Используется библиотека HAL и протокол SPI

2.Настройки SPI:
	Mode: Full-Duplex Master
	Data Size: 8 Bits
	First Bit: LSB First
	Baud Rate: 250.0 KBits/s
	Clock Polarity: High
	Clock Phase: 2 Edge

3.Подключение:
	SCK  -> CLK
	MOSI -> COM
	MISO -> DAT
	SS   -> ATT
	
4.Необходимо пину SS задать имя PS2_CS

5.Для работы необходимо определить переменную типа ps2_handle_t и задать поле:
	.spi_handle - дескриптор протокола SPI, подключенного устройства. 
 Данные с джойстика находятся в полях:
              .ID - режим работы
              .buttons - нажатые кнопки (1 - кнопка нажата; 0 - кнопка не нажата)
              .right_stick: .X и .Y - значения по осям X и Y соответсвенно для правого стика 
              .left_stick: .X и .Y - значения по осям X и Y соответсвенно для левого стика
	(для X: -128 - крайнее левое, 0 - нейтральное,  127 - крайнее правое)
	(для Y: -128 - крайнее вверхнее, 0 - нейтральное, 127 - крайнее нижнее)

6.Функция bool PS2_ReadData(ps2_handle_t *handle) служит для получения данных с джойстика.
 Входные параметры:
 	handle - дескриптор джойстика PS2
 return:
 	True - успешно, False - иначе

7.Литература:
	Даташит: Documents/PS2_manual.pdf
	https://store.curiousinventor.com/guides/PS2
	https://gamesx.com/controldata/psxcont/psxcont.htm