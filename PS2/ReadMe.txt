1.Используется библиотека HAL и протокол SPI1

2.Настройки SPI1:
    Mode: Full-Duplex Master
    Hardware NSS Signal: Disable
  Basic Parameters:
    Frame Format: Motorola
    Data Size: 8 Bits
    First Bit: LSB First
  Clock Parameters:
    Prescaler: 32
    Baud Rate: 250.0 KBits/s
    Clock Polarity: High
    Clock Phase: 2 Edge
  Advanced Parameters:
    CRC Calculation: Disabled
    NSS Signal Type: Software

3.Необходимо пину SS задать имя PS2_CS

4.Для работы необходимо определить переменную типа ps2_handle_t и задать поле .spi_handle.
 Данные с джойстика будут в полях:
              .ID - режим работы
              .buttons - нажатые кнопки (1 - кнопка нажата; 0 - кнопка не нажата)
              .right_stick: .X и .Y - Значения по осям X и Y соответсвенно для правого стика
              .lefr_stick: .X и .Y - Значения по осям X и Y соответсвенно для левого стика


5.Функция bool PS2_ReadData(ps2_handle_t *handle) служит для получения данных с джойстика.
 Входные данные:
 			handle - Дескриптор джойстика PS2
 return:
 			True - успешно, False - иначе

4.Литература:
    Даташит: Documents/Ps2_manual
    https://store.curiousinventor.com/guides/PS2
    https://gamesx.com/controldata/psxcont/psxcont.htm