#ifndef __ESPWEBCFG_H
#define __ESPWEBCFG_H

#include <Arduino.h>

//#define NOSERIAL // Раскомментируйте это макроопределение, чтобы не использовать отладочный вывод в Serial (можно будет использовать пины RX и TX после загрузки скетча для полезной нагрузки)
//#define NOBLED // Раскомментируйте это макроопределение, чтобы не использовать мигание встроенного светодиода (можно будет использовать пин LED_BUILTIN для полезной нагрузки)
//#define NOEEPROMERASE // Раскомментируйте это макроопределение, чтобы не использовать возможность стирания EEPROM замыканием A0 на VCC при старте

#ifndef NOEEPROMERASE
const uint16_t eepromEraseLevel = 900;
const uint32_t eepromEraseTime = 2000;
#endif

//#define USEDS3231 // Закомментируйте это макроопределение, если вы не планируете использовать часы реального времени DS3231
//#define USEAT24C32 // Закомментируйте это макроопределение, если вы не планируете использовать I2C EEPROM

#if defined(USEDS3231) || defined(USEAT24C32) || defined(USEBME) || defined(USEBH) // Определите пины, на которые подключены I2C устройства
const int8_t pinSDA = SDA; // I2C SDA pin
const int8_t pinSCL = SCL; // I2C SCL pin
const bool fastI2C = true; // I2C Fast mode (400 kHz)
#endif

#endif
