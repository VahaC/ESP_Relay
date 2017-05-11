/*

This is a library for the BH1750FVI Digital Light Sensor
breakout board.

The board uses I2C for communication. 2 pins are required to
interface to the device.


Written by Christopher Laws, March, 2013.
Modified by Vasyl Chepil (VahaC), May, 2017.

*/

#include <Wire.h>
#include "BH1750.h"
//#include <util/delay.h>


BH1750::BH1750() {
}

bool BH1750::begin(uint8_t mode) {

  Wire.begin();
  //write8(mode);
  return configure(mode);
}


bool BH1750::configure(uint8_t mode) {
	uint32_t startWaitTime;
    switch (mode) {
        case BH1750_CONTINUOUS_HIGH_RES_MODE:
        case BH1750_CONTINUOUS_HIGH_RES_MODE_2:
        case BH1750_CONTINUOUS_LOW_RES_MODE:
        case BH1750_ONE_TIME_HIGH_RES_MODE:
        case BH1750_ONE_TIME_HIGH_RES_MODE_2:
        case BH1750_ONE_TIME_LOW_RES_MODE:
            // apply a valid mode change
            Wire.beginTransmission(BH1750_I2CADDR);
            Wire.write(mode);
            Wire.endTransmission();
      			startWaitTime = millis();
      			while (startWaitTime + 10 > millis()) {
      				delay(1);
      			}
            return(true);
            break;
        default:
            // Invalid measurement mode
            #if BH1750_DEBUG == 1
            Serial.println("Invalid measurement mode");
            #endif
            return(false);
            break;
    }
}


uint16_t BH1750::readLightLevel(void) {

  uint16_t level;

  Wire.beginTransmission(BH1750_I2CADDR);
  Wire.requestFrom(BH1750_I2CADDR, 2);

  level = Wire.read();
  level <<= 8;
  level |= Wire.read();
  Wire.endTransmission();

#if BH1750_DEBUG == 1
  Serial.print("Raw light level: ");
  Serial.println(level);
#endif

  level = level/1.2; // convert to lux

#if BH1750_DEBUG == 1
  Serial.print("Light level: ");
  Serial.println(level);
#endif
  return level;
}

