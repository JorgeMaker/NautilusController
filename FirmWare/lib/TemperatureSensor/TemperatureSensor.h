#ifndef __TEMP_SENSOR__H
#define __TEMP_SENSOR__H

#include "Arduino.h"
#include "stm32f4xx_hal_i2c.h"

class TemperatureSensor
{
public:
	TemperatureSensor();
	HAL_StatusTypeDef init(void);
	float readTemperatureC(void);
  	I2C_HandleTypeDef hi2c;
};

#endif
