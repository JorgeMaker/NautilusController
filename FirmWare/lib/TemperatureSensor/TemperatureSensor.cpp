
#include <TemperatureSensor.h>
#include <Arduino.h>

extern "C" HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *);
extern "C" HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef*, uint16_t,uint16_t,uint16_t,uint8_t*,uint16_t,uint32_t);

//extern "C" HAL_StatusTypeDef HAL_I2C_Start(CAN_HandleTypeDef *);

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
    
    /* Peripheral clock enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }

}

TemperatureSensor::TemperatureSensor(){
    return;
}

HAL_StatusTypeDef TemperatureSensor::init(){

  hi2c.Instance = I2C1;
  hi2c.Init.ClockSpeed = 100000;
  hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c.Init.OwnAddress1 = 0;
  hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c.Init.OwnAddress2 = 0;
  hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_MspInit(&hi2c);

  return HAL_I2C_Init(&hi2c);
}

float TemperatureSensor::readTemperatureC(void){

 	float temperatureValue = 0;
 	uint8_t address = 0B10010000;
 	uint8_t regValue= 0B00000000;
 	uint8_t rxBuff[2] = {0};

 	HAL_I2C_Mem_Read(&hi2c, address, regValue, 1, rxBuff, 2, 50);
 	uint16_t temperatureReading = (rxBuff[0] << 8) | rxBuff[1];

 	if (temperatureReading & (1<<15)){
 		// negative number
 		temperatureReading = temperatureReading >>7;
 		temperatureValue = 0.5 * (temperatureReading & 1);
 		temperatureReading = temperatureReading >>1;
 		temperatureReading = ~temperatureReading;
 		temperatureValue = (temperatureValue + temperatureReading)*-1;
 		return temperatureValue;

 	}
 	else {
 		// positive number
 		temperatureReading = temperatureReading >>7;
 		temperatureValue = 0.5 * (temperatureReading & 1);
 		temperatureReading = temperatureReading >>1;
 		temperatureValue = temperatureValue + temperatureReading;

 		return temperatureValue;
 	}
}