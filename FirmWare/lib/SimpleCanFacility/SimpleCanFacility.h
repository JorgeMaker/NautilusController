#ifndef __CAN__H
#define __CAN__H

#include "Arduino.h"
#include "stm32f4xx_hal_can.h"

/* Value needed for prescaler. depends 
 * on CLK configuration and if you change
 * CLK you neet to update this
*/
enum CanSpeed{
    Kbit1000 =  3,
	Kbit500 =   6,
	Kbit250 =   12,
	Kbit125 =  24,
	Kbit100 =  30,
  	Kbit50 =   60,
  	Kbit20  =  150
};

enum CanMode{
	NormalCAN = CAN_MODE_NORMAL,
	SilentCAN = CAN_MODE_SILENT,
	LoopBackCan = CAN_MODE_LOOPBACK,
	SilentLoopBackCAN = CAN_MODE_SILENT_LOOPBACK
};

typedef struct{
  uint8_t dlc;
  uint32_t msgID;
  bool isRTR;
  bool isStandard;
  uint8_t data[8];
}CanMessage;

CanMessage createMessage(void);
/**
 CAN wrapper for STM32F405RGT6 board.
*/
class SimpleCanFacility
{
public:

	SimpleCanFacility();
	HAL_StatusTypeDef init(CanSpeed speed, CanMode mode);
	HAL_StatusTypeDef configFilter(CAN_FilterTypeDef *filterDef);
    HAL_StatusTypeDef configSnniferFilter();
	HAL_StatusTypeDef activateNotification();
	HAL_StatusTypeDef deactivateNotification();
	HAL_StatusTypeDef begin();
	HAL_StatusTypeDef stop();
	HAL_StatusTypeDef send(CanMessage message);
	uint32_t getNumberOfPendingMsg();
	uint32_t receiveCanMessages(CanMessage * pMsgBuff);
  	CAN_HandleTypeDef hcan;
};
#endif