#include <SimpleCanFacility.h>
#include <Arduino.h>

extern "C" HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *);
extern "C" HAL_StatusTypeDef HAL_CAN_Stop (CAN_HandleTypeDef *);
extern "C" HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *);
extern "C" HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, uint32_t );
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *);
extern "C" uint32_t HAL_CAN_GetRxFifoFillLevel (CAN_HandleTypeDef *hcan,uint32_t RxFifo);	

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX  
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
   
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }

}

CanMessage createMessage(){
  CanMessage message;
  message.dlc = 8;
  message.msgID = 200;
  message.isRTR = false;
  message.isStandard = true;
  //uint8_t messageLoadBuffer[8] ={0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x23};
  uint8_t messageLoadBuffer[8] ={'N','A','U','T','I','L','U','S'};
  memcpy(message.data, messageLoadBuffer, 8);
  
  return message;
}

SimpleCanFacility::SimpleCanFacility(){
    return;
}

HAL_StatusTypeDef SimpleCanFacility::init(CanSpeed speed, CanMode mode){

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = speed;
  hcan.Init.Mode = mode;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;

   HAL_CAN_MspInit(&hcan);

  return HAL_CAN_Init(&hcan);

}
HAL_StatusTypeDef SimpleCanFacility::begin(){
    return HAL_CAN_Start(&hcan);

}
HAL_StatusTypeDef SimpleCanFacility::stop(){
   	return HAL_CAN_Stop(&hcan);

}
HAL_StatusTypeDef SimpleCanFacility::configFilter(CAN_FilterTypeDef *filterDef){

    return HAL_CAN_ConfigFilter(&hcan, filterDef);
}
HAL_StatusTypeDef SimpleCanFacility::configSnniferFilter(){

    // Default filter - accept all to CAN_FIFO*
	  CAN_FilterTypeDef sFilterConfig;
	  sFilterConfig.FilterBank = 0;
	  sFilterConfig.FilterIdHigh = 0x00000;
	  sFilterConfig.FilterBank = 0x0000;
	  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	  sFilterConfig.FilterIdHigh = 0x0000 << 5;  //11-bit ID, in top bits is used s
	  sFilterConfig.FilterIdLow  = 0x0000;
	  sFilterConfig.FilterMaskIdHigh = 0x0000;
	  sFilterConfig.FilterMaskIdLow = 0x0000;
	  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	  sFilterConfig.FilterActivation = ENABLE;

    return HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
}
HAL_StatusTypeDef SimpleCanFacility::send(CanMessage message){
	
    uint32_t TxMailbox;
	CAN_TxHeaderTypeDef pHeader;
    pHeader.DLC= message.dlc;
    message.isStandard ? pHeader.IDE=CAN_ID_STD : pHeader.IDE=CAN_ID_EXT;
    message.isRTR ?  pHeader.RTR=CAN_RTR_REMOTE : pHeader.RTR=CAN_RTR_DATA;
    message.isStandard ? pHeader.StdId= message.msgID : pHeader.ExtId= message.msgID;

    return HAL_CAN_AddTxMessage(&hcan, &pHeader, (uint8_t*)message.data, &TxMailbox);
}

uint32_t SimpleCanFacility::getNumberOfPendingMsg(){
        return HAL_CAN_GetRxFifoFillLevel(&hcan,CAN_RX_FIFO0);
}

uint32_t SimpleCanFacility::receiveCanMessages(CanMessage * pMsgBuff){
    uint32_t pendingMsgs = HAL_CAN_GetRxFifoFillLevel(&hcan,CAN_RX_FIFO0);
    CAN_RxHeaderTypeDef pHeader;
    uint8_t data[8];
    uint32_t cursor = 0;
    while (cursor< pendingMsgs){
        
        HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&pHeader,data);

        pMsgBuff[cursor].dlc = pHeader.DLC;
        (pHeader.IDE == CAN_ID_STD) ? pMsgBuff[cursor].isStandard =true :pMsgBuff[cursor].isStandard =false;
        (pHeader.RTR == CAN_RTR_REMOTE) ? pMsgBuff[cursor].isRTR =true :pMsgBuff[cursor].isRTR =false;
        (pHeader.IDE == CAN_ID_STD) ? pMsgBuff[cursor].msgID =pHeader.StdId : pMsgBuff[cursor].msgID = pHeader.ExtId;
         memcpy(pMsgBuff[cursor].data, data, pMsgBuff[cursor].dlc); 

        cursor++;
    }
    return pendingMsgs;
}
HAL_StatusTypeDef SimpleCanFacility::activateNotification( )
{
    return HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}
HAL_StatusTypeDef SimpleCanFacility::deactivateNotification(){
   // _rxHandler = NULL;
    return HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}
