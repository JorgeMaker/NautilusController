/*
#include <Arduino.h>
#include <SimpleCanFacility.h>
#include <NautilusCANMessage.h>
#include "STM32F405RTGBorad.h"

#define BLUE_LED PA8 // Blue

#define NUM_OF_BLINKS 15
void bootLedIndicator()
{
  int counter = NUM_OF_BLINKS;
  pinMode(BLUE_LED, OUTPUT);
  while (counter > 0)
  {
    digitalToggle(BLUE_LED);
    delay(50);
    counter--;
  }
}

SimpleCanFacility can;
NautilusCANMesage nautilusMsg;
CanMessage rxMsgBuff[100];
double lastTime;

void setup()
{
  bootLedIndicator();
  Serial.begin(115200);

  can.init(Kbit250, NormalCAN);
  can.configSnniferFilter();
  can.activateNotification();
  can.begin();
  bootLedIndicator();
  lastTime = millis();

  // put your setup code here, to run once:
}

void loop()
{
  //Serial.println("Hola CAN");

  nautilusMsg.setEncodeMessageID(TARGET_CMD, FLOAT, 1, 0, 5);
  nautilusMsg.setFloatPayload(50.45);
  can.send(nautilusMsg.toCanMessage());
  digitalToggle(BLUE_LED);
  delay(5);
  

   Serial.println("Hola CAN");
   digitalToggle(BLUE_LED);


  int receivedMessages = can.receiveCanMessages(rxMsgBuff);
  if (receivedMessages > 0){
    digitalToggle(BLUE_LED);
    //Serial.println("------ Received : " + String(receivedMessages) + "  ------");
    for (int index = 0; index < receivedMessages; index++){

      NautilusCANMesage nautilusMsg = rxMsgBuff[index];
      Serial.println("Node ID:    " + String(nautilusMsg.getNodeID()));
      Serial.println("Multicast:  " + String(nautilusMsg.getMmulticast()));
      Serial.println("Entity:     " + String(nautilusMsg.getEntity()));
      Serial.println("Data Type:  " + String(nautilusMsg.getDdataType()));
      Serial.println("Command ID: " + String(nautilusMsg.getCommandID()));
      switch (nautilusMsg.getDdataType()){
        case FLOAT:
          Serial.println("Payload:  " + String(nautilusMsg.getFloatPayload()));
          break;
        case INT:
           Serial.println("Payload: " + String(nautilusMsg.getIntPayload()));
          break;
        case BOOLEAN:
          Serial.println("Payload:  " + String(nautilusMsg.getBoolPayLoad()));
          break;
        case CHAR:
           Serial.println("Payload: " + String(nautilusMsg.getCharPayload()));
          break;
        default:
          break;
      }
      Serial.println("-----------------------");
     //Serial.println("Received msg with id: " + String(rxMsgBuff[index].msgID));
    }
  }
}
*/
