/*
#include <SimpleFOC.h>
#include "STM32F405RTGBorad.h"

///    SPI 2 (AS5048A)
#define MOSI_DRV PA7 // Blue
#define MISO_DRV PA6 // Green
#define CLK_DRV PA5  // Orange
#define CS_DRV PA4   // Yellow

///     SPI 1  (DRV8305)
#define MOSI_ENC PB15 // Blue
#define MISO_ENC PB14 // Green
#define CLK_ENC PB13  // Orange
#define CS_ENC PB12   // Yellow

#define INH_A PC7 // TIM8_CH2
#define INH_B PA1 // TIM2_CH2
#define INH_C PA3 // TIM2_CH4

#define INL_A PC6 // TIM8-CH1
#define INL_B PA0 // TIM2-CH1
#define INL_C PA2 // TIM2-CH3

#define ENA_GATE PC3
#define nMFAULT PC4  

#define BLUE_LED PB2
#define LOOP_PIN PC13

#define BLUE_LED PB2
#define LOOP_PIN PC13

#define SENSE_A PB1
#define SENSE_B PB0
#define SENSE_C PC5


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
double lastTime;

void setOutPutMode(){

  pinMode(MOSI_DRV, OUTPUT);
  pinMode(MISO_DRV, OUTPUT);
  pinMode(CLK_DRV, OUTPUT);
  pinMode(CS_DRV, OUTPUT);

  pinMode(MOSI_ENC, OUTPUT);
  pinMode(MISO_ENC, OUTPUT);
  pinMode(CLK_ENC, OUTPUT);
  pinMode(CS_ENC, OUTPUT);

  pinMode(INH_A, OUTPUT);
  pinMode(INH_B, OUTPUT);
  pinMode(INH_C, OUTPUT);

  pinMode(INL_A, OUTPUT);
  pinMode(INL_B, OUTPUT);
  pinMode(INL_C, OUTPUT);

  pinMode(ENA_GATE, OUTPUT);
  pinMode(nMFAULT, OUTPUT);

  pinMode(SENSE_A, OUTPUT);
  pinMode(SENSE_B, OUTPUT);
  pinMode(SENSE_C, OUTPUT);

}

void togglePins(){
  digitalToggle(MOSI_DRV);
  digitalToggle(MISO_DRV);
  digitalToggle(CLK_DRV);
  digitalToggle(CS_DRV);

  digitalToggle(MOSI_ENC);
  digitalToggle(MISO_ENC);
  digitalToggle(CLK_ENC);
  digitalToggle(CS_ENC);

  digitalToggle(INH_A);
  digitalToggle(INH_B);
  digitalToggle(INH_C);

  digitalToggle(INL_A);
  digitalToggle(INL_B);
  digitalToggle(INL_C);

  digitalToggle(ENA_GATE);
  digitalToggle(nMFAULT);

  digitalToggle(SENSE_A);
  digitalToggle(SENSE_B);
  digitalToggle(SENSE_C);

}

void setup()
{
  Serial.begin(115200);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(LOOP_PIN, OUTPUT);

  setOutPutMode();
  bootLedIndicator();
  lastTime = millis();
}

void loop()
{

  if ((millis() - lastTime) > 100){
    Serial.println("Hola Nautilus !!");
    digitalToggle(BLUE_LED);
    lastTime = millis();
  }
  togglePins();
  digitalToggle(LOOP_PIN);
  delay(1);
}
*/