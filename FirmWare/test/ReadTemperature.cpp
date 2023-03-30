/*
#include <SimpleFOC.h>
#include "STM32F405RTGBorad.h"
#include <TemperatureSensor.h>

#define ENA_GATE PC3
#define nMFAULT PC4  

#define BLUE_LED PB2
#define LOOP_PIN PC13

#define NUM_OF_BLINKS 16

TemperatureSensor temperature;


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

void setup(){
  Serial.begin(115200);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(LOOP_PIN, OUTPUT);
  temperature.init();
  bootLedIndicator();
  lastTime = millis();
}

void loop(){
  if ((millis() - lastTime) > 100){
    Serial.print("Temperature = ");
    float temp = temperature.readTemperatureC();
    Serial.print(temp);
    Serial.println(" C");
    digitalToggle(BLUE_LED);
    lastTime = millis();
  }
  //digitalToggle(PB7);
  //digitalToggle(PB6);
  digitalToggle(LOOP_PIN);
  delay(1);
}
*/