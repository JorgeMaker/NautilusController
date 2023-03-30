/*
#include <SimpleFOC.h>
#include "STM32F405RTGBorad.h"
#include <Adafruit_NeoPixel.h>

#define BLUE_LED PB2
#define LOOP_PIN PC13
#define RGB_PIN  PB5
#define LED_COUNT  1

#define NUM_OF_BLINKS 15

Adafruit_NeoPixel regIndicator = Adafruit_NeoPixel(LED_COUNT, RGB_PIN, NEO_GRB + NEO_KHZ800);


void bootLedIndicator(){
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

bool regIndicatorStatus = false;

 void togggleRGB(){
    if (regIndicatorStatus){
      regIndicator.setBrightness(0);
      regIndicator.show();
    }
    else{
      regIndicator.setBrightness(15);
      regIndicator.setPixelColor(0, regIndicator.Color(255, 0, 255));
      regIndicator.show();
    }
    regIndicatorStatus = !regIndicatorStatus;
 }

void setup(){
  Serial.begin(115200);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(LOOP_PIN, OUTPUT);
  //pinMode(RGB_PIN, OUTPUT);
  bootLedIndicator();
  lastTime = millis();

  regIndicator.begin();
  regIndicator.setBrightness(20); // Configura el brillo a máximo
  regIndicator.setPixelColor(0, regIndicator.Color(255, 0, 255)); // Configura el color inicial a rojo
  regIndicator.show(); // Actualiza el LED

}

void loop(){

  if ((millis() - lastTime) > 100){
    Serial.println("Hola Nautilus !!");
    digitalToggle(BLUE_LED);
    lastTime = millis();
    togggleRGB();


  }
  digitalToggle(LOOP_PIN);
  //digitalToggle(RGB_PIN);

  //strip.Color(0, 255, 0); // Configura el color a verde
  //strip.show(); // Actualiza el LED

  delay(1);
}
*/