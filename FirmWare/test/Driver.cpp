/*
#include <SimpleFOC.h>
#include "STM32F405RTGBorad.h"

#define INH_A PC7 // TIM8_CH2
#define INH_B PA1 // TIM2_CH2
#define INH_C PA3 // TIM2_CH4

#define INL_A PC6 // TIM8-CH1
#define INL_B PA0 // TIM2-CH1
#define INL_C PA2 // TIM2-CH3

#define ENA_GATE PC3

#define BLUE_LED PB2
#define LOOP_PIN PC13

#define NUM_OF_BLINKS 15

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
BLDCDriver6PWM driver = BLDCDriver6PWM(INH_A, INL_A, INH_B, INL_B, INH_C, INL_C, ENA_GATE);


void configureDriver(){
    // pwm frequency to be used [Hz]
    driver.pwm_frequency = 20000;
    // power supply voltage [V]
    driver.voltage_power_supply = 22;
    // Max DC voltage allowed - default voltage_power_supply
    driver.voltage_limit = 12;
    // daad_zone [0,1] - default 0.02 - 2%
    driver.dead_zone = 0.05;
}

void setup(){
    bootLedIndicator();
    pinMode(LOOP_PIN, OUTPUT);
    configureDriver();
    // driver init
    driver.init();
    // enable driver
    driver.enable();
    lastTime = millis();
    _delay(1000);
}

void loop(){
    if ((millis() - lastTime) > 100){
        digitalToggle(BLUE_LED);
        lastTime = millis();
    }
    // setting pwm
    // phase A: 3V, phase B: 6V, phase C: 5V
    driver.setPwm(3, 6, 5);
    digitalToggle(LOOP_PIN);
}
*/