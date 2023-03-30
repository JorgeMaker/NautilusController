/*
#include <SimpleFOC.h>
#include <TrapezoidalPlanner.h>

#define TEST_LED PA8
// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);

TrapezoidalPlanner planner(10);

#define NUM_OF_BLINKS 15
void bootLedIndicator()
{
    int counter = NUM_OF_BLINKS;
    pinMode(TEST_LED, OUTPUT);
    while (counter > 0)
    {
        digitalToggle(TEST_LED);
        delay(50);
        counter--;
    }
}

Commander commander = Commander(Serial);

void doPlanner(char *cmd){
  planner.doTrapezoidalPlannerCommand(cmd);
}

void setup()
{
    bootLedIndicator();
    Serial.begin(115200);
    //  GCode move Gxx, GVxx, or GAxx - Example: G30 moves to position in rads. 
    //  GV10 sets velocity to 10 rads/s. GA5 sets acceleration to 5 rads/s/s.");

    planner.linkMotor(&motor);
    commander.add('G', doPlanner, "Motion Planner");
}

void loop(){
    commander.run();
    planner.runPlannerOnTick();
}
*/
