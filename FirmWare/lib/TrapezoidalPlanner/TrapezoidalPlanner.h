#ifndef __TRAPEZOIDAL_PLANNER__H
#define __TRAPEZOIDAL_PLANNER__H
#include <SimpleFOC.h>

class TrapezoidalPlanner
{
public:
    TrapezoidalPlanner(int);
    void doTrapezoidalPlannerCommand(char *command);
    void linkMotor(BLDCMotor*);
    void runPlannerOnTick();
    bool isPlannerMoving();
private:
    BLDCMotor* motor;
    unsigned long plannerTimeStap;
    int plannerPeriod; // 1000 / this number = Hz, i.e. 1000 / 100 = 10Hz, 1000 / 10 = 100Hz, 1000 / 5 = 200Hz, 1000 / 1 = 1000hZ
    float Vmax_;    // # Velocity max (rads/s)
    float Amax_;    // # Acceleration max (rads/s/s)
    float Dmax_;    // # Decelerations max (rads/s/s)
    float Y_;
    float Yd_;
    float Ydd_;
    float Tf_, Xi_, Xf_, Vi_, Ar_, Dr_, Vr_, Ta_, Td_, Tv_, yAccel_;
    unsigned long plannerStartingMovementTimeStamp;
    bool isTrajectoryExecuting;
    float sign(float val);
    float sign_hard(float val);
    bool calculateTrapezoidalPathParameters(float Xf, float Xi, float Vi, float Vmax, float Amax, float Dmax);
    void startExecutionOfPlannerTo(float newPos);
    void computeStepValuesForCurrentTime(float currentTrajectoryTime);
};

#endif