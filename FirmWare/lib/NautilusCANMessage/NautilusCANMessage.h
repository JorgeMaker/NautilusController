#ifndef __NAUTILUS_CAN__H
#define __NAUTILUS_CAN__H

#include <SimpleCanFacility.h>

typedef struct
{
    int nodeID;
    int multicast;
    int entity;
    int dataType;
    int commandID;
} DecodedMessageID;

typedef union
{
    float floatValue;
    char bytesArray[4];
} floatUnion;

typedef union
{
    uint32_t intValue;
    char bytesArray[4];
} intUnion;

int extract(int num, int hi, int lo);

DecodedMessageID decodeMessageID(CanMessage msg);

enum NautilusCommandIDs
{
    TARGET_CMD = 100,
    PH_RESISTANCE_CMD = 200,
    MOTOR_STATUS_CMD = 101,
    PWM_MODULATION_CMD = 201,
    MODULATION_CENTER_CMD = 202,

    ZERO_ANGLE_OFFSET_CMD = 203,
    ELECTRICAL_OFFSET_CMD = 204,
    SENSOR_ZERO_CMD = 102,

    STATES_REPORT_CONFIG_CMD = 500,
    TARGET_STATE = 501,
    VOLTAGE_Q_STATE = 502,
    VOLTAGE_D_STATE = 503,
    CURRENT_Q_STATE = 504,
    CURRENT_D_STATE = 505,
    VELOCITY_STATE = 506,
    ANGLE_STATE = 507,

    VELOCITY_LIMIT_CMD = 206,
    VOLTAGE_LIMIT_CMD = 207,
    CURRENT_LIMIT_CMD = 208,

    MOTOR_CONTROL_TYPE_CMD = 210,
    TORQUE_CONTROL_TYPE_CMD = 211,
    MOTION_DOWN_SAMPLE_CMD = 212,

    VEL_PID_P_GAIN_CMD = 230,
    VEL_PID_I_GAIN_CMD = 231,
    VEL_PID_D_GAIN_CMD = 232,
    VEL_PID_OUT_RAMP_CMD = 233,
    VEL_PID_OUT_LIMIT_CMD = 234,
    VEL_PID_LPF_CMD = 235,

    ANGLE_PID_P_GAIN_CMD = 330,
    ANGLE_PID_I_GAIN_CMD = 331,
    ANGLE_PID_D_GAIN_CMD = 332,
    ANGLE_PID_OUT_RAMP_CMD = 333,
    ANGLE_PID_OUT_LIMIT_CMD = 334,
    ANGLE_PID_LPF_CMD = 335,

    CURRENT_D_PID_P_GAIN_CMD = 430,
    CURRENT_D_PID_I_GAIN_CMD = 431,
    CURRENT_D_PID_D_GAIN_CMD = 432,
    URRENT_D_PID_OUT_RAMP_CMD = 433,
    CURRENT_D_PID_OUT_LIMIT_CMD = 434,
    CURRENT_D_PID_LPF_CMD = 435,

    CURRENT_Q_PID_P_GAIN_CMD = 530,
    CURRENT_Q_PID_I_GAIN_CMD = 531,
    CURRENT_QD_PID_D_GAIN_CMD = 532,
    CURRENT_Q_PID_OUT_RAMP_CMD = 533,
    CURRENT_Q_PID_OUT_LIMIT_CMD = 534,
    CURRENT_Q_PID_LPF_CMD = 535,

    GET_VALUE = 0,
    BOOLEAN = 1,
    CHAR = 2,
    INT = 3,
    FLOAT = 4,

    MULTICAST_ENA = 1,
    MULTICAST_DIS = 0
};

class NautilusCANMesage
{

public:

    NautilusCANMesage(CanMessage);
    NautilusCANMesage();

    void setCanMessage(CanMessage);
    void setRTR(bool);
    bool getRTR();
    
    CanMessage toCanMessage();

    int getNodeID();
    int getMmulticast();
    int getEntity();
    int getDdataType();
    int getCommandID();
    CanMessage getCanMessage();

    void setFloatPayload(float);
    void setIntPayload(uint32_t);
    void setEncodeMessageID(int, int, int, int, int);
    
    float getFloatPayload();
    int getIntPayload();
    bool getBoolPayLoad();
    char getCharPayload();    

protected:
    float extactFloat(CanMessage);
    int extactInteger(CanMessage);
    bool extactBoolean(CanMessage);
    CanMessage msg;
};

#endif