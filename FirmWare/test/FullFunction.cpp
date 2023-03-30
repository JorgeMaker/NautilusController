/*
#include <SimpleFOC.h>
#include "STM32F405RTGBorad.h"

///    SPI 2 (AS5048A)
#define MOSI_ENC PB15   //Blue
#define MISO_ENC PB14   //Green
#define CLK_ENC PB13    //Orange
#define CS_ENC PB12     //Yellow

///     SPI 1  (DRV8305)
#define MOSI_DRV PA7    //Blue
#define MISO_DRV PA6    //Green
#define CLK_DRV PA5     //Orange
#define CS_DRV PA4      //Yellow

#define INH_A PA0
#define INH_B PA1
#define INH_C PA2

#define ENA_GATE PC1

#define TEST_LED PA8
#define LOOP_PIN PB10

MagneticSensorSPIConfig_s AS5047A_SPI_Config{
    .spi_mode = SPI_MODE1,
    .clock_speed = 10000000,
    .bit_resolution = 14,
    .angle_register = 0x3FFF,
    .data_start_bit = 13,
    .command_rw_bit = 14,
    .command_parity_bit = 15};

MagneticSensorSPI sensor = MagneticSensorSPI(AS5047A_SPI_Config, CS_ENC);
SPIClass spiConnectionEncoder(MOSI_ENC, MISO_ENC, CLK_ENC);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM (INH_A,INH_B, INH_C, ENA_GATE);

Commander commander = Commander(Serial4);

void doMotor(char *cmd) { 
  commander.motor(&motor, cmd); 
}

void configureNuathilus(){
// control loop type and torque mode 
motor.torque_controller = TorqueControlType::voltage;
motor.controller = MotionControlType::velocity;
motor.motion_downsample = 0.0;
driver.voltage_power_supply = 22;
// velocity loop PID
motor.PID_velocity.P = 10.0;
motor.PID_velocity.I = 10.0;
motor.PID_velocity.D = 0.0;
motor.PID_velocity.output_ramp = 1000.0;
motor.PID_velocity.limit = 20.0;
// Low pass filtering time constant 
motor.LPF_velocity.Tf = 0.2;
// angle loop PID
motor.P_angle.P = 10.0;
motor.P_angle.I = 1.0;
motor.P_angle.D = 0.0;
motor.P_angle.output_ramp = 0.0;
motor.P_angle.limit = 10.0;
// Low pass filtering time constant 
motor.LPF_angle.Tf = 0.3;
// current q loop PID 
motor.PID_current_q.P = 3.0;
motor.PID_current_q.I = 300.0;
motor.PID_current_q.D = 0.0;
motor.PID_current_q.output_ramp = 0.0;
motor.PID_current_q.limit = 12.0;
// Low pass filtering time constant 
motor.LPF_current_q.Tf = 0.005;
// current d loop PID
motor.PID_current_d.P = 3.0;
motor.PID_current_d.I = 300.0;
motor.PID_current_d.D = 0.0;
motor.PID_current_d.output_ramp = 0.0;
motor.PID_current_d.limit = 12.0;
// Low pass filtering time constant 
motor.LPF_current_d.Tf = 0.005;
// Limits 
motor.velocity_limit = 20.0;
motor.voltage_limit = 22.0;
//motor.current_limit = 2.0;
// general settings 
// pwm modulation settings 
motor.foc_modulation = FOCModulationType::SinePWM;
motor.modulation_centered = 1.0;

}

void setup() {
    Serial4.begin(115200);
    Serial4.println("Starting Motor configuration ....");
    commander.add('M', doMotor, "my motor");
    motor.useMonitoring(Serial4);
    pinMode(TEST_LED, OUTPUT);
    sensor.init(&spiConnectionEncoder);
    // link the motor to the sensor
    motor.linkSensor(&sensor);
    driver.init();
    // link the motor and the driver
    motor.linkDriver(&driver);
    configureNuathilus();
    // initialize motor
    motor.init();
    motor.initFOC();
    Serial4.println("Motor ready.");
    _delay(1000);

    motor.target = 10;
}

void loop() {

  digitalToggle(LOOP_PIN);

  //driver.setPwm(10,10,10);

  
  motor.loopFOC();
  motor.move();
  commander.run();
  motor.monitor();
  
}
*/