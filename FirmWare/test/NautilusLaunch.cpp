/*
#include <SimpleFOC.h>
#include "STM32F405RTGBorad.h"
#include <SimpleCanFacility.h>
#include <NautilusCANMessage.h>
#include <TrapezoidalPlanner.h>

///    SPI 2 (AS5048A)
#define MOSI_DRV  PA7  // Blue
#define MISO_DRV  PA6 // Green
#define CLK_DRV   PA5 // Orange
#define CS_DRV    PA4  // Yellow

///     SPI 1  (DRV8305)
#define MOSI_ENC PB15 // Blue
#define MISO_ENC PB14  // Green
#define CLK_ENC  PB13   // Orange
#define CS_ENC   PB12    // Yellow

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

SimpleCanFacility canFacility;
NautilusCANMesage nautilusMsg;
double lastTime;

SPIClass spiConnectionDRV(MOSI_DRV, MISO_DRV, CLK_DRV);

void configureAmplierClamping(SPIClass spiConnectionToDRV){
  // Clamp sense amplifier output to 3.3V
  digitalWrite(CS_DRV, LOW);
  byte resp1 = spiConnectionToDRV.transfer(B01001100);
  byte resp2 = spiConnectionToDRV.transfer(B10100000);
  digitalWrite(CS_DRV, HIGH);
  Serial.println("Configure Amplier Clamping");
  Serial.print("Byte 0 : ");
  Serial.println(resp1, BIN);
  Serial.print("Byte 1 : ");
  Serial.println(resp2, BIN);
}
void configure6PWM_Mode(SPIClass spiConnectionToDRV){
  // Set to three PWM inputs mode to 6PWM
  digitalWrite(CS_DRV, LOW);
  byte resp1 = spiConnectionToDRV.transfer(B00111010);
  byte resp2 = spiConnectionToDRV.transfer(B0000110);
  digitalWrite(CS_DRV, HIGH);
  Serial.println("Configure 6PWM Mode");
  Serial.print("Byte 0 : ");
  Serial.println(resp1, BIN);
  Serial.print("Byte 1 : ");
  Serial.println(resp2, BIN);
}

void drv8305Initialization()
{
  Serial.println("DRIVER: DRV8305 INIT");
  pinMode(ENA_GATE, OUTPUT);
  pinMode(CS_DRV, OUTPUT);
  digitalWrite(CS_DRV, HIGH);
  configure6PWM_Mode(spiConnectionDRV);
  configureAmplierClamping(spiConnectionDRV);
  _delay(500);
  Serial.println("DRIVER: enGate Enabled");
  digitalWrite(ENA_GATE, HIGH);
}

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
//BLDCMotor motor = BLDCMotor(12);

// BLDCDriver3PWM driver = BLDCDriver3PWM (INH_A,INH_B, INH_C, ENA_GATE);
BLDCDriver6PWM driver = BLDCDriver6PWM(INH_A, INL_A, INH_B, INL_B, INH_C, INL_C, ENA_GATE);

Commander commander = Commander(Serial);

TrapezoidalPlanner planner(10);

void doPlanner(char *cmd){
  planner.doTrapezoidalPlannerCommand(cmd);
}

void doMotor(char *cmd){
  commander.motor(&motor, cmd);
}

void configureNuathilus(){
  // control loop type and torque mode
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity_openloop;
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
  motor.voltage_limit = 1;

  // motor.current_limit = 2.0;

  //  general settings
  //  pwm modulation settings
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.modulation_centered = 1.0;
}

void setup(){

  bootLedIndicator();
  Serial.begin(115200);
  Serial.println("Starting Motor configuration ....");

  canFacility.init(Kbit250, NormalCAN);
  canFacility.configSnniferFilter();
  canFacility.activateNotification();
  canFacility.begin();

  lastTime = millis();

  pinMode(LOOP_PIN, OUTPUT);

  char motor_id = 'M';
  commander.add(motor_id, doMotor, "Motor");

  motor.useMonitoring(Serial);
  pinMode(BLUE_LED, OUTPUT);
  sensor.init(&spiConnectionEncoder);

  // DRV8305 Inicialization
  spiConnectionDRV.begin();
  spiConnectionDRV.setBitOrder(MSBFIRST);
  spiConnectionDRV.setDataMode(SPI_MODE1);
  spiConnectionDRV.setClockDivider(SPI_CLOCK_DIV8);
  drv8305Initialization();

  // link the motor to the sensor
  //motor.linkSensor(&sensor);

  driver.voltage_power_supply = 22;
  driver.init();
  driver.enable();

  // link the motor and the driver
  motor.linkDriver(&driver);
  configureNuathilus();

  //  GCode move Gxx, GVxx, or GAxx - Example: G30 moves to position in rads. 
  //  GV10 sets velocity to 10 rads/s. GA5 sets acceleration to 5 rads/s/s.");
  planner.linkMotor(&motor);
  commander.add('G', doPlanner, "Motion Planner");

  motor.target = 10;
  // initialize motor
  motor.init();
  motor.initFOC();
  Serial.println("Motor ready.");
  _delay(1000);
}

void senAngleByCanBus(){
  nautilusMsg.setEncodeMessageID(ANGLE_STATE, FLOAT, 5, 0, 10);
  sensor.update();
  nautilusMsg.setFloatPayload(sensor.getAngle());
  canFacility.send(nautilusMsg.toCanMessage());
}

CanMessage rxMsgBuff[3];
int nodeID = 5;
int entity = 10;

void attendCanBusCommands(){
  int receivedMessages = canFacility.receiveCanMessages(rxMsgBuff);
  if (receivedMessages > 0){
    for (int index = 0; index < receivedMessages; index++){
      NautilusCANMesage nautilusMsg = rxMsgBuff[index];
      if ((nautilusMsg.getNodeID() == nodeID) & (nautilusMsg.getEntity() == entity)){
        if (nautilusMsg.getCommandID() == TARGET_CMD){
          motor.target = nautilusMsg.getFloatPayload();
          motor.move(motor.target);
          digitalToggle(BLUE_LED);
        }
      }
    }
  }
}

void loop()
{

  if ((millis() - lastTime) > 100){
    digitalToggle(BLUE_LED);
    //senAngleByCanBus();
    lastTime = millis();
  }
  
  motor.loopFOC();
  motor.move();
  commander.run();
  //planner.runPlannerOnTick();
  motor.monitor();
  //attendCanBusCommands();
  
  digitalToggle(LOOP_PIN);
}
*/