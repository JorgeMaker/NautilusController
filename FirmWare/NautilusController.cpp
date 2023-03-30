#include "STM32F405RTG6Board.h"
#include <SimpleFOC.h>

///   I2C 1
#define SDA PB7
#define SCL PB6
//  Temperature Senror
#define ADDR_TEMP_SENSOR 0x48

///    SPI 2 (AS5048A)
#define MOSI_ENC PB15
#define MISO_ENC PB14
#define CLK_ENC PB13
#define CS_ENC PB12

///     SPI 1  (DRV8305)
#define MOSI_DRV PA7
#define MISO_DRV PA6
#define CLK_DRV PA5
#define CS_DRV PA4

#define ENA_GATE PC1
#define nMFAULT PC3

#define INH_A PC7
#define INH_B PA1
#define INH_C PA3

#define INL_A PC6
#define INL_B PA0
#define INL_C PA2

#define SENSE_A PC5
#define SENSE_B PB0
#define SENSE_C PB1

// Status indicators
#define LOOP_PIN PB10
#define BLUE_LED PA15
#define CONFIG_SW PC4

#define CAN_RX PA11
#define CAN_TX PA12

//////////////////////////////////////
//    GLOBAL VARIABLES
//////////////////////////////////////
double timeStamp = 0;
const float  MAX_TEMP = 80 ; //Maximum temperature [°C]
float temperature;
MagneticSensorSPIConfig_s AS5047A_SPI_Config{
    .spi_mode = SPI_MODE1,
    .clock_speed = 10000000,
    .bit_resolution = 14,
    .angle_register = 0x3FFF,
    .data_start_bit = 13,
    .command_rw_bit = 14,
    .command_parity_bit = 15
};
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047A_SPI_Config, CS_ENC);
SPIClass spiConnectionEncoder(MOSI_ENC, MISO_ENC, CLK_ENC);
SPIClass spiConnectionDRV(MOSI_DRV, MISO_DRV, CLK_DRV);
// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver6PWM driver = BLDCDriver6PWM (INH_A, INL_A, INH_B, INL_B, INH_C, INL_C, ENA_GATE);
//instantiate commander
Commander commander = Commander(Serial4);
void doMotor(char* cmd){commander.motor(&motor, cmd);}
//////////////////////////////////////
//    UTILITY FUNCTIONS
//////////////////////////////////////
#define NUM_OF_BLINKS 15

void bootLedIndicator(){
    int counter = NUM_OF_BLINKS;
    while (counter > 0){
        digitalToggle(BLUE_LED);
        delay(50);
        counter--;
    }
}
float readTemperature()
{
  unsigned int data[2];
  data[0] = B00000000;
  data[1] = B00000000;
  // Start I2C transmission
  Wire.beginTransmission(ADDR_TEMP_SENSOR);
  // Select data register
  Wire.write(0x00);
  // Stop I2C transmission
  Wire.endTransmission();
  // Request 2 bytes of data
  Wire.requestFrom(ADDR_TEMP_SENSOR, 2);
  // Read 2 bytes of data
  if (Wire.available() == 2)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
  }
  // Convert the data to 10-bits
  int temp = (((data[0] & 0xFF) * 256) + (data[1] & 0xC0)) / 64;
  if (temp > 511)
  {
    temp -= 1024;
  }
  return temp * 0.25;
}
void configure6PWM_Mode(SPIClass spiConnectionToDRV){
  //Set to three PWM inputs mode to 6PWM
  digitalWrite(CS_DRV, LOW);
  byte resp1 = spiConnectionToDRV.transfer(B00111010);
  byte resp2 = spiConnectionToDRV.transfer(B0000110);
  digitalWrite(CS_DRV, HIGH);
  Serial4.println("Configure 6PWM Mode");
  Serial4.print("Byte 0 : "); Serial4.println(resp1, BIN);
  Serial4.print("Byte 1 : "); Serial4.println(resp2, BIN);
}

void configureAmplierClamping(SPIClass spiConnectionToDRV){
  //Clamp sense amplifier output to 3.3V
  digitalWrite(CS_DRV, LOW);
  byte resp1 = spiConnectionToDRV.transfer(B01001100);
  byte resp2 = spiConnectionToDRV.transfer(B10100000);
  digitalWrite(CS_DRV, HIGH);
  Serial4.println("Configure Amplier Clamping");
  Serial4.print("Byte 0 : "); Serial4.println(resp1, BIN);
  Serial4.print("Byte 1 : "); Serial4.println(resp2, BIN);
}
void drv8305Initialization()
{
  Serial4.println("DRIVER: DRV8305 INIT");
  pinMode(ENA_GATE, OUTPUT);
  pinMode(CS_DRV, OUTPUT);
  digitalWrite(CS_DRV, HIGH);  
  configure6PWM_Mode(spiConnectionDRV);
  configureAmplierClamping(spiConnectionDRV);
  _delay(500);
  Serial4.println("DRIVER: enGate Enabled");
  digitalWrite(ENA_GATE, HIGH);
}

void setup(){
    // Serial communications initialization
    Serial4.begin(115200);
    _delay(2000);
    Serial4.println("*************************************");
    Serial4.println("Setup Nautilus Controller Started !!!");
    // Pinout configuration
    pinMode(BLUE_LED, OUTPUT);
    pinMode(LOOP_PIN, OUTPUT);
    pinMode(nMFAULT, INPUT);
    // Sensor Configuration & inicialization 
    sensor.init(&spiConnectionEncoder);
    //sensor.initAbsoluteZero();
    //sensor.initRelativeZero();
    // link the motor to the sensor
    motor.linkSensor(&sensor);
    // DRV8305 Inicialization
    spiConnectionDRV.begin();
    spiConnectionDRV.setBitOrder(MSBFIRST);
    spiConnectionDRV.setDataMode(SPI_MODE1);
    spiConnectionDRV.setClockDivider(SPI_CLOCK_DIV8);
    drv8305Initialization();
    // driver config
    // power supply voltage [V]
    driver.voltage_power_supply = 11;
    driver.init();
    // link the motor and the driver
    motor.linkDriver(&driver);
    // choose FOC modulation
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    // set control loop type to be used
    motor.controller = MotionControlType::angle;
    // contoller configuration based on the control type 
    motor.PID_velocity.P = 0.2;
    motor.PID_velocity.I = 20;
    motor.PID_velocity.D = 0;
    // default voltage_power_supply
    motor.voltage_limit = 11;
    // velocity low pass filtering time constant
    motor.LPF_velocity.Tf = 0.2;
    // angle loop controller
    motor.P_angle.P = 20;
    // angle loop velocity limit
    motor.velocity_limit = 50;
    // comment out if not needed
    motor.useMonitoring(Serial4);
    // initialize motor
    motor.init();
    // align encoder and start FOC
    motor.initFOC();
    // set the inital target value
    motor.target = 0;
    // define the motor id
    commander.add('M', doMotor, "motor");
    Serial4.println("*************************************");
    bootLedIndicator();
    _delay(1000);
    timeStamp = millis();
}

void loop(){
    // iterative setting FOC phase voltage
    motor.loopFOC();
    motor.move();
    motor.monitor();
    if ((millis() - timeStamp) > 250){
        timeStamp = millis();
        digitalToggle(BLUE_LED);
        temperature = readTemperature();
        if(temperature > MAX_TEMP){
          digitalWrite(ENA_GATE, LOW);
          while(true){
            Serial4.println("NautilusBoard Dissabled. Please stop and reboot after cooling");
            bootLedIndicator();
          }
        }
    }
    // user communication
    commander.run();
    digitalToggle(LOOP_PIN);
}
