/*
#include <SimpleFOC.h>

///    SPI 2 (AS5048A)


#define MOSI_ENC PB15 // Blue
#define MISO_ENC PB14  // Green
#define CLK_ENC  PB13   // Orange
#define CS_ENC   PB12    // Yellow


MagneticSensorSPIConfig_s AS5047A_SPI_Config{
    .spi_mode = SPI_MODE1,
    .clock_speed = 100000,
    .bit_resolution = 14,
    .angle_register = 0x3FFF,
    .data_start_bit = 13,
    .command_rw_bit = 14,
    .command_parity_bit = 15};

MagneticSensorSPI sensor = MagneticSensorSPI(AS5047A_SPI_Config, CS_ENC);
SPIClass spiConnectionEncoder(MOSI_ENC, MISO_ENC, CLK_ENC);


void setup() {
    Serial.begin(115200);
    sensor.init(&spiConnectionEncoder);

    //pinMode(CS_ENC,OUTPUT);
}

void loop() {
    //digitalToggle(CS_ENC);
    sensor.update();
    Serial.println(sensor.getAngle());
    delay(1);
}
*/