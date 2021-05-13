<p align="center">
  <img  src="https://github.com/JorgeMaker/NautilusController/blob/main/docs/nautiluslogo.png?raw=true">
</p>

## Nautilus Brushless Controller

The Nautilus Controller, a project still under development will to turn a hobby brushless motors into a robotic servo actuator.  for precise control of 3-phase brushless motors.

The board   will integrate the necessary drive electronics, a high performance SMT32 microcontroller, an absolute magnetic encoder, and a high-speed data interface in a compact board that cab be attached at the rear part of a BLDC motor.

<p align="center">
  <img  src="https://github.com/JorgeMaker/NautilusController/blob/main/docs/NautilusBoard.jpg?raw=true">
</p>

Some features of the project under development are:

- Compact size of 48x48 mm 4-layer PCB
- SMT32F405RGT6 as MCU
- Uses the DRV8305 as gate driver.
- Absolute 14 bits magnetic encoder AS5048A with SPI communications
- Power Connector: 2x XT30PW-F
- Dual connector of CAN bus interface
- Data Connector: 4x Hirose DF13
- Status LEDs
- RS232 communications
- Temperature sensor

It will be able to operate in in torque, velocity or position modes, using Field Oriented Control (FOC) developed using the open source project [SimpleFOC](https://github.com/simplefoc). For communications it will offer a dual interface based on CAN bus for a reliable network communication and RS322 for configuration and basic operation.

The ultimate goal of the controller under development is to be able to develop A ROS actuator that can be used to build dynamic robots like, as for examples, a robot arm or a quadruped robot.

As previously stated, this project is still in the very early stages of development, under construction and continuous change so take it in consideration if you decide to reuse some of the provided information.
