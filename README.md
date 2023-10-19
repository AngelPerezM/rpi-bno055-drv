# rpi-bno055-drv
_BNO055 IMU driver in C++ for RPi targets_

This repo contains a driver for the BNO055 IMU.
The driver uses the Bus Handlers layer to communicate with the I2C bus from the RPi.
The driver is implemented as a class, this way, several instances can be included in your application.

## Project's structure
- [*`include`*](include): Contains the header of the driver class.
- [*`src`*](src): Contains the implementation of the driver class.
- [*`deps`*](deps) Contains the dependencies of this driver, that is the [`Bus Handlers`](https://github.com/AngelPerezM/rpi-bushdl) layer for the RPi.
