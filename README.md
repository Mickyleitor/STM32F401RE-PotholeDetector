# Pervasive Systems 2018 - Pothole Detector

This repository contains all information regarding the code of STM32F401RE for the Pothole Detector project.

## Introduction
This board is part of a bigger system, it will be connected to other board via SPI/I2C and it will send data through LoRa.

The code send via SPI/I2C the events detected from LSM6DSL (x-nucleo-iks01a2) accelerometer.

## Finite State Machine Diagram
![Finite State Machine Diagram](https://raw.githubusercontent.com/Mickyleitor/STM32F401RE-PotholeDetector
/master/Docs/State-machine.png)

## Serial Peripheral Interface Payload Format
![Serial Peripheral Interface Payload](https://raw.githubusercontent.com/Mickyleitor/STM32F401RE-PotholeDetector
/master/Docs/Payload-format.png)
## Credits

Michele La Malva Moreno - [LinkedIn](https://www.linkedin.com/in/michele-la-malva-moreno/) | [Blogspot](https://mickysim.blogspot.com/) | [Github](https://github.com/Mickyleitor)