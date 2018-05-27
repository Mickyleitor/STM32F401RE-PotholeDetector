# Pervasive Systems 2018 - Pothole Detector

This repository contains all information regarding the code of STM32F401RE for the Pothole Detector project.

This board is part of a bigger system, it will be connected to other board via I2C and it will send data through LoRa.
The code send via I2C the events detected from LSM6DSL (x-nucleo-iks01a2) accelerometer.

## Instrucctions
This project was done using a generated code by System Workbench toolchain distributed by STMicroelectronics, you can download the pack [here](http://www.st.com/en/development-tools/sw4stm32.html) as well as eclipse IDE.

### 1. First you need to make the board connections:

<center>

<img src="https://raw.githubusercontent.com/Mickyleitor/STM32F401RE-PotholeDetector/master/Docs/Board-connections.png" width="692">

| STM32F401RE (X-NUCLEO-IKS01A2)  | Seeeduino LoRaWAN w/ GPS |
|             :---:               |          :---:           |
|         PA8 (I2C3 SCL)          |         PC5 (SCL)        |
|         PC9 (I2C3 SDA)          |         PC4 (SDA)        |
|              +5V                |           +5V            |
|              GND                |           GND            |
  
</center>

### 2. Upload the code inside the board
* The code contains all libraries used by peripherical modules but make sure you all have a working environment.
* Import the code by manualling clicking right on the Project Explorer
* Build the code and run it.
* If it fails then make sure libraries of X-NUCLEO-IKS01A2 (Drivers/BSP) dependencies are added inside project properties.

### 3. Testing board, button and LED.
You can see basic information of what's happening inside the board. 
* LED: if something fails it would flash very quickly (250 ms) infinitively.
* Button: via SERCOM (using Putty, for example) you can view:

| Times button pressed within 1 s |            Functionality              |
|             :---:               |                :---:                  |
|               1                 | Print counted events and timer status |
|               2                 |    Switch between Burst/Timed mode    |
|               3                 |  Test I2C connection and send events  |

### 4. Burst / Timed Modes
* Timed mode means every detected events is stored until a timer interrupts. If Timer interrupts it's sent via I2C to Seeeduino. 
* In Burst mode there is no need to wait for the Timer IT so every detected events will be sent instantly to Seeeduino via I2C.

### 5. DEBUG Mode
You can view all basic information like detected potholes and low-level processes by uncommenting the DEBUG_MODE define in [main.c](https://github.com/Mickyleitor/STM32F401RE-PotholeDetector/blob/master/Src/main.c) file and openning a SERIAL/USART terminal monitor.

## Finite State Machine Diagram
![Finite State Machine Diagram](https://raw.githubusercontent.com/Mickyleitor/STM32F401RE-PotholeDetector/master/Docs/State-machine.png)

## Credits

Michele La Malva Moreno - [LinkedIn](https://www.linkedin.com/in/michele-la-malva-moreno/) | [Blogspot](https://mickysim.blogspot.com/) | [Github](https://github.com/Mickyleitor)
