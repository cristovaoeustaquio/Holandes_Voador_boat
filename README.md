# Holandês Voador - Autonomous Boat


This project was developed as part of the Embedded Systems Programming course at UFMG - Prof. Ricardo de Oliveira Duarte – Department of Electronic Engineering.

This projects contains functions and codes to provide use of some hardware resources from an autonomous boat which is equipped with a MG90S Tower Pro servo motor and an RF-200 DC motor. The servo motor is the actuator responsible for changing the boat's orientation through the rudder. The DC motor is coupled to the boat's propulsion propeller. 
![Holandês Voador](holandes_voador.png)

For more information on how this boat should work:
------
## Hardware requirements:

These are the parts needed to assemble a boat like the Holandês Voador:
- NUCLEO-F446RET6 from STMicroeletronics.
    Manufacturer website: https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html
- Micro Servo MG90S TowerPro.
    Datasheet: https://www.electronicoscaldas.com/datasheet/MG90S_Tower-Pro.pdf
- Motor DC RF-300.
    Datasheet: https://datasheetspdf.com/pdf/917209/KYSAN/RF-300CH-11400/1
- Voltage regulators: DS-Mini-360.
    Datasheet: http://www.mantech.co.za/datasheets/products/MINI-360-R0.pdf.
- Motor Shield LD293D.
    Datasheet: https://5.imimg.com/data5/PX/UK/MY-1833510/l293d-based-arduino-motor-shield.pdf.
- 6V battery pack.
- 9V battery for the Motor Shield.
- HMC5883L Magnetometer.
    Datasheet: https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf.
- BLE V4.2 JDY-18 Bluetooth Module.
    Datasheet: https://curtocircuito.com.br/datasheet/JDY-18.pdf.


This project was implemented and tested in STM32 Nucleo-64 with STM32f446RE MCU. To use this code with another MCU, it might be necessary to check the pins and the availability of resources such as timers and PWM generation.

## Software requirements:
- STM32CubeIDE 1.10.1: Available at https://www.st.com/en/development-tools/stm32cubeide.html
- It might be helpful to use Tauno Plotter to plot data. You can download and learn to use it on https://github.com/taunoe/tauno-serial-plotter

## Project's main files:
* 'Bluetooth_BLE_V4.2_JDY-18.c' + * 'Bluetooth_BLE_V4.2_JDY-18.h' - * 'Bluetooth_BLE_V4.2_JDY-18.c' - Functions for using the BLE JDY-18 module.

## How to use it:

To use it, it is necessary to include any header file in your main file and check and modify (if neessary) the used pins.

### Authors:  
   * David Simon Marques - <davidsimon@ufmg.br>
   * Cristóvão Eustaquio da Silva - <>
   * Victor Araujo Sander Silva - <victorsander@gmail.com>

Institution: Universidade Federal de Minas Gerais (UFMG)
