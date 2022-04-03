# Speed-control-of-BLDC


In this repository, I will share the code for controlling the speed of BLDC motor

I will give the details to calculate the speed of BLDC motor and control the speed variable in real time.


For controlling the BLDC motor speed, I developed some code for microcontrollers the devices ATSAM3X8E and Atmega2560.


Connections for Arduino Due:

 The connection for the gate driver board is.
 pin 34          PWML0 -------> T4
 pin 35          PWMH0 -------> T1
 pin 36          PWML1 -------> T6
 pin 37          PWMH1 -------> T3
 pin 38          PWML2 -------> T2 
 pin 39          PWMH2 -------> T5

 pin 25          Hu -- yellow
 pin 26          Hv -- green
 pin 27          Hw -- blue

The Rotary Encoder signals are connected as
pin 4           PHA
pin 5           PHB


Connections for Arduino Mega:
