# Navigation_RYA_Micromouse_Robot
+ Micromouse robot (TI Tiva Launchpad) using Energia (Arduino language)
+ Author: Vo Anh Kha (voanhkha@yahoo.com)
+ Date release: August 2016

This project's repository contains a main project (Kha_Navigation_Robot) which includes the sub-modules (Gyroscope, Compass, LED&KEY, UART, Spin).
Each sub-module runs separately from the main project. The purpose of this repository's structure is for easier understanding and for better integration for later Energia projects.

The Micromouse robot can do the following things with fine smoothness:
1) Follow the line (black tape) adhered on the floor. Lines can be divided into different zones. Each zone has at least one open end.
2) Stop at the open end of the line.
3) Wait for the command sent from Zigbee host (Telegesis) to determine the next line zone.
4) Spin to the next zone with accurate spin angle using the Gyroscope.
5) Move straight to the next zone without line using the Gyroscope (or Compass, modifiable).
6) Auto-detect the line of the targeted zone then return to step 1.
7) Get the Earth's magnetic field compass angle (0-360).

Hardware used:
- Micromouse robot from Raise Your Arm 2013 contest (core MCU: Tiva C TM4C123G).
- Infrared LED digital module for line following (5 pairs of trasmitting and receiving LEDs).
- Integrated multi sensor GY-80 (including Gyro L3G4200D, Accelerometer	ADXL345, Magnetometer MC5883L and Barometer+Thermometer BMP085).
- Telegesis Zigbee Module ETRX357.
- LED&KEY module of 7-segment display (TM1638).

Updated: August 02, 2016.
