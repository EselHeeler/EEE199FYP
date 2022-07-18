# EEE199FYP

Affordable Real-Time Air Quality Monitoring System for Medical Facilities

An affordable real-time air quality monitoring system for medical facilities that uses a layered architecture. The three layers are: (1) the sensing layer/sensor node which is responsible for sensing the indoor air quality and is the basis of the entire IAQM system, (2) the processing layer/gateway which performs necessary processes on the gathered data from the sensing layer before passing it to the application layer, and (3) the application layer which is responsible for storing processed data and presenting said data to the users.

This repository contains the libraries, schematic, and scripts used for the project. 

Components:
-ESP32 MCU
-BME680 sensor
-PMS7003 sensor
-MQ135 sensor
-MQ7 sensor
-external antenna

Arduino IDE
Libraries
-Adafruit BME680 library
-MQ135 library
-PMS library
-EspSoftwareSerial library
-ThingsBoard library
-Arduino_JSON
-ArduinoHttpClient
-Painless Mesh
-TaskScheduler

WiFi 
The access point details in the provided code must be changed depending on the network used. 

ThingsBoard
The ThingsBoard details in the provided code must be changed depending on the access token provided after creating a device. 

