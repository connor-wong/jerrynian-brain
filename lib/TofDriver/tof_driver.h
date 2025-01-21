#ifndef TOF_DRIVER_H
#define TOF_DRIVER_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <TelnetStream.h>
#include <array>

#define XSHUT_1 32
#define XSHUT_2 33
#define XSHUT_3 23
#define XSHUT_4 25
#define XSHUT_5 26

#define tofAddress1 0x30
#define tofAddress2 0x31
#define tofAddress3 0x32
#define tofAddress4 0x33
#define tofAddress5 0x34

#define sensorNum 4

#define SDA_PIN 21  
#define SCL_PIN 22

/* Variables */
extern volatile int leftFactor;
extern volatile int rightFactor;

void tof_setup(void);
void calibrate_tof_front_threshold(void);
std::array<uint16_t, sensorNum> tof_read(bool debug);

#endif