#ifndef PID_DRIVER_H
#define PID_DRIVER_H

#include <Arduino.h>
#include <TelnetStream.h>

/* Variables */
extern volatile int leftFactor;
extern volatile int rightFactor;
extern volatile float lastWallError;
extern volatile float wallIntegral;

// Encoders
extern volatile int leftEncoderValue;
extern volatile int rightEncoderValue;

float calculate_wall_pid(int leftDistance, int rightDistance, bool debug);
int calculate_left_encoder_pid(int leftTargetSpeed, bool debug);
int calculate_right_encoder_pid(int rightTargetSpeed, bool debug);

#endif