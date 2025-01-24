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
extern volatile float lastEncoderError;
extern volatile float encoderIntegral;

float calculate_wall_pid(int leftDistance, int rightDistance, bool debug);
float calculate_encoder_pid(bool debug);

#endif