#ifndef MOTOR_FUNCTION_H
#define MOTOR_FUNCTION_H

#include <Arduino.h>
#include <TelnetStream.h>

/* Variables */
extern volatile int leftEncoderValue;
extern volatile int rightEncoderValue;
extern volatile bool isFastRun;

void forward_wall_pid(float correction, int baseSpeed);
void forward(void);
void reverse(void);
void brake(void);
void turn_right(void);
void turn_left(void);

#endif