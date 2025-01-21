#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <Arduino.h>
#include <TelnetStream.h>

/* Motor Encoder Pins */
#define L_ENCODER_A 18
#define L_ENCODER_B 19
#define R_ENCODER_A 35
#define R_ENCODER_B 34

/* Variables */
extern volatile int leftEncoderValue;
extern volatile int rightEncoderValue;
extern volatile bool encoderDataReady;

void encoder_setup(void);
void encoder_left_isr(void);
void encoder_right_isr(void);
void encoder_debug(void);

#endif