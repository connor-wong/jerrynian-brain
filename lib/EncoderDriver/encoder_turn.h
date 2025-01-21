#ifndef ENCODER_TURN_H
#define ENCODER_TURN_H

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

void encoder_forward(void);
void encoder_reverse(void);
void encoder_turn_back(void);
void encoder_turn_right(void);
void encoder_turn_left(void);

#endif