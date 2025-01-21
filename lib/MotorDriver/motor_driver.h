#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

/* Motor Driver Pins */
#define L_MOTOR_IN_1 13
#define L_MOTOR_IN_2 12

#define R_MOTOR_IN_1 14
#define R_MOTOR_IN_2 27
#define L_PWM 2
#define R_PWM 4
#define MOTOR_EN 5

void motor_setup(void);
void enable_motor(void);
void disable_motor(void);

/* Left Motor Functions */
void left_forward(void);
void left_reverse(void);
void left_brake(void);

/* Right Motor Functions */
void right_forward(void);
void right_reverse(void);
void right_brake(void);

/* PWM */
void write_pwm(int leftSpeed, int rightSpeed);

#endif