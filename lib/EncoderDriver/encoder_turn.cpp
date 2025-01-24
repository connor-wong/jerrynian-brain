#include <encoder_turn.h>
#include <encoder_driver.h>
#include <motor_function.h>
#include <motor_driver.h>

const long pulsesForForward = 2300;
const long pulsesForReverse = 750;
const long pulsesFor180DegreeTurn = 1950;
const long pulsesFor90DegreeTurn = 890;
const long pulsesFor45DegreeTurn = 445;

void encoder_forward(void)
{
    leftEncoderValue = 0;
    rightEncoderValue = 0;

    forward();
    write_pwm(180, 180);

    // Wait until the required pulses are reached
    while (abs(leftEncoderValue) < pulsesForForward || abs(rightEncoderValue) < pulsesForForward)
    {
        ;
        // encoder_debug();
    }

    brake();
    write_pwm(0, 0);

    leftEncoderValue = 0;
    rightEncoderValue = 0;
}

void encoder_reverse(void)
{
    leftEncoderValue = 0;
    rightEncoderValue = 0;

    reverse();
    write_pwm(180, 180);

    // Wait until the required pulses are reached
    while (abs(leftEncoderValue) < pulsesForReverse || abs(rightEncoderValue) < pulsesForReverse)
    {
        ;
        // encoder_debug();
    }

    brake();
    write_pwm(0, 0);

    leftEncoderValue = 0;
    rightEncoderValue = 0;
}

void encoder_turn_back(void)
{
    leftEncoderValue = 0;
    rightEncoderValue = 0;

    turn_right();
    write_pwm(255, 255);

    // Wait until the required pulses are reached
    while (abs(leftEncoderValue) < pulsesFor180DegreeTurn || abs(rightEncoderValue) < pulsesFor180DegreeTurn)
    {
        ;
        // encoder_debug();
    }

    brake();
    write_pwm(0, 0);

    lastWallError = 0;
    wallIntegral = 0;
    leftEncoderValue = 0;
    rightEncoderValue = 0;
}

void encoder_turn_right(void)
{
    leftEncoderValue = 0;
    rightEncoderValue = 0;

    turn_right();
    write_pwm(255, 255);

    // Wait until the required pulses are reached
    while (abs(leftEncoderValue) < pulsesFor90DegreeTurn || abs(rightEncoderValue) < pulsesFor90DegreeTurn)
    {
        ;
        // encoder_debug();
    }

    brake();
    write_pwm(0, 0);

    lastWallError = 0;
    wallIntegral = 0;
    leftEncoderValue = 0;
    rightEncoderValue = 0;
}

void encoder_turn_left(void)
{
    leftEncoderValue = 0;
    rightEncoderValue = 0;

    turn_left();
    write_pwm(255, 255);

    // Wait until the required pulses are reached
    while (abs(leftEncoderValue) < pulsesFor90DegreeTurn || abs(rightEncoderValue) < pulsesFor90DegreeTurn)
    {
        ;
        // encoder_debug();
    }

    brake();
    write_pwm(0, 0);

    lastWallError = 0;
    wallIntegral = 0;
    leftEncoderValue = 0;
    rightEncoderValue = 0;
}