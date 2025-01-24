#include <motor_function.h>
#include <motor_driver.h>
#include <pid_driver.h>

void forward_wall_pid(float wallCorrection, int baseSpeed)
{
    // Encoder PID
    float encoderCorrection = calculate_encoder_pid(false);
    // float encoderCorrection = 0;
    int leftSpeed = baseSpeed - encoderCorrection;
    int rightSpeed = baseSpeed + encoderCorrection;

    // Wall PID
    leftSpeed = baseSpeed - wallCorrection;
    rightSpeed = baseSpeed + wallCorrection;

    left_forward();
    right_forward();
    write_pwm(leftSpeed, rightSpeed);
}

void forward(void)
{
    left_forward();
    right_forward();
}

void reverse(void)
{
    left_reverse();
    right_reverse();
}

void brake(void)
{
    left_brake();
    right_brake();
}

void turn_right(void)
{
    right_reverse();
    left_forward();
}

void turn_left(void)
{
    left_reverse();
    right_forward();
}