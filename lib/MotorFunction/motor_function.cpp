#include <motor_function.h>
#include <motor_driver.h>

void forward_wall_pid(float correction)
{
    int baseSpeed = 130;
    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

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