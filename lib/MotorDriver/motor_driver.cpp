#include <motor_driver.h>

/* Variables */
int leftPwm = 255;
int rightPwm = 255;

void motor_setup(void)
{
    pinMode(MOTOR_EN, OUTPUT);

    /* Left Motor */
    pinMode(L_PWM, OUTPUT);
    pinMode(L_MOTOR_IN_1, OUTPUT);
    pinMode(L_MOTOR_IN_2, OUTPUT);

    /* Right Motor */
    pinMode(R_PWM, OUTPUT);
    pinMode(R_MOTOR_IN_1, OUTPUT);
    pinMode(R_MOTOR_IN_2, OUTPUT);

    enable_motor();
}

void enable_motor(void)
{
    digitalWrite(MOTOR_EN, HIGH);
}

void disable_motor(void)
{
    digitalWrite(MOTOR_EN, LOW);
}

void left_forward(void)
{
    digitalWrite(L_MOTOR_IN_1, HIGH);
    digitalWrite(L_MOTOR_IN_2, LOW);
}

void left_reverse(void)
{
    digitalWrite(L_MOTOR_IN_1, LOW);
    digitalWrite(L_MOTOR_IN_2, HIGH);
}

void left_brake(void)
{
    digitalWrite(L_MOTOR_IN_1, HIGH);
    digitalWrite(L_MOTOR_IN_2, HIGH);
}

void right_forward(void)
{
    digitalWrite(R_MOTOR_IN_1, HIGH);
    digitalWrite(R_MOTOR_IN_2, LOW);
}

void right_reverse(void)
{
    digitalWrite(R_MOTOR_IN_1, LOW);
    digitalWrite(R_MOTOR_IN_2, HIGH);
}

void right_brake(void)
{
    digitalWrite(R_MOTOR_IN_1, HIGH);
    digitalWrite(R_MOTOR_IN_2, HIGH);
}

void write_pwm(int leftSpeed, int rightSpeed)
{
    analogWrite(L_PWM, constrain(leftSpeed, 0, 255));
    analogWrite(R_PWM, constrain(rightSpeed, 0, 255));
}