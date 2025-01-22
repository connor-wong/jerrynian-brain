#include <pid_driver.h>

// Wall PID Parameters
const double wallKp = 0.70;
const double wallKi = 0.0;
const double wallKd = 2.0;

// Encoder PID Parameters
const double encoderKp = 1;
const double encoderKi = 0.0;
const double encoderKd = 0.0;

const int availableSpaceThreshold = 130;
const int wallDistance = 115;

float leftIntegral = 0, rightIntegral = 0;
float leftPrevError = 0, rightPrevError = 0;

float calculate_wall_pid(int leftDistance, int rightDistance, bool debug)
{

    if (leftDistance > availableSpaceThreshold)
    {
        leftDistance = 115;
    }

    if (rightDistance > availableSpaceThreshold)
    {
        rightDistance = 115;
    }

    double error = leftDistance - rightDistance;
    wallIntegral += error;

    float proportional = wallKp * error;
    float derivative = error - lastWallError;

    float correction = proportional + wallKi * wallIntegral + wallKd * derivative;
    lastWallError = error;

    if (debug)
    {
        TelnetStream.println("");
        TelnetStream.print("Correction: ");
        TelnetStream.println(correction);
    }

    return correction;
}

int calculate_left_encoder_pid(int leftTargetSpeed, bool debug)
{
    // Calculate speed errors
    float leftError = leftTargetSpeed - leftEncoderValue;

    // Update integral terms
    leftIntegral += leftError;

    // Calculate derivative terms
    float leftDerivative = leftError - leftPrevError;

    // Compute PID outputs for motors
    float leftPWM = encoderKp * leftError + encoderKi * leftIntegral + encoderKd * leftDerivative;

    // Save errors for next loop
    leftPrevError = leftError;

    // Constrain PWM values
    leftPWM = constrain(leftPWM, 0, 255);

    return leftPWM;
}

int calculate_right_encoder_pid(int rightTargetSpeed, bool debug)
{
    // Calculate speed errors
    float rightError = rightTargetSpeed - rightEncoderValue;

    // Update integral terms
    rightIntegral += rightError;

    // Calculate derivative terms
    float rightDerivative = rightError - rightPrevError;

    // Compute PID outputs for motors
    float rightPWM = encoderKp * rightError + encoderKi * rightIntegral + encoderKd * rightDerivative;

    // Save errors for next loop
    rightPrevError = rightError;

    // Constrain PWM values
    rightPWM = constrain(rightPWM, 0, 255);

    return rightPWM;
}