#include <pid_driver.h>

// Wall PID Parameters
const double wallKp = 0.70;
const double wallKi = 0.0;
const double wallKd = 2.0;

// Encoder PID Parameters
const double encoderKp = 1.0;
const double encoderKi = 0.0;
const double encoderKd = 0.0;

const int availableSpaceThreshold = 130;
const int wallDistance = 117;

float calculate_wall_pid(int leftDistance, int rightDistance, bool debug)
{
    if (leftDistance > availableSpaceThreshold)
    {
        leftDistance = wallDistance;
    }

    if (rightDistance > availableSpaceThreshold)
    {
        rightDistance = wallDistance;
    }

    float error = leftDistance - rightDistance;
    wallIntegral += error;

    float proportional = wallKp * error;
    float derivative = error - lastWallError;

    float correction = proportional + wallKi * wallIntegral + wallKd * derivative;
    lastWallError = error;

    if (debug)
    {
        TelnetStream.println("");
        TelnetStream.print("Wall PID Correction: ");
        TelnetStream.println(correction);
    }

    return correction;
}

float calculate_encoder_pid(bool debug)
{
    float encoderError = leftEncoderValue - rightEncoderValue;
    encoderIntegral += encoderError;

    float proportional = encoderKp * encoderError;
    float derivative = encoderError - lastEncoderError;

    float correction = proportional + encoderKi * encoderIntegral + encoderKd * derivative;
    lastEncoderError = encoderError;

    if (debug)
    {
        TelnetStream.println("");
        TelnetStream.print("Encoder PID Correction: ");
        TelnetStream.println(correction);
    }

    return correction;
}