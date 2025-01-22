#include <pid_driver.h>

/* PID Control Parameters */
const double Kp = 0.70;
const double Ki = 0.0;
const double Kd = 2.0;

const int availableSpaceThreshold = 180;

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

    float proportional = Kp * error;
    float derivative = error - lastWallError;

    float correction = proportional + Ki * wallIntegral + Kd * derivative;
    lastWallError = error;

    if (debug)
    {
        TelnetStream.println("");
        TelnetStream.print("Correction: ");
        TelnetStream.println(correction);
    }

    return correction;
}