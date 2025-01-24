#include <wall_follower.h>
#include <tof_driver.h>
#include <pid_driver.h>
#include <encoder_turn.h>
#include <motor_function.h>

// Front Distance Threshold
const int frontDistanceThreshold = 130;
const int availableSpaceThreshold = 175;
const int wallSpeed = 130;

bool rightWallCell = false;
bool leftWallCell = false;
bool wallCalibrationFlag = false;

void wall_follower(void)
{
    if (wallCalibrationFlag)
    {
        std::array<uint16_t, 4> readings = tof_read(false);

        int leftDistance = readings[3];
        int leftDiagonal = readings[2];
        int rightDiagonal = readings[1];
        int rightDistance = readings[0];

        wall_check_available_cell(leftDiagonal, rightDiagonal);

        /* Wall Follower PID */
        float correction = calculate_wall_pid(leftDistance, rightDistance, false);

        /* Uncomment for right follower */
        // if (rightWallCell)
        // {
        //     if (rightDiagonal < availableSpaceThreshold) // Detect next edge
        //     {
        //         brake();
        //         delay(500);
        //         encoder_turn_right();
        //         delay(500);
        //         rightWallCell = false;
        //     }
        // }

        /* Left follower */
        if (leftWallCell)
        {
            if (leftDiagonal <= availableSpaceThreshold) // Detect next edge
            {
                brake();
                delay(500);
                encoder_turn_left();
                delay(500);
                leftWallCell = false;
            }
        }

        /* Reach end of wall*/
        if (leftDiagonal < frontDistanceThreshold && rightDiagonal < frontDistanceThreshold)
        {
            lastWallError = 0;
            wallIntegral = 0;
            brake();
            delay(500);

            readings = tof_read(false);

            int leftDistance = readings[3];
            int rightDistance = readings[0];

            // Right cell is available
            if (rightDistance > availableSpaceThreshold)
            {
                // TelnetStream.println("");
                //  TelnetStream.println("Right cell available!");
                encoder_turn_right();
                leftWallCell = false;
                rightWallCell = false;
            }

            // Left cell is available
            else if (leftDistance > availableSpaceThreshold)
            {
                // TelnetStream.println("");
                //  TelnetStream.println("Left cell available!");
                encoder_turn_left();
                leftWallCell = false;
                rightWallCell = false;
            }

            else
            {
                // TelnetStream.println("");
                //  TelnetStream.println("No cell available!");
                encoder_turn_back();
                leftWallCell = false;
                rightWallCell = false;
            }

            delay(500);
        }

        else
        {
            forward_wall_pid(correction, wallSpeed);
        }
    }
    else // Calibrate before start
    {
        calibrate_tof_front_threshold();
        delay(100);
        encoder_reverse();
        delay(100);
        encoder_turn_back();
        delay(100);
        encoder_reverse();
        delay(250);
        
        wallCalibrationFlag = true;
    }
}

void wall_check_available_cell(int leftDiagonal, int rightDiagonal)
{
    if (!leftWallCell)
    {
        if (leftDiagonal > 240)
        {
            TelnetStream.println("");
            TelnetStream.println("Left cell detected!");
            leftWallCell = true;
        }
    }

    if (!rightWallCell)
    {
        if (rightDiagonal > 240)
        {
            TelnetStream.println("");
            TelnetStream.println("Right cell detected!");
            rightWallCell = true;
        }
    }
}