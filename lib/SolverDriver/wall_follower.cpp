#include <wall_follower.h>
#include <tof_driver.h>
#include <pid_driver.h>
#include <encoder_turn.h>
#include <motor_function.h>

// Front Distance Threshold
const int frontDistanceThreshold = 130;
const int availableSpaceThreshold = 175;

bool rightCell = false;
bool leftCell = false;
bool calibrationFlag = false;

void wall_follower(void)
{
    if (calibrationFlag)
    {
        std::array<uint16_t, 4> readings = tof_read(false);

        int leftDistance = readings[3];
        int leftDiagonal = readings[2];
        int rightDiagonal = readings[1];
        int rightDistance = readings[0];

        check_available_cell(leftDiagonal, rightDiagonal);

        /* Wall Follower PID */
        float correction = calculate_wall_pid(leftDistance, rightDistance, false);

        // if (rightCell)
        // {
        //     if (rightDiagonal < availableSpaceThreshold) // Detect next edge
        //     {
        //         brake();
        //         delay(500);
        //         encoder_turn_right();
        //         delay(500);
        //         rightCell = false;
        //     }
        // }

        if (leftCell)
        {
            if (leftDiagonal <= availableSpaceThreshold) // Detect next edge
            {
                brake();
                delay(500);
                encoder_turn_left();
                delay(500);
                leftCell = false;
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
                leftCell = false;
                rightCell = false;
            }

            // Left cell is available
            else if (leftDistance > availableSpaceThreshold)
            {
                // TelnetStream.println("");
                //  TelnetStream.println("Left cell available!");
                encoder_turn_left();
                leftCell = false;
                rightCell = false;
            }

            else
            {
                // TelnetStream.println("");
                //  TelnetStream.println("No cell available!");
                encoder_turn_back();
                leftCell = false;
                rightCell = false;
            }

            delay(500);
        }

        else
        {
            forward_wall_pid(correction);
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
        
        calibrationFlag = true;
    }
}

void check_available_cell(int leftDiagonal, int rightDiagonal)
{
    if (!leftCell)
    {
        if (leftDiagonal > 240)
        {
            TelnetStream.println("");
            TelnetStream.println("Left cell detected!");
            leftCell = true;
        }
    }

    if (!rightCell)
    {
        if (rightDiagonal > 240)
        {
            TelnetStream.println("");
            TelnetStream.println("Right cell detected!");
            rightCell = true;
        }
    }
}