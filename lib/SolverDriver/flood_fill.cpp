#include <flood_fill.h>
#include <tof_driver.h>
#include <encoder_turn.h>
#include <pid_driver.h>
#include <motor_function.h>
#include <motor_driver.h>

#define loopCost 20

// Front Distance Threshold
const int frontDistanceThreshold = 130;
const int availableSpaceThreshold = 180;
bool calibrationFlag = false;

int score[4][4];  // Scores every square, initialised to 0
int solved[4][4]; // Records the steps moved
// int history[4][4][3]; // Stores last 3 steps of each point

int x, y, facing, x_current, y_current; // Coordinates & direction faced
int moveCount = 1;                      // Total number of moves made till now
bool inDeadEnd;                         // Checks if we are inside a deadend
bool isFastRun = false;                 // During scouting, this is false
bool reachedEnd;
bool isLooping;

// Coordinates of ending point on the map (tbc)
int x1 = 2, y_1 = 2, x2 = 3, y2 = 2, x3 = 2, y3 = 1, x4 = 3, y4 = 1;

void flood_fill(void)
{
    if (calibrationFlag)
    {
        // 0,0 is initialised to be the starting point (bottom right of the maze)
        // robot facing upwards, need to change depending on starting position
        x = 0;
        y = 0;
        x_current = x - 1;
        y_current = y - 1;
        facing = 0;

        set_maze_map_scouting(x1, y_1, x2, y2, x3, y3, x4, y4);

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                if (i == x1 && j == y_1 || i == x2 && j == y2 || i == x3 && j == y3 || i == x4 && j == y4)
                {
                    score[i][j] = 0;
                }

                if (abs(i - x1) <= abs(i - x2))
                {
                    score[i][j] += abs(i - x1);
                }
                else
                {
                    score[i][j] += abs(i - x2);
                }

                if (abs(j - y_1) <= abs(j - y3))
                {
                    score[i][j] += abs(j - y_1);
                }
                else
                {
                    score[i][j] += abs(j - y3);
                }
            }
        }

        // for (int i = 0; i < 4; i++)
        // {
        //     for (int j = 0; j < 4; j++)
        //     {
        //         if (solved[i][j] == 111)
        //         {
        //             solved[i][j] = 1;
        //         }
        //         else if (solved[i][j] == 999)
        //         {
        //             solved[i][j] = 9;
        //         }
        //         solved[3 - j][i] = solved[i][j];
        //     }
        // }

        TelnetStream.println("Checking initialised score array...");

        // checking score array
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                score[3 - j][i] = score[i][j];
                TelnetStream.print(score[i][j]);
            }
            TelnetStream.println("");
        }

        TelnetStream.println("starting scouting...");

        while (!(x == x1 && y == y_1) && !(x == x2 && y == y2) && !(x == x3 && y == y3) && !(x == x4 && y == y4))
        {
            if (x != x_current || y != y_current) // basically when the x or y coordinate changes it will need to think_scout again
            {
                think_scout(true); // thinkScout() while not yet at end point
                movement_debug();
            }
        }

        brake();
        disable_motor();
        TelnetStream.println("Scouting Complete");

        // creating score array after scouting (to set the map)
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                score[i][j] = solved[i][j];
            }
        }

        x = 0;
        y = 0;
        x_current = x - 1;
        y_current = y - 1;
        facing = 0;

        TelnetStream.println("*** Restarting New Map ***");
        TelnetStream.println("Facing: " + String(facing));

        isFastRun = false;

        if (isFastRun)
        {
            while (!(x == x1 && y == y_1) && !(x == x2 && y == y2) && !(x == x3 && y == y3) && !(x == x4 && y == y4))
            {
                think_fast_run(true); // find a way to make robot move after resetting maze
            }
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

        leftEncoderValue = 0;
        rightEncoderValue = 0;
        calibrationFlag = true;
    }
}

void think_scout(bool debug)
{
    bool wallFront = false;
    bool wallLeft = false;
    bool wallRight = false;

    std::array<uint16_t, 4> readings = tof_read(false);

    int leftDistance = readings[3];
    int leftDiagonal = readings[2];
    int rightDiagonal = readings[1];
    int rightDistance = readings[0];

    float correction = calculate_wall_pid(leftDistance, rightDistance, false);

    // Reach end of wall
    if (leftDiagonal < frontDistanceThreshold && rightDiagonal < frontDistanceThreshold)
    {
        wallFront = true;
    }

    if (leftDistance < availableSpaceThreshold)
    {
        wallLeft = true;
    }

    if (rightDistance < availableSpaceThreshold)
    {
        wallRight = true;
    }

    if (debug)
    {
        TelnetStream.println("");
        TelnetStream.println("Current coordinates (" + String(x) + "," + String(y) + "): ");
        TelnetStream.println("wallFront: " + String(wallFront) + " WallLeft: " + String(wallLeft) + " WallRight: " + String(wallRight));
        TelnetStream.println("");
    }

    int wallCount = wallFront + wallRight + wallLeft;

    if (wallCount == 3) // dead end
    {
        if (debug)
        {
            TelnetStream.println("Dead End");
        }

        inDeadEnd = true; // initiate protocol
        move('b', correction, true);
    }

    else if (wallCount == 2) // only one way to go
    {
        if (!wallFront)
        {
            move('f', correction, true);
        }
        else if (!wallLeft)
        {
            move('l', correction, true);
        }
        else
        {
            move('r', correction, true);
        }
    }
    else if (wallCount < 2)
    {
        inDeadEnd = false; // no longer in dead end if we are thinking

        int dir, u, v;
        int scoreF = 111, scoreR = 111, scoreL = 111;

        // choices on which way to turn!

        if (!wallFront)
        {                 // moving to direction where there is open space, u and v represent the coordinates after moving
            dir = facing; // dir is straight ahead relative to most recent position, moving forwards

            if (dir == 1)
            {
                u = x + 1;
            }
            else if (dir == 3)
            {
                u = x - 1;
            }
            else
            {
                u = x;
            }

            if (dir == 0)
            {
                v = y + 1;
            }
            else if (dir == 2)
            {
                v = y - 1;
            }
            else
            {
                v = y;
            }

            scoreF = score[u][v]; // score of the pixel in front of robot
        }

        if (!wallRight) // moving to direction where there is open space, u and v represent the coordinates after moving
        {

            dir = (facing + 1) % 4; // dir is right relative to most recent position, moving rightwards

            if (dir == 1)
            {
                u = x + 1;
            }
            else if (dir == 3)
            {
                u = x - 1;
            }
            else
            {
                u = x;
            }

            if (dir == 0)
            {
                v = y + 1;
            }
            else if (dir == 2)
            {
                v = y - 1;
            }
            else
            {
                v = y; // leftwards
            }

            scoreR = score[u][v]; // score of pixel to the right
        }

        if (!wallLeft) // moving to direction where there is open space, u and v represent the coordinates after moving
        {

            dir = (facing + 3) % 4; // dir is left relative to most recent position, moving leftwards

            if (dir == 1)
            {
                u = x + 1;
            }
            else if (dir == 3)
            {
                u = x - 1;
            }
            else
            {
                u = x;
            }

            if (dir == 0)
            {
                v = y + 1;
            }
            else if (dir == 2)
            {
                v = y - 1;
            }
            else
            {
                v = y; // leftwards
            }

            scoreL = score[u][v];
        }

        if (debug)
        {
            TelnetStream.println("");
            TelnetStream.print("scoreF: ");
            TelnetStream.print(scoreF);
            TelnetStream.print(" scoreR: ");
            TelnetStream.print(scoreR);
            TelnetStream.print(" scoreL: ");
            TelnetStream.println(scoreL);
        }

        // checking if the squares have been visited before
        if (!(wallFront) && scoreF <= scoreR && scoreF <= scoreL) // if front scores more than right and left, go ahead
        {
            if (debug)
            {
                TelnetStream.println("");
                TelnetStream.println("Go ahead");
            }

            move('f', correction, true);
        }
        else if (!(wallRight) && scoreR <= scoreL && scoreR <= scoreF)
        {
            if (debug)
            {
                TelnetStream.println("");
                TelnetStream.println("Take a right");
            }

            move('r', correction, true);
        }
        else if (!(wallLeft) && scoreL <= scoreR && scoreL <= scoreF)
        {
            if (debug)
            {
                TelnetStream.println("");
                TelnetStream.println("Take a Left");
            }

            move('l', correction, true);
        }
        else
        {
            inDeadEnd = true;            // no other ways left to go
            move('b', correction, true); // only way left to go
        }
    }
}

void move(char relativeDir, float correction, bool debug)
{

    score[x][y]++;
    moveCount++;
    solved[x][y] = 111; // Recording the moves made in the array

    if (inDeadEnd)
    {
        score[x][y] = 999;  // Mark as dead end
        solved[x][y] = 999; // Marking dead ends as path to not travel

        if (debug)
        {
            TelnetStream.println("");
            TelnetStream.print("This is dead end, coordinates are ");
            TelnetStream.print(x);
            TelnetStream.print(", ");
            TelnetStream.println(y);
        }
    }

    if (relativeDir == 'b') // Turn back
    {
        encoder_turn_back();
        facing = (facing + 2) % 4;

        if (debug)
        {
            TelnetStream.println("");
            TelnetStream.println("Moving back");
        }
    }
    else if (relativeDir == 'r') // Turn right
    {
        encoder_turn_right();
        facing = (facing + 1) % 4;
    }
    else if (relativeDir == 'l') // Turn left
    {
        encoder_turn_left();
        facing = (facing + 3) % 4;
    }

    while (leftEncoderValue < 3000 && rightEncoderValue < 3000) // changed it to this, so while it hasnt moved a coordinate it keeps moving forward
    {
        std::array<uint16_t, 4> readings = tof_read(false);

        int leftDistance = readings[3];
        int leftDiagonal = readings[2];
        int rightDiagonal = readings[1];
        int rightDistance = readings[0];

        // Reach end of wall
        if (leftDiagonal < frontDistanceThreshold && rightDiagonal < frontDistanceThreshold)
        {
            break;
        }

        float correction = calculate_wall_pid(leftDistance, rightDistance, false);

        forward_wall_pid(correction); // Move forward
    }

    if (debug)
    {
        TelnetStream.println("");
        TelnetStream.println("leftEncoderValue: " + String(leftEncoderValue) + " rightEncoderValue: " + String(rightEncoderValue));
        TelnetStream.println("");
    }

    x_current = x;
    y_current = y;

    // Adjust the coordinates
    x = (facing == 1) ? x + 1 : (facing == 3) ? x - 1
                                              : x;

    y = (facing == 0) ? y + 1 : (facing == 2) ? y - 1
                                              : y;

    // Reset encoder values after each cell detection
    leftEncoderValue = 0;
    rightEncoderValue = 0;
}

void set_maze_map_scouting(int x1, int y_1, int x2, int y2, int x3, int y3, int x4, int y4)
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (i == x1 && j == y_1 || i == x2 && j == y2 || i == x3 && j == y3 || i == x4 && j == y4)
            {
                score[i][j] = 0;
            }

            if (abs(i - x1) <= abs(i - x2))
            {
                score[i][j] += abs(i - x1);
            }
            else
            {
                score[i][j] += abs(i - x2);
            }

            if (abs(j - y_1) <= abs(j - y3))
            {
                score[i][j] += abs(j - y_1);
            }
            else
            {
                score[i][j] += abs(j - y3);
            }
        }
    }
}

void set_maze_map_actual_run(int x1, int y_1, int x2, int y2, int x3, int y3, int x4, int y4)
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (i == x1 && j == y_1 || i == x2 && j == y2 || i == x3 && j == y3 || i == x4 && j == y4)
            {
                score[i][j] == 0;
            }
            else if (score[i][j] == 111)
            {
                score[i][j] = 0;
                if (abs(i - x1) <= abs(i - x2))
                {
                    score[i][j] += abs(i - x1);
                }
                else
                {
                    score[i][j] += abs(i - x2);
                }

                if (abs(j - y_1) <= abs(j - y3))
                {
                    score[i][j] += abs(j - y_1);
                }
                else
                {
                    score[i][j] += abs(j - y3);
                }
            }
        }
    }
}

void think_fast_run(bool debug)
{
    bool wallFront = false;
    bool wallLeft = false;
    bool wallRight = false;

    std::array<uint16_t, 4> readings = tof_read(false);

    int leftDistance = readings[3];
    int leftDiagonal = readings[2];
    int rightDiagonal = readings[1];
    int rightDistance = readings[0];

    float correction = calculate_wall_pid(leftDistance, rightDistance, false);

    // Reach end of wall
    if (leftDiagonal < frontDistanceThreshold && rightDiagonal < frontDistanceThreshold)
    {
        wallFront = true;
    }

    if (leftDistance < availableSpaceThreshold)
    {
        wallLeft = true;
    }

    if (rightDistance < availableSpaceThreshold)
    {
        wallRight = true;
    }

    int wallCount = wallFront + wallRight + wallLeft;

    if (wallCount == 3) // dead end
    {
        inDeadEnd = true; // initiate protocol
        move('b', correction, true);
    }
    else if (wallCount == 2)
    { // only one way to go
        if (!wallFront)
        {
            move('f', correction, true);
        }
        else if (!wallLeft)
        {
            move('l', correction, true);
        }
        else
        {
            move('r', correction, true);
        }
    }
    else if (wallCount < 2)
    {
        inDeadEnd = false; // no longer in dead end if we are thinking

        int dir, u, v;
        int scoreF = 111, scoreR = 111, scoreL = 111;

        // choices on which way to turn!
        if (!wallFront)
        {                 // moving to direction where there is open space, u and v represent the coordinates after moving
            dir = facing; // dir is straight ahead relative to most recent position, moving forwards

            if (dir == 1)
            {
                u = x + 1;
            }
            else if (dir == 3)
            {
                u = x - 1;
            }
            else
            {
                u = x;
            }

            if (dir == 0)
            {
                v = y + 1;
            }
            else if (dir == 2)
            {
                v = y - 1;
            }
            else
            {
                v = y;
            }

            if (u == x1 && v == y_1 || u == x2 && v == y2 || u == x3 && v == y3 || u == x4 && v == y4)
            {
                scoreF = 0;
            }
            else if (score[u][v] == 0 || score[u][v] == 999)
            {
                scoreF = 113;
            }
            else
            {
                scoreF = score[u][v]; // score of the pixel in front of robot
            }
        }

        if (!wallRight) // moving to direction where there is open space, u and v represent the coordinates after moving
        {
            dir = (facing + 1) % 4; // dir is right relative to most recent position, moving rightwards

            if (dir == 1)
            {
                u = x + 1;
            }
            else if (dir == 3)
            {
                u = x - 1;
            }
            else
            {
                u = x;
            }

            if (dir == 0)
            {
                v = y + 1;
            }
            else if (dir == 2)
            {
                v = y - 1;
            }
            else
            {
                v = y; // leftwards
            }
            if (u == x1 && v == y_1 || u == x2 && v == y2 || u == x3 && v == y3 || u == x4 && v == y4)
            {
                scoreR = 0;
            }
            else if (score[u][v] == 0 || score[u][v] == 999)
            {
                scoreR = 113;
            }
            else
            {
                scoreR = score[u][v]; // score of the pixel to the right of robot
            }
        }

        if (!wallLeft) // moving to direction where there is open space, u and v represent the coordinates after moving
        {

            dir = (facing + 3) % 4; // dir is left relative to most recent position, moving leftwards

            if (dir == 1)
            {
                u = x + 1;
            }
            else if (dir == 3)
            {
                u = x - 1;
            }
            else
            {
                u = x;
            }

            if (dir == 0)
            {
                v = y + 1;
            }
            else if (dir == 2)
            {
                v = y - 1;
            }
            else
            {
                v = y; // leftwards
            }

            if (u == x1 && v == y_1 || u == x2 && v == y2 || u == x3 && v == y3 || u == x4 && v == y4)
            {
                scoreL = 0;
            }
            else if (score[u][v] == 0 || score[u][v] == 999)
            {
                scoreL = 113;
            }
            else
            {
                scoreL = score[u][v]; // score of the pixel in front of robot
            }
        }

        if (debug)
        {
            TelnetStream.println("");
            TelnetStream.print("scoreF is ");
            TelnetStream.print(scoreF);
            TelnetStream.print("scoreR is ");
            TelnetStream.print(scoreR);
            TelnetStream.print("scoreL is ");
            TelnetStream.println(scoreL);
        }

        // checking if the squares have been visited before
        if (!(wallFront) && scoreF <= scoreR && scoreF <= scoreL && scoreF != 0 && scoreL != 0)
        {
            TelnetStream.println("");
            TelnetStream.println("Go ahead");
            move('f', correction, true); // if front scores more than right and left, go ahead
        }
        else if (!(wallRight) && scoreR <= scoreL && scoreR <= scoreF)
        {
            TelnetStream.println("");
            TelnetStream.println("Take a right");
            move('r', correction, true);
        }
        else if (!(wallLeft) && scoreL <= scoreR && scoreL <= scoreF)
        {
            TelnetStream.println("");
            TelnetStream.println("Take a left");
            move('l', correction, true);
        }
        else
        {
            inDeadEnd = true;            // no other ways left to go
            move('b', correction, true); // only way left to go
        }
    }
}

void movement_debug(void)
{
    TelnetStream.println("");

    // creating score array after scouting (correct rotation according to map)
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (solved[i][j] == 111)
            {
                solved[i][j] = 1;
            }
            else if (solved[i][j] == 999)
            {
                solved[i][j] = 9;
            }
            solved[3 - j][i] = solved[i][j];
        }
    }

    // checking score array
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            TelnetStream.print(solved[i][j]);
        }
        TelnetStream.println("");
    }
}