#include <flood_fill.h>
#include <tof_driver.h>
#include <encoder_turn.h>
#include <pid_driver.h>
#include <motor_function.h>

#define loopCost 20

// Front Distance Threshold
const int frontDistanceThreshold = 130;
const int availableSpaceThreshold = 175;

bool rightCell = false;
bool leftCell = false;
bool calibrationFlag = false;

int score[14][14];      // Scores every square, initialised to 0
int solved[14][14];     // Records the steps moved
int history[14][14][3]; // Stores last 3 steps of each point

int x, y, facing;  // coordinates & direction faced
int moveCount = 1; // total number of moves made till now
bool inDeadEnd;    // checks if we are inside a deadend
bool reachedEnd;
bool isLooping;
bool isFastRun = false; // during scouting, this is false

int x1 = 7, y_1 = 7, x2 = 8, y2 = 7, x3 = 7, y3 = 8, x4 = 8, y4 = 8; // coordinates of ending point on the map (tbc)

void flood_fill(void)
{
    // 0,0 is initialised to be the starting point (bottom right of the maze)
    // robot facing upwards, need to change depending on starting position
    x = 0;
    y = 0;
    facing = 0;

    set_maze_map_scouting(x1, y_1, x2, y2, x3, y3, x4, y4);

    while (!(x == x1 && y == y_1) && !(x == x2 && y == y2) && !(x == x3 && y == y3) && !(x == x4 && y == y4))
    {
        think_scout(); // thinkScout() while not yet at end point
    }

    // creating score array after scouting (to set the map)
    for (int i = 0; i < 14; i++)
    {
        for (int j = 0; j < 14; j++)
        {
            score[i][j] = solved[i][j];
        }
    }

    set_maze_map_actual_run(x1, y_1, x2, y2, x3, y3, x4, y4); // can remove in actual one

    x = 0;
    y = 0;
    facing = 0;

    TelnetStream.println("*** Restarting New Map ***");
    TelnetStream.println("Facing: " + String(facing));

    isFastRun = true;

    while (!(x == x1 && y == y_1) && !(x == x2 && y == y2) && !(x == x3 && y == y3) && !(x == x4 && y == y4))
    {
        think_fast_run(); // find a way to make robot move after resetting maze
    }
}

void think_scout(void)
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

    // lowkey dont know if this is useful since it does not looked like it has looped at all, but need to check ?? w actual map
    //  if the last 3 times we reached a square were at regular intervals, we are looping
    if (history[x][y][1] - history[x][y][0] == history[x][y][2] - history[x][y][1])
    {
        bool isLooping = true;
        score[x][y] -= loopCost;
    }

    if (wallCount == 3) // dead end
    {
        inDeadEnd = true; // initiate protocol
        move('b', correction);
    }

    else if (wallCount == 2) // only one way to go
    {
        if (!wallFront)
        {
            move('f', correction);
        }
        else if (!wallLeft)
        {
            move('l', correction);
        }
        else
        {
            move('r', correction);
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

        // checking if the squares have been visited before
        if (!(wallFront) && scoreF <= scoreR && scoreF <= scoreL) // if front scores more than right and left, go ahead
        {
            move('f', correction);
        }
        else if (!(wallRight) && scoreR <= scoreL && scoreR <= scoreF)
        {
            move('r', correction);
        }
        else if (!(wallLeft) && scoreL <= scoreR && scoreL <= scoreF)
        {
            move('l', correction);
        }
        else
        {
            inDeadEnd = true;      // no other ways left to go
            move('b', correction); // only way left to go
        }
    }
}

void move(char relativeDir, float correction)
{
    score[x][y]++;
    moveCount++;
    solved[x][y] = 111; // Recording the moves made in the array

    if (inDeadEnd)
    {
        score[x][y] = 999;  // Mark as dead end
        solved[x][y] = 999; // Marking dead ends as path to not travel
    }

    if (relativeDir == 'b') // Turn back
    {
        encoder_turn_back();
        facing = (facing + 2) % 4;
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

    forward_wall_pid(correction); // Move forward

    // Adjust the coordinates
    x = (facing == 1) ? x + 1 : (facing == 3) ? x - 1
                                              : x;

    y = (facing == 0) ? y + 1 : (facing == 2) ? y - 1
                                              : y;
}

void set_maze_map_scouting(int x1, int y_1, int x2, int y2, int x3, int y3, int x4, int y4)
{
    // Initialise score array and display it
    for (int i = 0; i < 14; i++)
    {
        for (int j = 0; j < 14; j++)
        {
            if (i == x1 && j == x1 || i == x2 && j == x2 || i == x3 && j == x3 || i == x4 && j == y4)
            {
            }
            else
            {
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

void set_maze_map_actual_run(int x1, int y_1, int x2, int y2, int x3, int y3, int x4, int y4)
{
    for (int i = 0; i < 14; i++)
    {
        for (int j = 0; j < 14; j++)
        {
            if (i == x1 && j == x1 || i == x2 && j == x2 || i == x3 && j == x3 || i == x4 && j == y4)
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

void think_fast_run(void)
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

    // lowkey dont know if its useful
    //  if the last 3 times we reached a square were at regular intervals, we are looping
    if (history[x][y][1] - history[x][y][0] == history[x][y][2] - history[x][y][1])
    {
        bool isLooping = true;
        score[x][y] -= loopCost;
    }

    if (wallCount == 3) // dead end
    {
        inDeadEnd = true; // initiate protocol
        move('b', correction);
    }
    else if (wallCount == 2)
    { // only one way to go
        if (!wallFront)
        {
            move('f', correction);
        }
        else if (!wallLeft)
        {
            move('l', correction);
        }
        else
        {
            move('r', correction);
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

            if (score[u][v] == 0 || score[u][v] == 999)
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

            if (score[u][v] == 0 || score[u][v] == 999)
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

            if (score[u][v] == 0 || score[u][v] == 999)
            {
                scoreL = 113;
            }
            else
            {
                scoreL = score[u][v]; // score of the pixel in front of robot
            }
        }

        // checking if the squares have been visited before
        if (!(wallFront) && scoreF <= scoreR && scoreF <= scoreL && scoreF != 0 && scoreL != 0)
        {
            move('f', correction); // if front scores more than right and left, go ahead
        }
        else if (!(wallRight) && scoreR <= scoreL && scoreR <= scoreF)
        {
            move('r', correction);
        }
        else if (!(wallLeft) && scoreL <= scoreR && scoreL <= scoreF)
        {
            move('l', correction);
        }
        else
        {
            inDeadEnd = true;      // no other ways left to go
            move('b', correction); // only way left to go
        }
    }
}
