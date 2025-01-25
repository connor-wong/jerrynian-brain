#include <flood_fill.h>
#include <tof_driver.h>
#include <encoder_turn.h>
#include <pid_driver.h>
#include <motor_function.h>
#include <motor_driver.h>
#include <global_command_manager.h>
#include <wireless_driver.h>

#define loopCost 20

// Speed Profile
const int normalSpeed = 130;
const int fastSpeed = 150;
const int cellNormalDistance = 2100;
const int cellFastDistance = 2100;
int baseSpeed;
int cellDistance;
bool rightCell = false;
bool leftCell = false;

// Front Distance Threshold
const int frontDistanceThreshold = 145;
const int availableSpaceThreshold = 170;
bool calibrationFlag = false;

// Number of cells
const int mazeType = 14;

int score[mazeType][mazeType];  // Scores every square, initialised to 0
int solved[mazeType][mazeType]; // Records the steps moved
int debug_maze[mazeType][mazeType];
int alt_path_back[mazeType][mazeType];

int x, y, facing, x_current, y_current; // Coordinates & direction faced
bool inDeadEnd;                         // Checks if we are inside a deadend
bool if_alt_path_back_run = false;

// Coordinates of ending point on the map (4x4)
// int x1 = 1, y_1 = 1, x2 = 1, y2 = 2, x3 = 2, y3 = 1, x4 = 2, y4 = 2;

// Coordinates of ending point on the map (14x14)
int x1 = 6, y_1 = 6, x2 = 6, y2 = 7, x3 = 7, y3 = 6, x4 = 7, y4 = 7;

void flood_fill(void)
{

    wait_for_command();

    // 0,0 is initialised to be the starting point (bottom right of the maze)
    // robot facing upwards, need to change depending on starting position
    x = 0;
    y = 0;
    x_current = x - 1;
    y_current = y - 1;
    facing = 0;

    if (!isFastRun)
    {
        if (calibrationFlag)
        {
            set_maze_map_scouting(x1, y_1, x2, y2, x3, y3, x4, y4);

            TelnetStream.println("Checking initialised score array...");

            // Checking score array
            for (int i = 0; i < mazeType; i++)
            {
                for (int j = 0; j < mazeType; j++)
                {
                    // score[3 - j][i] = score[i][j];
                    TelnetStream.print(score[i][j]);
                }
                TelnetStream.println("");
            }

            baseSpeed = normalSpeed;
            cellDistance = cellNormalDistance;
            TelnetStream.println("");
            TelnetStream.println("*** Scouting Started ***");

            solved[0][0] = 111;

            while (!(x == x1 && y == y_1) && !(x == x2 && y == y2) && !(x == x3 && y == y3) && !(x == x4 && y == y4))
            {
                if (x != x_current || y != y_current) // basically when the x or y coordinate changes it will need to think_scout again
                {
                    think_scout(true); // thinkScout() while not yet at end point
                    movement_debug();
                }
            }

            brake();
            TelnetStream.println("");
            TelnetStream.println("*** Scouting Complete ***");

            // alternate path back after fast run - COMMENT OUT IF NOT USING
            TelnetStream.println("");
            TelnetStream.println("*** Starting Path Back ***");
            if_alt_path_back_run = true;
            delay(3000);
            set_maze_map_alt_path_back(); // sets and print maze map
            move('b', 0, true);           // rotate and turn back

            while (!(x == 0 && y == 0))
            {
                if (x != x_current || y != y_current)
                {
                    think_scout(true);
                    movement_debug();
                }
            }

            brake();
            if_alt_path_back_run = false;

            TelnetStream.println("");
            TelnetStream.println("*** Alt Path Back Complete ***");
            TelnetStream.println("");

            commandManager.stop_execution();
        }
        else
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

    // Start fast run
    if (isFastRun)
    {
        if (calibrationFlag)
        {
            baseSpeed = fastSpeed;
            cellDistance = cellFastDistance;
            TelnetStream.println("");
            TelnetStream.println("*** Fast Run Started ***");

            set_maze_map_actual_run(x1, y_1, x2, y2, x3, y3, x4, y4);

            TelnetStream.println("Checking actual maze map...");

            // checking new score array for actual maze map
            for (int i = 0; i < mazeType; i++)
            {
                for (int j = 0; j < mazeType; j++)
                {
                    solved[i][j] = 0; // reinitialising solved array
                    debug_maze[i][j] = score[i][j];
                    TelnetStream.print(debug_maze[i][j]);
                }
                TelnetStream.println("");
            }

            while (!(x == x1 && y == y_1) && !(x == x2 && y == y2) && !(x == x3 && y == y3) && !(x == x4 && y == y4))
            {
                if (x != x_current || y != y_current) // basically when the x or y coordinate changes it will need to think_scout again
                {
                    think_fast_run(true); // thinkScout() while not yet at end point
                    movement_debug();
                }
            }

            brake();
            TelnetStream.println("");
            TelnetStream.println("*** Fast Run Complete ***");
            TelnetStream.println("");

            // alternate path back after fast run - COMMENT OUT IF NOT USING
            TelnetStream.println("");
            TelnetStream.println("*** Starting Path Back ***");
            if_alt_path_back_run = true;
            delay(3000);
            set_maze_map_alt_path_back(); // sets and print maze map
            move('b', 0, true);           // rotate and turn back

            while (!(x == 0 && y == 0))
            {
                if (x != x_current || y != y_current)
                {
                    think_scout(true);
                    movement_debug();
                }
            }

            brake();
            if_alt_path_back_run = false;

            TelnetStream.println("");
            TelnetStream.println("*** Alt Path Back Complete ***");
            TelnetStream.println("");

            commandManager.stop_execution();
        }

        else
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

    int wallCount = wallFront + wallRight + wallLeft;

    if (wallCount == 3) // dead end
    {
        if (if_alt_path_back_run)
        {
            if (x == 0 && y == 1)
            {
                TelnetStream.println("coordinate is: " + String(x) + String(y) + "Has reached The End, coordinate should be (0,0).");
                x = 0;
                y = 0;
                brake();
            }
        }
        else
        {
            if (debug)
            {
                TelnetStream.println("Dead End");
            }

            inDeadEnd = true; // initiate protocol
            move('b', correction, true);
        }
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

            if (if_alt_path_back_run)
            {
                scoreF = alt_path_back[u][v];
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

            if (if_alt_path_back_run)
            {
                scoreR = alt_path_back[u][v];
            }
            else
            {
                scoreR = score[u][v]; // score of the pixel in front of robot
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

            if (if_alt_path_back_run)
            {
                scoreL = alt_path_back[u][v];
            }
            else
            {
                scoreL = score[u][v]; // score of the pixel in front of robot
            }
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
    std::array<uint16_t, 4> readings = tof_read(false);

    int leftDistance = readings[3];
    int leftDiagonal = readings[2];
    int rightDiagonal = readings[1];
    int rightDistance = readings[0];

    check_available_cell(leftDiagonal, rightDiagonal);

    score[x][y]++;
    // solved[x][y] = 111; // Recording the moves made in the array

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
        rightCell = false;
        leftCell = false;

        if (debug)
        {
            TelnetStream.println("");
            TelnetStream.println("Moving back");
        }
    }
    else if (relativeDir == 'r') // Turn right
    {
        // encoder_turn_right();
        // facing = (facing + 1) % 4;

        if (rightCell)
        {
            if (rightDiagonal <= availableSpaceThreshold) // Detect next edge
            {
                encoder_turn_right();
                facing = (facing + 1) % 4;
                rightCell = false;
            }
        }
    }
    else if (relativeDir == 'l') // Turn left
    {
        // encoder_turn_left();
        // facing = (facing + 3) % 4;

        if (leftCell)
        {
            if (leftDiagonal <= availableSpaceThreshold) // Detect next edge
            {
                encoder_turn_left();
                facing = (facing + 3) % 4;
                leftCell = false;
            }
        }
    }

    // Changed it to this, so while it hasnt moved a coordinate it keeps moving forward
    while (leftEncoderValue < cellDistance && rightEncoderValue < cellDistance)
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

        forward_wall_pid(correction, baseSpeed); // Move forward
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
    if (facing == 1)
    {
        x += 1;
    }
    else if (facing == 3)
    {
        x -= 1;
    }
    else
    {
        x = x;
    }

    if (facing == 0)
    {
        y += 1;
    }
    else if (facing == 2)
    {
        y -= 1;
    }
    else
    {
        y = y;
    }

    TelnetStream.println("coordinates after moving: " + String(x) + " ," + String(y));
    TelnetStream.println("");

    if (!inDeadEnd)
    {
        solved[x][y] = 111; // Recording the moves made in the array
    }

    // Reset encoder values after each cell detection
    leftEncoderValue = 0;
    rightEncoderValue = 0;
}

void set_maze_map_scouting(int x1, int y_1, int x2, int y2, int x3, int y3, int x4, int y4)
{
    for (int i = 0; i < mazeType; i++)
    {
        for (int j = 0; j < mazeType; j++)
        {
            if (i == x1 && j == y_1 || i == x2 && j == y2 || i == x3 && j == y3 || i == x4 && j == y4)
            {
                score[i][j] = 0;
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
    for (int i = 0; i < mazeType; i++)
    {
        for (int j = 0; j < mazeType; j++)
        {
            score[i][j] = solved[i][j];
        }
    }

    for (int i = 0; i < mazeType; i++)
    {
        for (int j = 0; j < mazeType; j++)
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
            else
            {
                score[i][j] = 999;
            }
        }
    }
}

void set_maze_map_alt_path_back(void)
{
    TelnetStream.println("");
    for (int i = 0; i < mazeType; i++)
    {
        for (int j = 0; j < mazeType; j++)
        {
            solved[i][j] = 0;
            if (i == 0 && j == 0)
            {
                alt_path_back[i][j] = 0;
            }
            else
            {
                alt_path_back[i][j] = i + j;
            }
            TelnetStream.print(alt_path_back[i][j]);
        }
        TelnetStream.println("");
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

    // Create score array after scouting (correct rotation according to map)
    for (int i = 0; i < mazeType; i++)
    {
        for (int j = 0; j < mazeType; j++)
        {
            debug_maze[mazeType - 1 - j][i] = solved[i][j];

            if (solved[i][j] == 111)
            {
                debug_maze[mazeType - 1 - j][i] = 1;
            }
            else if (solved[i][j] == 999)
            {
                debug_maze[mazeType - 1 - j][i] = 9;
            }
            else
            {
                debug_maze[mazeType - 1 - j][i] = 0;
            }
        }
    }

    // Print score array
    for (int i = 0; i < mazeType; i++)
    {
        for (int j = 0; j < mazeType; j++)
        {
            TelnetStream.print(debug_maze[i][j]);
        }
        TelnetStream.println("");
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