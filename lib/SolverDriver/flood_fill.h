#ifndef FLOOD_FILL_H
#define FLOOD_FILL_H

#include <Arduino.h>
#include <TelnetStream.h>

/* Variables */
extern volatile float lastWallError;
extern volatile float wallIntegral;
extern volatile int leftEncoderValue;
extern volatile int rightEncoderValue;

void flood_fill(void);                                     // Flood fill program
void think_scout(bool debug);               // Movement Logic (Greed)
void move(char relativeDir, float correction, bool debug); // Moves jerry in the direction of relativeDir (f, l, r)
void set_maze_map_scouting(int x1, int y_1, int x2, int y2, int x3, int y3, int x4, int y4);
void set_maze_map_actual_run(int x1, int y_1, int x2, int y2, int x3, int y3, int x4, int y4);
void set_maze_map_alt_path_back(void);
void think_fast_run(bool debug); // Movement logic for fast run
void movement_debug(void);
void check_available_cell(int leftDiagonal, int rightDiagonal);

#endif