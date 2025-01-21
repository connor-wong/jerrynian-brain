#ifndef WALL_FOLLOWER_H
#define WALL_FOLLOWER_H

#include <Arduino.h>
#include <TelnetStream.h>

extern volatile float lastWallError;
extern volatile float wallIntegral;

void wall_follower(void);
void check_available_cell(int leftDiagonal, int rightDiagonal);

#endif