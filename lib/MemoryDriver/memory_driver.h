#ifndef MEMORY_DRIVER_H
#define MEMORY_DRIVER_H

#include <Arduino.h>
#include <EEPROM.h>
#include <TelnetStream.h>

#define EEPROM_SIZE 448  // Must be >= 392 for a 14x14 int array
#define ROWS 14
#define COLS 14

/* Variables */
extern volatile float lastWallError;
extern volatile float wallIntegral;
extern volatile int leftEncoderValue;
extern volatile int rightEncoderValue;

void memory_setup(void);
void memory_store(int data);
int memory_read(int address);
void memory_clear(int address);

#endif